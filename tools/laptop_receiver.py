#!/usr/bin/env python3
"""
laptop_receiver.py

Receives RSSI measurement JSON datagrams from 3 RSU Raspberry Pis over UDP,
runs trilateration to estimate the OBU position, and serves a live Leaflet.js
map at http://localhost:8080.

Dependencies:
    pip install flask numpy

Run:
    python3 laptop_receiver.py
    python3 laptop_receiver.py --port 5005 --web-port 8080  (same, explicit)

Each RSU sends newline-terminated JSON like:
    {"ts":1716400000123, "rsu_id":1, "rsu_lat":1.34, "rsu_lon":103.12,
     "obu_id":1, "obu_lat":1.341, "obu_lon":103.121, "rssi":-75}
"""

import argparse
import json
import math
import socket
import threading
import time

from flask import Flask, jsonify

# ---------------------------------------------------------------------------
# Path loss model parameters
# Tune these by doing a calibration walk:
#   - Stand 1 metre from an RSU, note the RSSI -> that is PATH_LOSS_A
#   - PATH_LOSS_N: 2.0 = free space, 2.5-3.0 = open outdoor, 3.5 = urban
# ---------------------------------------------------------------------------
PATH_LOSS_A = -40.0   # RSSI (dBm) at 1 metre reference distance
PATH_LOSS_N = 2.5     # Path loss exponent

# ---------------------------------------------------------------------------
# Shared state between UDP thread and Flask thread.
# Always acquire state_lock before reading or writing these.
# ---------------------------------------------------------------------------
latest_measurements = {}   # { rsu_id (int) -> measurement dict }
latest_estimate     = None  # { "lat": float, "lon": float, "ts": int }
state_lock          = threading.Lock()


# ===========================================================================
# 1.  TRILATERATION MATHS
# ===========================================================================

def rssi_to_distance(rssi_dbm):
    """
    Log-distance path loss model:
        RSSI(d) = A - 10*n*log10(d)
    Rearranging for d:
        d = 10 ^ ((A - RSSI) / (10 * n))
    """
    return 10.0 ** ((PATH_LOSS_A - rssi_dbm) / (10.0 * PATH_LOSS_N))


def latlon_to_xy(lat, lon, ref_lat, ref_lon):
    """
    Convert (lat, lon) degrees to local (x, y) metres relative to a reference
    point.  Uses the equirectangular (flat-earth) approximation — accurate to
    better than 0.1 % for distances up to ~10 km, which is fine here.

        x (east)  = Δlon * cos(ref_lat) * R
        y (north) = Δlat * R
    """
    R = 6_371_000.0   # Earth radius in metres
    x = math.radians(lon - ref_lon) * math.cos(math.radians(ref_lat)) * R
    y = math.radians(lat - ref_lat) * R
    return x, y


def xy_to_latlon(x, y, ref_lat, ref_lon):
    """Inverse of latlon_to_xy."""
    R = 6_371_000.0
    lat = ref_lat + math.degrees(y / R)
    lon = ref_lon + math.degrees(x / (math.cos(math.radians(ref_lat)) * R))
    return lat, lon


def trilaterate(measurements):
    """
    Estimate position from 3+ (lat, lon, distance_m) tuples.

    The key insight: each RSU gives a circle (known centre, known radius).
    The OBU is at the intersection of the three circles.

    Exact algebra:
      Circle i:  (x - xi)^2 + (y - yi)^2 = di^2

    Subtract circle 1 from circle i to cancel the squared unknowns:
      2(xi - x1)*x + 2(yi - y1)*y = di^2 - d1^2 + x1^2 - xi^2 + y1^2 - yi^2

    This gives a LINEAR system  A * [x, y]^T = b  which numpy solves.
    With more than 3 RSUs we get an overdetermined system; numpy's lstsq()
    finds the least-squares best fit automatically.

    Returns (lat, lon) or None if the system can't be solved.
    """
    import numpy as np

    if len(measurements) < 3:
        return None

    # Use the first RSU as the local coordinate origin
    ref_lat, ref_lon, _ = measurements[0]

    points    = []
    distances = []
    for lat, lon, dist in measurements:
        x, y = latlon_to_xy(lat, lon, ref_lat, ref_lon)
        points.append((x, y))
        distances.append(dist)

    x1, y1 = points[0]
    d1      = distances[0]

    A_rows, b_rows = [], []
    for i in range(1, len(points)):
        xi, yi = points[i]
        di     = distances[i]
        A_rows.append([2 * (xi - x1), 2 * (yi - y1)])
        b_rows.append(d1**2 - di**2 + xi**2 - x1**2 + yi**2 - y1**2)

    A = np.array(A_rows, dtype=float)
    b = np.array(b_rows, dtype=float)

    try:
        result, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
        est_x, est_y    = result
        return xy_to_latlon(est_x, est_y, ref_lat, ref_lon)
    except np.linalg.LinAlgError:
        return None


def haversine_m(lat1, lon1, lat2, lon2):
    """Straight-line distance in metres between two (lat, lon) points."""
    R = 6_371_000.0
    d_lat = math.radians(lat2 - lat1)
    d_lon = math.radians(lon2 - lon1)
    a = (math.sin(d_lat / 2) ** 2 +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
         math.sin(d_lon / 2) ** 2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


# ===========================================================================
# 2.  UDP LISTENER  (runs in its own thread)
# ===========================================================================

def udp_listener(port):
    """
    Blocks on a UDP socket, parses each JSON datagram from an RSU, then
    updates latest_measurements and (if we have ≥ 3 RSUs) latest_estimate.

    This runs as a daemon thread so it dies automatically when the main
    process exits — no cleanup needed.
    """
    global latest_estimate

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(1.0)   # wake up every second so KeyboardInterrupt works
    print(f"[UDP] Listening on 0.0.0.0:{port}")

    while True:
        try:
            data, addr = sock.recvfrom(4096)
        except socket.timeout:
            continue    # just loop — gives SIGINT a chance to land

        try:
            msg = json.loads(data.decode("utf-8").strip())
        except (json.JSONDecodeError, UnicodeDecodeError) as e:
            print(f"[UDP] Bad packet from {addr}: {e}")
            continue

        rsu_id = msg.get("rsu_id")
        if rsu_id is None:
            continue

        # Print a one-liner so you can see data arriving in the terminal
        dist_m = rssi_to_distance(msg["rssi"])
        print(f"[UDP] RSU {rsu_id}  RSSI={msg['rssi']:4d} dBm  "
              f"est_dist={dist_m:6.1f} m  "
              f"OBU@({msg['obu_lat']:.6f}, {msg['obu_lon']:.6f})")

        with state_lock:
            latest_measurements[rsu_id] = {
                "rsu_lat": msg["rsu_lat"],
                "rsu_lon": msg["rsu_lon"],
                "obu_lat": msg["obu_lat"],   # OBU's own GPS — used as ground truth
                "obu_lon": msg["obu_lon"],
                "rssi":    msg["rssi"],
                "ts":      msg["ts"],
            }

            # Expire measurements older than 5 seconds so a crashed RSU
            # doesn't poison the trilateration with stale data.
            now_ms = int(time.time() * 1000)
            latest_measurements = {
                k: v for k, v in latest_measurements.items()
                if now_ms - v["ts"] < 5000
            }

            # Run trilateration when we have fresh data from at least 3 RSUs
            if len(latest_measurements) >= 3:
                meas_list = [
                    (v["rsu_lat"], v["rsu_lon"], rssi_to_distance(v["rssi"]))
                    for v in latest_measurements.values()
                ]
                result = trilaterate(meas_list)
                if result:
                    latest_estimate = {
                        "lat": result[0],
                        "lon": result[1],
                        "ts":  msg["ts"],
                    }
                    # Print error vs OBU's own GPS so you can judge accuracy
                    err = haversine_m(result[0], result[1],
                                      msg["obu_lat"], msg["obu_lon"])
                    print(f"[TRILAT] Estimate ({result[0]:.6f}, {result[1]:.6f})  "
                          f"error={err:.1f} m vs OBU GPS")
            else:
                remaining = 3 - len(latest_measurements)
                print(f"[TRILAT] Waiting for {remaining} more RSU(s)...")


# ===========================================================================
# 3.  FLASK WEB SERVER  — serves the live map at http://localhost:8080
# ===========================================================================

app = Flask(__name__)

# The whole front-end lives in this HTML string.
# Leaflet.js is loaded from a CDN so you don't need to install anything.
MAP_HTML = """<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8"/>
  <title>V2X Live Tracking</title>
  <link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css"/>
  <script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
  <style>
    body { margin: 0; font-family: sans-serif; }
    #map { height: 100vh; }
    #panel {
      position: absolute; top: 10px; right: 10px; z-index: 1000;
      background: white; padding: 14px; border-radius: 8px;
      box-shadow: 0 2px 10px rgba(0,0,0,0.3); min-width: 240px;
      max-width: 280px;
    }
    h3  { margin: 0 0 10px 0; font-size: 15px; }
    .row { margin: 5px 0; font-size: 13px; line-height: 1.4; }
    .lbl { font-weight: bold; }
    .sep { border: none; border-top: 1px solid #ddd; margin: 8px 0; }
    /* coloured legend dots */
    .dot { display:inline-block; width:10px; height:10px;
           border-radius:50%; margin-right:5px; vertical-align:middle; }
    .blue  { background:#2196F3; }
    .red   { background:#F44336; }
    .green { background:#4CAF50; }
  </style>
</head>
<body>
<div id="map"></div>

<div id="panel">
  <h3>V2X Live Tracking</h3>

  <div class="row">
    <span class="dot blue"></span><span class="lbl">RSUs online:</span>
    <span id="rsu-count">0 / 3</span>
  </div>

  <hr class="sep"/>

  <div class="row">
    <span class="dot red"></span><span class="lbl">Estimated position</span>
  </div>
  <div class="row" id="est-pos" style="color:#666">Waiting for 3 RSUs…</div>

  <hr class="sep"/>

  <div class="row">
    <span class="dot green"></span><span class="lbl">OBU GPS (ground truth)</span>
  </div>
  <div class="row" id="gps-pos" style="color:#666">—</div>

  <hr class="sep"/>

  <div class="row"><span class="lbl">Localisation error:</span>
    <span id="error">—</span>
  </div>

  <hr class="sep"/>

  <div class="row" id="rssi-list" style="color:#555"></div>
</div>

<script>
  // ------------------------------------------------------------------
  // Set up the Leaflet map centred roughly on Singapore as a default.
  // It will re-centre automatically once the first RSU data arrives.
  // ------------------------------------------------------------------
  const map = L.map('map').setView([1.3521, 103.8198], 17);
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
    maxZoom: 20,
    attribution: '© OpenStreetMap contributors'
  }).addTo(map);

  // Small coloured circle markers (no default Leaflet pin icon)
  function circleIcon(color) {
    return L.divIcon({
      className: '',
      html: `<div style="background:${color};width:14px;height:14px;
                         border-radius:50%;border:2px solid #fff;
                         box-shadow:0 0 4px rgba(0,0,0,0.5)"></div>`,
      iconAnchor: [7, 7]
    });
  }

  const rsuIcon  = circleIcon('#2196F3');   // blue  — RSU fixed positions
  const obuIcon  = circleIcon('#F44336');   // red   — our trilateration estimate
  const gpsIcon  = circleIcon('#4CAF50');   // green — OBU's own GPS (ground truth)

  let rsuMarkers = {};   // RSU markers indexed by rsu_id
  let obuMarker  = null;
  let gpsMarker  = null;
  let mapCentred = false;

  // Haversine distance in metres — used to display localisation error
  function haversine(lat1, lon1, lat2, lon2) {
    const R = 6371000;
    const dLat = (lat2 - lat1) * Math.PI / 180;
    const dLon = (lon2 - lon1) * Math.PI / 180;
    const a = Math.sin(dLat/2)**2 +
              Math.cos(lat1*Math.PI/180) * Math.cos(lat2*Math.PI/180) *
              Math.sin(dLon/2)**2;
    return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));
  }

  // ------------------------------------------------------------------
  // refresh() — fetches /data from Flask and updates the map + panel.
  // Called every 1 second by setInterval below.
  // ------------------------------------------------------------------
  function refresh() {
    fetch('/data')
      .then(r => r.json())
      .then(d => {

        // ---- RSU markers (blue) ----
        const rsuIds = Object.keys(d.rsus);
        document.getElementById('rsu-count').textContent =
          rsuIds.length + ' / 3';

        let rssiHtml = '<b>RSSI readings:</b><br/>';
        for (const [id, rsu] of Object.entries(d.rsus)) {
          // Add marker on first sight, then leave it (RSUs are fixed)
          if (!rsuMarkers[id]) {
            rsuMarkers[id] = L.marker([rsu.rsu_lat, rsu.rsu_lon], {icon: rsuIcon})
              .addTo(map)
              .bindPopup(`<b>RSU ${id}</b><br/>${rsu.rsu_lat.toFixed(6)},
                          ${rsu.rsu_lon.toFixed(6)}`);
          }
          rssiHtml += `RSU ${id}: ${rsu.rssi} dBm<br/>`;
        }
        document.getElementById('rssi-list').innerHTML = rssiHtml;

        // ---- Estimated OBU position (red) ----
        if (d.estimate) {
          const pos = [d.estimate.lat, d.estimate.lon];
          if (!obuMarker) {
            obuMarker = L.marker(pos, {icon: obuIcon})
              .addTo(map)
              .bindPopup('<b>OBU — estimated</b>');
            // Centre the map on first estimate
            if (!mapCentred) { map.setView(pos, 18); mapCentred = true; }
          } else {
            obuMarker.setLatLng(pos);
          }
          document.getElementById('est-pos').textContent =
            d.estimate.lat.toFixed(6) + ', ' + d.estimate.lon.toFixed(6);
        }

        // ---- OBU ground-truth GPS (green) ----
        if (d.obu_gps) {
          const gpos = [d.obu_gps.lat, d.obu_gps.lon];
          if (!gpsMarker) {
            gpsMarker = L.marker(gpos, {icon: gpsIcon})
              .addTo(map)
              .bindPopup('<b>OBU — GPS ground truth</b>');
          } else {
            gpsMarker.setLatLng(gpos);
          }
          document.getElementById('gps-pos').textContent =
            d.obu_gps.lat.toFixed(6) + ', ' + d.obu_gps.lon.toFixed(6);

          // Localisation error (only meaningful when we also have an estimate)
          if (d.estimate) {
            const err = haversine(d.estimate.lat, d.estimate.lon,
                                   d.obu_gps.lat,  d.obu_gps.lon);
            document.getElementById('error').textContent =
              err.toFixed(1) + ' m';
          }
        }
      })
      .catch(err => console.warn('fetch /data failed:', err));
  }

  // Poll every second
  setInterval(refresh, 1000);
  refresh();   // first call immediately on page load
</script>
</body>
</html>
"""


@app.route("/")
def index():
    """Serve the map page."""
    return MAP_HTML


@app.route("/data")
def data():
    """
    JSON endpoint polled by the browser every second.
    Returns:
        rsus      — latest measurement from each RSU
        estimate  — trilaterated OBU position (or null)
        obu_gps   — OBU's own GPS position from the CAM (ground truth)
    """
    with state_lock:
        rsus     = dict(latest_measurements)
        estimate = dict(latest_estimate) if latest_estimate else None

    # Use any RSU's reported OBU GPS as the ground truth reference
    obu_gps = None
    if rsus:
        sample  = next(iter(rsus.values()))
        obu_gps = {"lat": sample["obu_lat"], "lon": sample["obu_lon"]}

    return jsonify({"rsus": rsus, "estimate": estimate, "obu_gps": obu_gps})


# ===========================================================================
# 4.  ENTRY POINT
# ===========================================================================

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="V2X laptop receiver — trilateration + live map"
    )
    parser.add_argument("--port",     type=int, default=5005,
                        help="UDP port to receive RSU measurements (default 5005)")
    parser.add_argument("--web-port", type=int, default=8080,
                        help="HTTP port for the live map (default 8080)")
    args = parser.parse_args()

    # Start the UDP listener as a background daemon thread.
    # daemon=True means Python won't wait for it on exit — Ctrl+C kills everything.
    udp_thread = threading.Thread(
        target=udp_listener, args=(args.port,), daemon=True
    )
    udp_thread.start()

    print(f"[WEB] Open  http://localhost:{args.web_port}  in your browser")
    print(f"[WEB] (also accessible from other devices on the same network)")

    # Flask's built-in server is fine for this use case.
    # use_reloader=False is important — reloader would start a second UDP thread.
    app.run(host="0.0.0.0", port=args.web_port, debug=False, use_reloader=False)
