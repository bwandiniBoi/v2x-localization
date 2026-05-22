# V2X Localization Project

## What this project is

A CV2X (Cellular Vehicle-to-Everything) system that uses **RSSI-based trilateration** to
estimate the real-time position of a moving vehicle (OBU) using three fixed roadside units
(RSUs). Built on Raspberry Pi 4 devices with Autotalks SECTON CV2X radio hardware.

The system uses the **Vanetza** open-source V2X protocol stack (based on ETSI ITS standards)
to send and receive CAM (Cooperative Awareness Message) beacons over the CV2X radio.

---

## Hardware setup

| Device | Role | Count |
|--------|------|-------|
| Raspberry Pi 4 + Autotalks SECTON | RSU (fixed, known position) | 3 |
| Raspberry Pi 4 + Autotalks SECTON | OBU (moving vehicle) | 1 |
| Laptop (any OS) | Data aggregation + live map | 1 |

All 4 Pis are **identical hardware** running the same OS and the same compiled binary
(`socktap`). The role (RSU or OBU) is chosen at runtime by which `--applications` flag
you pass.

---

## Architecture (new design — what this codebase implements)

```
OBU Pi
└── broadcasts CAM beacons containing its own GPS position every ~1 s
    (uses --applications ca)

RSU Pi #1  ──┐
RSU Pi #2  ──┼──  each RSU receives OBU CAMs, measures RSSI of the signal,
RSU Pi #3  ──┘    and streams a JSON measurement UDP packet to the laptop
                  (uses --applications rsu-rx)

Laptop
└── laptop_receiver.py
    ├── receives UDP from all 3 RSUs on port 5005
    ├── converts RSSI → estimated distance (log-distance path loss model)
    ├── runs least-squares trilateration to estimate OBU position
    └── serves a live Leaflet.js/OpenStreetMap at http://localhost:8080
```

### Why this direction (RSU receives, OBU broadcasts)?
The old code had RSUs broadcasting their position and the OBU doing trilateration
on-device. The new design flips this: RSUs are at known fixed positions, so they
measure the RSSI from the OBU's broadcast and report it to the laptop. This lets
the laptop do the trilateration with data from all 3 RSUs simultaneously and
display a live track.

---

## Repository layout (key files)

```
vanetza/                          ← this repo (bwandiniBoi/v2x-localization)
└── tools/socktap/
    ├── main.cpp                  ← entry point, CLI option definitions, app wiring
    ├── cam_application.cpp/.hpp  ← OBU mode: broadcasts own GPS as CAM every 1 s
    ├── rsu_receiver_application.cpp/.hpp  ← NEW: RSU mode, receives OBU CAMs,
    │                                         measures RSSI, streams JSON to laptop
    ├── rsu_application.cpp/.hpp  ← OLD RSU mode (broadcasts RSU GPS) — kept for reference
    ├── vehicle_localizer_application.cpp/.hpp  ← OLD OBU mode — kept for reference
    ├── rssi_cache.hpp            ← global singleton storing the latest RSSI per MAC
    ├── rpc_link.cpp/.hpp         ← bridges Vanetza to the RPC server; populates RSSICache
    └── laptop_receiver.py        ← run this on the laptop (Flask + numpy)
```

The `rpc_server` / `vanetza_rpc_bridge` directory contains the **RPC server** binary
(`cv2x_rpc_server`) that owns the Autotalks hardware. It must be started before `socktap`.
That code is tracked separately at `github.com/cookiedough0/vanetza_rpc_bridge`.

---

## How to build (on each Raspberry Pi)

All dependencies (`capnp`, `boost`, `gpsd`, `eigen3`) are standard system packages and
should already be installed on all 4 Pis.

### If the Pi already has the old vanetza directory:
```bash
cd /home/eee-rpi/cv2x_testing/vanetza
git remote set-url origin https://github.com/bwandiniBoi/v2x-localization.git
git config user.email "limshengzebrandon@gmail.com"
git config user.name "bwandiniBoi"
git pull origin master
cd build
make socktap -j$(nproc)
```

### If starting fresh on a Pi:
```bash
cd /home/eee-rpi/cv2x_testing
git clone https://github.com/bwandiniBoi/v2x-localization.git vanetza
cd vanetza
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
         -DSOCKTAP_WITH_RPC=ON \
         -DSOCKTAP_WITH_GPSD=ON
make socktap -j$(nproc)
```

Built binary lands at: `build/bin/socktap`

---

## How to run

### IMPORTANT: start the RPC server FIRST on every Pi
The RPC server initialises the Autotalks SECTON radio chip. `socktap` connects to it
as a client on `localhost:23057`. If `socktap` starts before the RPC server is ready,
it will fail to connect.

```bash
# Terminal 1 on every Pi (RSU or OBU)
cd /home/eee-rpi/cv2x_testing/vanetza_rpc_bridge/build
./cv2x_rpc_server
# Wait until you see: "RPC server ready! Waiting for connections..."
```

### RSU Pis (3 of them — use --rsu-id 1, 2, 3 respectively)
```bash
# Terminal 2
cd /home/eee-rpi/cv2x_testing/vanetza/build/bin
./socktap --link-layer rpc --applications rsu-rx \
          --rsu-id 1 \
          --laptop-ip <laptop-ip-address> \
          --laptop-port 5005
```

### OBU Pi
```bash
# Terminal 2
cd /home/eee-rpi/cv2x_testing/vanetza/build/bin
./socktap --link-layer rpc --applications ca
```

### Laptop
```bash
# Install dependencies once
pip install flask numpy

# Run
cd vanetza/tools
python3 laptop_receiver.py

# Open browser at http://localhost:8080
```

---

## Laptop map legend

| Colour | Meaning |
|--------|---------|
| Blue dot | RSU fixed position (appears once per RSU on first packet) |
| Red dot | OBU estimated position (from trilateration) |
| Green dot | OBU GPS ground truth (from the CAM message itself) |

The panel also shows RSSI per RSU and localisation error (distance between red and
green dots in metres).

---

## Path loss model (needs calibration)

RSSI is converted to distance using the log-distance path loss model:

```
distance = 10 ^ ((A - RSSI) / (10 * n))
```

Current defaults in `laptop_receiver.py`:
- `PATH_LOSS_A = -40.0` dBm — measured RSSI at exactly 1 metre
- `PATH_LOSS_N = 2.5` — path loss exponent

**These must be calibrated for your environment.** To do it:
1. Place an RSU and OBU exactly 1 metre apart, note the average RSSI → set `PATH_LOSS_A`
2. Move the OBU to known distances (5 m, 10 m, 20 m) and fit `PATH_LOSS_N` to minimise
   the error between estimated and actual distance

---

## JSON format sent from each RSU to laptop (UDP)

One newline-terminated JSON object per received OBU packet:

```json
{
  "ts":      1716400000123,
  "rsu_id":  1,
  "rsu_lat": 1.3456789,
  "rsu_lon": 103.1234567,
  "obu_id":  1,
  "obu_lat": 1.3456780,
  "obu_lon": 103.1234560,
  "rssi":    -75
}
```

`obu_lat`/`obu_lon` is the OBU's self-reported GPS position from the CAM — useful as
ground truth to measure localisation accuracy.

---

## Known limitations / TODO

- **Trilateration not yet implemented on-Pi** — the old `vehicle_localizer_application`
  has a `perform_trilateration()` stub that never computes a real position. Trilateration
  now lives in `laptop_receiver.py` instead.
- **Path loss constants need field calibration** before results are meaningful.
- **RSSI cache is global** — if two packets arrive very close together, the RSSI for one
  may be incorrectly paired with the CAM from another. Fine at 1 Hz CAM rate.
- **No authentication or encryption** on the UDP stream to the laptop.
- The laptop script uses Flask's built-in dev server — fine for field testing, not for
  production.

---

## GitHub

- Main repo (this one): `https://github.com/bwandiniBoi/v2x-localization`
- RPC bridge (separate): `https://github.com/cookiedough0/vanetza_rpc_bridge`
- Upstream Vanetza library: `https://github.com/riebl/vanetza`
