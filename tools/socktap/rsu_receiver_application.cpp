#include "rsu_receiver_application.hpp"
#include "rssi_cache.hpp"
#include <vanetza/btp/ports.hpp>
#include <vanetza/asn1/cam.hpp>
#include <vanetza/asn1/packet_visitor.hpp>
#include <boost/asio/buffer.hpp>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <cmath>

using namespace vanetza;

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
RsuReceiverApplication::RsuReceiverApplication(
    vanetza::PositionProvider& positioning,
    boost::asio::io_context& io,
    uint32_t rsu_id,
    const std::string& laptop_ip,
    uint16_t laptop_port)
    : positioning_(positioning),
      udp_socket_(io, boost::asio::ip::udp::v4()),
      rsu_id_(rsu_id)
{
    // Resolve the laptop's address once at startup so every send is fast.
    laptop_endpoint_ = boost::asio::ip::udp::endpoint(
        boost::asio::ip::address::from_string(laptop_ip),
        laptop_port
    );

    std::cout << "[RSU-RX] RSU #" << rsu_id_
              << " listening for OBU CAMs, forwarding to "
              << laptop_ip << ":" << laptop_port << std::endl;
}

// ---------------------------------------------------------------------------
// Application interface
// ---------------------------------------------------------------------------

RsuReceiverApplication::PortType RsuReceiverApplication::port()
{
    // Listen on the CAM BTP port — same port the OBU transmits on.
    return btp::ports::CAM;
}

Application::PromiscuousHook* RsuReceiverApplication::promiscuous_hook()
{
    // Returning `this` tells the router to call tap_packet() for EVERY
    // received packet, not just those addressed to our BTP port.  We need
    // this because RSSI is stored in a global cache by rpc_link.cpp at the
    // moment the hardware delivers the frame — we must read it immediately
    // before the next frame overwrites it.
    return this;
}

void RsuReceiverApplication::indicate(const DataIndication&, UpPacketPtr)
{
    // All work is done in tap_packet(); this can stay empty.
}

// ---------------------------------------------------------------------------
// tap_packet — called for every received frame with RSSI already in cache
// ---------------------------------------------------------------------------
void RsuReceiverApplication::tap_packet(
    const DataIndication& /*indication*/,
    const vanetza::UpPacket& packet)
{
    // Step 1: Try to decode the packet as a CAM message.
    //
    // Vanetza uses a "visitor" pattern here: we ask the packet to apply a
    // typed visitor that tries to deserialise ASN.1 CAM bytes.  If the
    // packet is not a valid CAM, `cam` will be null and we bail out.
    asn1::PacketVisitor<asn1::Cam> visitor;
    std::shared_ptr<const asn1::Cam> cam = boost::apply_visitor(visitor, packet);
    if (!cam) return;

    const CAM_t& cam_msg = **cam;
    const BasicContainer_t& basic = cam_msg.cam.camParameters.basicContainer;

    // Step 2: Ignore beacons from other RSUs — we only want OBU (vehicle) CAMs.
    //
    // The CAM standard encodes the sender type in stationType.
    // StationType_roadSideUnit == 15 in the ASN.1 enum.
    if (basic.stationType == StationType_roadSideUnit) return;

    // Step 3: Extract OBU identity and GPS position from the CAM.
    //
    // CAM positions are stored in 1/10 micro-degree units, so we divide by
    // 10,000,000 to get decimal degrees.
    uint32_t obu_id  = cam_msg.header.stationID;
    double obu_lat   = basic.referencePosition.latitude  / 10000000.0;
    double obu_lon   = basic.referencePosition.longitude / 10000000.0;

    // Step 4: Read RSSI from the global cache.
    //
    // rpc_link.cpp populates RSSICache every time the Autotalks hardware
    // delivers a frame.  tap_packet() fires immediately after, so the cache
    // should always hold the RSSI for this frame.
    auto rssi_entry = RSSICache::instance().get_last();
    if (!rssi_entry) {
        std::cout << "[RSU-RX] OBU " << obu_id << " packet received but RSSI unavailable" << std::endl;
        return;
    }
    int16_t rssi_dbm = rssi_entry->rssi_dbm;

    // Step 5: Get this RSU's own GPS position from the position provider.
    //
    // The RSU is stationary, so this value should be stable once the GPS
    // has a fix.  We skip forwarding if the fix isn't ready yet.
    auto rsu_pos = positioning_.position_fix();
    if (!std::isfinite(rsu_pos.latitude.value()) || !std::isfinite(rsu_pos.longitude.value())) {
        std::cerr << "[RSU-RX] Waiting for RSU GPS fix — skipping" << std::endl;
        return;
    }
    double rsu_lat = rsu_pos.latitude.value();
    double rsu_lon = rsu_pos.longitude.value();

    std::cout << "[RSU-RX] OBU " << obu_id
              << " @ (" << std::fixed << std::setprecision(7) << obu_lat << ", " << obu_lon << ")"
              << "  RSSI=" << rssi_dbm << " dBm" << std::endl;

    // Step 6: Send everything to the laptop.
    send_to_laptop(rsu_lat, rsu_lon, obu_id, obu_lat, obu_lon, rssi_dbm);
}

// ---------------------------------------------------------------------------
// send_to_laptop — serialise one measurement and fire it over UDP
// ---------------------------------------------------------------------------
void RsuReceiverApplication::send_to_laptop(
    double rsu_lat, double rsu_lon,
    uint32_t obu_id, double obu_lat, double obu_lon,
    int16_t rssi_dbm)
{
    // Timestamp in milliseconds since Unix epoch — easy to parse in Python
    // with datetime.fromtimestamp(ts / 1000).
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();

    // We build a newline-terminated JSON string by hand to avoid pulling in
    // a JSON library.  The format is deliberately flat so Python can parse
    // it with a single json.loads() call.
    std::ostringstream json;
    json << std::fixed << std::setprecision(7)
         << "{"
         << "\"ts\":"      << now_ms   << ","
         << "\"rsu_id\":"  << rsu_id_  << ","
         << "\"rsu_lat\":" << rsu_lat  << ","
         << "\"rsu_lon\":" << rsu_lon  << ","
         << "\"obu_id\":"  << obu_id   << ","
         << "\"obu_lat\":" << obu_lat  << ","
         << "\"obu_lon\":" << obu_lon  << ","
         << "\"rssi\":"    << rssi_dbm
         << "}\n";

    std::string msg = json.str();

    try {
        udp_socket_.send_to(boost::asio::buffer(msg), laptop_endpoint_);
    } catch (const boost::system::system_error& e) {
        std::cerr << "[RSU-RX] UDP send failed: " << e.what() << std::endl;
    }
}
