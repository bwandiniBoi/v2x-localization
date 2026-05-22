#pragma once

#include "application.hpp"
#include <vanetza/common/position_provider.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/ip/udp.hpp>
#include <string>
#include <cstdint>

/*
 * RsuReceiverApplication — new-architecture RSU role.
 *
 * Listens for CAM beacons from the OBU, reads the RSSI of each received
 * packet, and forwards a JSON measurement record to the laptop over UDP so
 * the laptop can run trilateration across all three RSUs.
 *
 * Data sent per received OBU packet (newline-terminated JSON):
 *   { "ts":<ms>, "rsu_id":<n>, "rsu_lat":<deg>, "rsu_lon":<deg>,
 *     "obu_id":<n>, "obu_lat":<deg>, "obu_lon":<deg>, "rssi":<dBm> }
 */
class RsuReceiverApplication : public Application, public Application::PromiscuousHook
{
public:
    RsuReceiverApplication(
        vanetza::PositionProvider& positioning,
        boost::asio::io_context& io,
        uint32_t rsu_id,
        const std::string& laptop_ip,
        uint16_t laptop_port
    );

    PortType port() override;
    Application::PromiscuousHook* promiscuous_hook() override;

    // Required by Application interface; actual work happens in tap_packet.
    void indicate(const DataIndication&, UpPacketPtr) override;

    // Called for every received packet before the stack processes it.
    void tap_packet(const DataIndication&, const vanetza::UpPacket&) override;

private:
    void send_to_laptop(
        double rsu_lat, double rsu_lon,
        uint32_t obu_id, double obu_lat, double obu_lon,
        int16_t rssi_dbm
    );

    vanetza::PositionProvider& positioning_;
    boost::asio::ip::udp::socket udp_socket_;
    boost::asio::ip::udp::endpoint laptop_endpoint_;
    uint32_t rsu_id_;
};
