#pragma once

#include "application.hpp"
#include <vanetza/common/position_provider.hpp>
#include <vanetza/common/runtime.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <map>
#include <chrono>
#include <fstream>  // ADD THIS LINE


class VehicleLocalizerApplication : public Application, public Application::PromiscuousHook
{
public:
    VehicleLocalizerApplication(vanetza::PositionProvider& positioning,
                                vanetza::Runtime& rt,
                                boost::asio::io_context& io);
    
    PortType port() override;
    Application::PromiscuousHook* promiscuous_hook() override;
    void indicate(const DataIndication&, UpPacketPtr) override;
    void tap_packet(const DataIndication&, const vanetza::UpPacket&) override;

private:
    struct RSUMeasurement {
        uint32_t rsu_id;
        double latitude;
        double longitude;
        int16_t rssi_dbm;
        std::chrono::system_clock::time_point timestamp;
    };
    
    void schedule_timer();
    void on_timer(const boost::system::error_code& ec);
    void perform_trilateration();
    double rssi_to_distance(int16_t rssi_dbm);
    void log_measurements();
    
    vanetza::PositionProvider& positioning_;
    vanetza::Runtime& runtime_;
    boost::asio::steady_timer timer_;
    std::chrono::seconds interval_;
    
    // RSU measurements for trilateration
    std::map<uint32_t, RSUMeasurement> rsu_measurements_;
    
    // Path loss model parameters (calibrate these!)
    const double path_loss_A_ = -40.0;  // Reference power at 1m (dBm)
    const double path_loss_n_ = 2.5;    // Path loss exponent

    // CSV logging
    std::ofstream rssi_log_file_;      // ADD THIS
    std::ofstream position_log_file_;  // ADD THIS
};
