#include "vehicle_localizer_application.hpp"
#include <vanetza/btp/ports.hpp>
#include <vanetza/asn1/cam.hpp>
#include <vanetza/asn1/packet_visitor.hpp>
#include <fstream>      // ADD THIS - for ofstream
#include <sstream>      // ADD THIS - for stringstream
#include <iostream>
#include <iomanip>
#include <cmath>
#include "rssi_cache.hpp"

using namespace vanetza;

VehicleLocalizerApplication::VehicleLocalizerApplication(
    PositionProvider& positioning,
    Runtime& rt,
    boost::asio::io_context& io) :
    positioning_(positioning),
    runtime_(rt),
    timer_(io),
    interval_(std::chrono::seconds(2))
{
    // Open CSV log files with timestamp
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
    std::string timestamp = ss.str();
    
    // RSSI measurements log
    std::string rssi_filename = "vehicle_rssi_" + timestamp + ".csv";
    rssi_log_file_.open(rssi_filename);
    rssi_log_file_ << "timestamp,rsu_id,latitude,longitude,rssi_dbm,distance_m" << std::endl;
    std::cout << "Logging RSSI to: " << rssi_filename << std::endl;
    
    // Position estimates log (for future trilateration)
    std::string pos_filename = "vehicle_position_" + timestamp + ".csv";
    position_log_file_.open(pos_filename);
    position_log_file_ << "timestamp,num_rsus,gps_lat,gps_lon,estimated_lat,estimated_lon" << std::endl;
    std::cout << "Logging positions to: " << pos_filename << std::endl;


    schedule_timer();
    std::cout << "Vehicle Localizer initialized - Ready for trilateration with RSSI" << std::endl;
}

VehicleLocalizerApplication::PortType VehicleLocalizerApplication::port()
{
    return btp::ports::CAM;
}

Application::PromiscuousHook* VehicleLocalizerApplication::promiscuous_hook()
{
    return this;
}

void VehicleLocalizerApplication::indicate(const DataIndication& indication, UpPacketPtr packet)
{
    // Processed via promiscuous hook with RSSI data
}

void VehicleLocalizerApplication::tap_packet(const DataIndication& indication, const vanetza::UpPacket& packet)
{
    asn1::PacketVisitor<asn1::Cam> visitor;
    std::shared_ptr<const asn1::Cam> cam = boost::apply_visitor(visitor, packet);

    if (!cam) {
        return;
    }
    
    // Dereference to get CAM_t structure
    const CAM_t& cam_msg = **cam;
    
    // Extract header info
    uint32_t station_id = cam_msg.header.stationID;
    
    // Extract basic container
    const BasicContainer_t& basic = cam_msg.cam.camParameters.basicContainer;
    StationType_t station_type = basic.stationType;
    
    // Extract position (1/10 microdegrees to degrees)
    double latitude = basic.referencePosition.latitude / 10000000.0;
    double longitude = basic.referencePosition.longitude / 10000000.0;
    
    // Check if this is an RSU
    bool is_rsu = (station_type == StationType_roadSideUnit);
    
    std::cout << "\n[LOCALIZER] Received CAM:" << std::endl;
    std::cout << "  Station ID: " << station_id << std::endl;
    std::cout << "  Type: " << (is_rsu ? "RSU" : "Vehicle") << std::endl;
    std::cout << "  Position: " << std::fixed << std::setprecision(7)
              << "Lat=" << latitude << ", Lon=" << longitude << std::endl;
    
    // *** EXTRACT RSSI FROM CACHE ***
    int16_t rssi_dbm = 0;
    uint32_t src_l2id = 0;
    
    // Get the most recent RSSI measurement
    auto rssi_entry = RSSICache::instance().get_last();
    
    if (rssi_entry) {
        rssi_dbm = rssi_entry->rssi_dbm;
        src_l2id = rssi_entry->src_l2id;
        
        std::cout << "  Source MAC: " << rssi_entry->mac << std::endl;
    }
    
    std::cout << "  L2 ID: 0x" << std::hex << src_l2id << std::dec << std::endl;
    std::cout << "  RSSI: " << rssi_dbm << " dBm" << std::endl;
    
    if (is_rsu && rssi_dbm != 0) {
        std::cout << "  >>> RSU BEACON WITH RSSI DETECTED <<<" << std::endl;
        
        // Store RSU measurement
        RSUMeasurement measurement;
        measurement.rsu_id = station_id;
        measurement.latitude = latitude;
        measurement.longitude = longitude;
        measurement.rssi_dbm = rssi_dbm;
        measurement.timestamp = std::chrono::system_clock::now();
        
        // Add to measurements map
        rsu_measurements_[station_id] = measurement;
        
        // Clean old measurements (>2 seconds old)
        auto now = std::chrono::system_clock::now();
        for (auto it = rsu_measurements_.begin(); it != rsu_measurements_.end(); ) {
            auto age = std::chrono::duration_cast<std::chrono::seconds>(
                now - it->second.timestamp
            ).count();
            if (age > 2) {
                it = rsu_measurements_.erase(it);
            } else {
                ++it;
            }
        }

        std::cout << "  Total RSU measurements: " << rsu_measurements_.size() << std::endl;
        
        // Try trilateration if we have 3+ RSUs
        if (rsu_measurements_.size() >= 3) {
            std::cout << "\n  >>> PERFORMING TRILATERATION <<<" << std::endl;
            perform_trilateration();
        } else {
            std::cout << "  Need " << (3 - rsu_measurements_.size())
                      << " more RSU(s) for trilateration" << std::endl;
        }
    } else if (is_rsu) {
        std::cout << "  >>> RSU BEACON DETECTED (but no RSSI data) <<<" << std::endl;
    }
}

void VehicleLocalizerApplication::schedule_timer()
{
    timer_.expires_after(interval_);
    timer_.async_wait(std::bind(&VehicleLocalizerApplication::on_timer,
                                 this, std::placeholders::_1));
}

void VehicleLocalizerApplication::on_timer(const boost::system::error_code& ec)
{
    if (ec == boost::asio::error::operation_aborted) {
        return;
    }
    
    std::cout << "[LOCALIZER] Monitoring for RSU beacons with RSSI..." << std::endl;
    
    // Show current measurement status
    if (!rsu_measurements_.empty()) {
        std::cout << "[LOCALIZER] Active RSU measurements: " << rsu_measurements_.size() << std::endl;
        for (auto it = rsu_measurements_.begin(); it != rsu_measurements_.end(); ++it) {
            uint32_t rsu_id = it->first;
            const RSUMeasurement& meas = it->second;
            
            auto age = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::system_clock::now() - meas.timestamp
            ).count();
            std::cout << "  RSU " << rsu_id << ": RSSI=" << meas.rssi_dbm
                      << " dBm (age: " << age << " ms)" << std::endl;
        }
    }
    
    schedule_timer();
}

double VehicleLocalizerApplication::rssi_to_distance(int16_t rssi_dbm)
{
    // Path loss model: RSSI(d) = A - 10*n*log10(d)
    // Solving for d: d = 10^((A - RSSI) / (10*n))
    return std::pow(10.0, (path_loss_A_ - rssi_dbm) / (10.0 * path_loss_n_));
}

void VehicleLocalizerApplication::perform_trilateration()
{
    if (rsu_measurements_.size() < 3) return;
    
    std::cout << "\n  === TRILATERATION CALCULATION ===" << std::endl;
    
    // Convert RSSI to distances and display
    std::vector<std::tuple<double, double, double>> measurements; // lat, lon, distance
    
    for (auto it = rsu_measurements_.begin(); it != rsu_measurements_.end(); ++it) {
        uint32_t rsu_id = it->first;
        const RSUMeasurement& meas = it->second;
        
        double distance = rssi_to_distance(meas.rssi_dbm);
        measurements.push_back(std::make_tuple(meas.latitude, meas.longitude, distance));
        
        std::cout << "  RSU " << rsu_id << ": "
                  << "Pos(" << std::fixed << std::setprecision(7)
                  << meas.latitude << ", " << meas.longitude << "), "
                  << "RSSI=" << meas.rssi_dbm << " dBm, "
                  << "Dist=" << std::fixed << std::setprecision(1)
                  << distance << "m" << std::endl;
    }
    
    std::cout << "  Ready for position calculation with "
              << measurements.size() << " RSU measurements!" << std::endl;
    std::cout << "  ==================================\n" << std::endl;
    
    // TODO: Implement actual trilateration algorithm here
    // This would calculate vehicle position from the 3+ distance measurements
    // For now, we're just showing we have the data
}
void VehicleLocalizerApplication::log_measurements()
{
    if (rsu_measurements_.empty()) return;
    
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S");
    std::string timestamp = ss.str();
    
    // Log each RSU measurement
    for (auto it = rsu_measurements_.begin(); it != rsu_measurements_.end(); ++it) {
        uint32_t rsu_id = it->first;
        const RSUMeasurement& meas = it->second;
        
        double distance = rssi_to_distance(meas.rssi_dbm);
        
        rssi_log_file_ << timestamp << ","
                       << rsu_id << ","
                       << std::fixed << std::setprecision(7) << meas.latitude << ","
                       << std::fixed << std::setprecision(7) << meas.longitude << ","
                       << meas.rssi_dbm << ","
                       << std::fixed << std::setprecision(2) << distance
                       << std::endl;
    }
    
    rssi_log_file_.flush();
    
    // Log vehicle GPS position (if available)
    if (rsu_measurements_.size() >= 3) {
        auto gps_pos = positioning_.position_fix();
        
        position_log_file_ << timestamp << ","
                          << rsu_measurements_.size() << ","
                          << std::fixed << std::setprecision(7) << gps_pos.latitude.value() << ","
                          << std::fixed << std::setprecision(7) << gps_pos.longitude.value() << ","
                          << "0.0,0.0"  // Estimated position (placeholder for now)
                          << std::endl;
        
        position_log_file_.flush();
    }
}
