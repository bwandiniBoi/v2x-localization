#ifndef RSU_APPLICATION_HPP_RSUCA9S
#define RSU_APPLICATION_HPP_RSUCA9S

#include "application.hpp"
#include <vanetza/common/clock.hpp>
#include <vanetza/common/position_provider.hpp>
#include <vanetza/common/runtime.hpp>

/**
 * RsuApplication - Roadside Unit Beacon Application
 * 
 * Broadcasts periodic CAM-like beacons containing:
 * - RSU position (fixed location)
 * - Station ID
 * - Timestamp
 * 
 * Used by mobile vehicles for RSSI-based trilateration
 */
class RsuApplication : public Application
{
public:
    RsuApplication(vanetza::PositionProvider& positioning, vanetza::Runtime& rt);
    PortType port() override;
    void indicate(const DataIndication&, UpPacketPtr) override;
    void set_interval(vanetza::Clock::duration);
    void print_generated_message(bool flag);

private:
    void schedule_timer();
    void on_timer(vanetza::Clock::time_point);

    vanetza::PositionProvider& positioning_;
    vanetza::Runtime& runtime_;
    vanetza::Clock::duration beacon_interval_;
    bool print_tx_msg_ = false;
};

#endif /* RSU_APPLICATION_HPP_RSUCA9S */
