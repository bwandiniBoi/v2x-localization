#ifndef MONITOR_APPLICATION_HPP_MONITOR
#define MONITOR_APPLICATION_HPP_MONITOR

#include "application.hpp"
#include <boost/asio/io_context.hpp>
#include <boost/asio/steady_timer.hpp>
#include <map>
#include <string>

class MonitorApplication : public Application, private Application::PromiscuousHook
{
public:
    MonitorApplication(boost::asio::io_context& io);
    PortType port() override;
    void indicate(const DataIndication&, UpPacketPtr) override;
    Application::PromiscuousHook* promiscuous_hook() override;

private:
    void schedule_display_timer();
    void on_display_timer(const boost::system::error_code& ec);
    void tap_packet(const DataIndication&, const vanetza::UpPacket&) override;
    
    boost::asio::steady_timer display_timer_;
    std::chrono::milliseconds display_interval_;
    unsigned int message_count_;
};

#endif
