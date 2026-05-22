#include "monitor_application.hpp"
#include <vanetza/btp/ports.hpp>
#include <vanetza/asn1/cam.hpp>
#include <vanetza/asn1/packet_visitor.hpp>
#include <iostream>

using namespace vanetza;

MonitorApplication::MonitorApplication(boost::asio::io_context& io) :
    display_timer_(io),
    display_interval_(std::chrono::seconds(5)),
    message_count_(0)
{
    schedule_display_timer();
}

MonitorApplication::PortType MonitorApplication::port()
{
    return btp::ports::CAM;
}

Application::PromiscuousHook* MonitorApplication::promiscuous_hook()
{
    return this;
}

void MonitorApplication::indicate(const DataIndication& indication, UpPacketPtr packet)
{
    // Processed via promiscuous hook
}

void MonitorApplication::tap_packet(const DataIndication& indication, const vanetza::UpPacket& packet)
{
    asn1::PacketVisitor<asn1::Cam> visitor;
    std::shared_ptr<const asn1::Cam> cam = boost::apply_visitor(visitor, packet);
    
    if (cam) {
        message_count_++;
    }
}

void MonitorApplication::schedule_display_timer()
{
    display_timer_.expires_after(display_interval_);
    display_timer_.async_wait(std::bind(&MonitorApplication::on_display_timer, 
                                        this, std::placeholders::_1));
}

void MonitorApplication::on_display_timer(const boost::system::error_code& ec)
{
    if (ec == boost::asio::error::operation_aborted) {
        return;
    }

    std::cout << "\n=== MONITOR: Received " << message_count_ << " CAMs ===" << std::endl;
    
    schedule_display_timer();
}
