#include "rsu_application.hpp"
#include <vanetza/btp/ports.hpp>
#include <vanetza/asn1/cam.hpp>
#include <vanetza/asn1/packet_visitor.hpp>
#include <vanetza/facilities/cam_functions.hpp>
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <chrono>
#include <exception>
#include <functional>
#include <iostream>

using namespace vanetza;
using namespace vanetza::facilities;
using namespace std::chrono;

RsuApplication::RsuApplication(PositionProvider& positioning, Runtime& rt) :
    positioning_(positioning), runtime_(rt), beacon_interval_(seconds(1))
{
    schedule_timer();
}

void RsuApplication::set_interval(Clock::duration interval)
{
    beacon_interval_ = interval;
    runtime_.cancel(this);
    schedule_timer();
}

void RsuApplication::print_generated_message(bool flag)
{
    print_tx_msg_ = flag;
}

RsuApplication::PortType RsuApplication::port()
{
    return btp::ports::CAM;
}

void RsuApplication::indicate(const DataIndication& indication, UpPacketPtr packet)
{
    // RSU only transmits, does not process received packets
}

void RsuApplication::schedule_timer()
{
    runtime_.schedule(beacon_interval_, std::bind(&RsuApplication::on_timer, this, std::placeholders::_1), this);
}

void RsuApplication::on_timer(Clock::time_point)
{
    schedule_timer();
    vanetza::asn1::Cam message;

    ItsPduHeader_t& header = message->header;
    header.protocolVersion = 2;
    header.messageID = ItsPduHeader__messageID_cam;
    header.stationID = 100; // RSU station ID (different from vehicles)

    const auto time_now = duration_cast<milliseconds>(runtime_.now().time_since_epoch());
    uint16_t gen_delta_time = time_now.count();

    CoopAwareness_t& cam = message->cam;
    cam.generationDeltaTime = gen_delta_time * GenerationDeltaTime_oneMilliSec;

    auto position = positioning_.position_fix();

    if (!std::isfinite(position.latitude.value()) || !std::isfinite(position.longitude.value())) {
        std::cerr << "RSU: Skipping beacon - no valid position available" << std::endl;
        return;
    }

    BasicContainer_t& basic = cam.camParameters.basicContainer;
    basic.stationType = StationType_roadSideUnit; // Mark as RSU
    copy(position, basic.referencePosition);

    // Minimal high frequency container for RSU
    cam.camParameters.highFrequencyContainer.present = HighFrequencyContainer_PR_basicVehicleContainerHighFrequency;

    BasicVehicleContainerHighFrequency& bvc = cam.camParameters.highFrequencyContainer.choice.basicVehicleContainerHighFrequency;
    bvc.heading.headingValue = 0;
    bvc.heading.headingConfidence = HeadingConfidence_unavailable;

    bvc.speed.speedValue = 0; // RSU is stationary
    bvc.speed.speedConfidence = SpeedConfidence_equalOrWithinOneCentimeterPerSec;

    bvc.driveDirection = DriveDirection_unavailable;
    bvc.longitudinalAcceleration.longitudinalAccelerationValue = LongitudinalAccelerationValue_unavailable;

    bvc.vehicleLength.vehicleLengthValue = VehicleLengthValue_unavailable;
    bvc.vehicleLength.vehicleLengthConfidenceIndication = VehicleLengthConfidenceIndication_unavailable;
    bvc.vehicleWidth = VehicleWidth_unavailable;

    bvc.curvature.curvatureValue = 0;
    bvc.curvature.curvatureConfidence = CurvatureConfidence_unavailable;
    bvc.curvatureCalculationMode = CurvatureCalculationMode_unavailable;

    bvc.yawRate.yawRateValue = YawRateValue_unavailable;

    std::string error;
    if (!message.validate(error)) {
        throw std::runtime_error("Invalid RSU beacon: " + error);
    }

    if (print_tx_msg_) {
        std::cout << "RSU Beacon transmitted (Lat: " << position.latitude.value() 
                  << ", Lon: " << position.longitude.value() << ")" << std::endl;
    }

    DownPacketPtr packet { new DownPacket() };
    packet->layer(OsiLayer::Application) = std::move(message);

    DataRequest request;
    request.its_aid = aid::CA;
    request.transport_type = geonet::TransportType::SHB;
    request.communication_profile = geonet::CommunicationProfile::ITS_G5;

    auto confirm = Application::request(request, std::move(packet));
    if (!confirm.accepted()) {
        throw std::runtime_error("RSU beacon transmission failed");
    }
}
