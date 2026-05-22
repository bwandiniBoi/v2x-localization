#include <vanetza/access/data_request.hpp>
#include <vanetza/access/pppp.hpp>
#include <vanetza/net/packet_variant.hpp>
#include <vanetza/rpc/link_layer_client.hpp>
#include <vanetza/rpc/logger.hpp>

#include <capnp/rpc-twoparty.h>
#include <capnp/rpc.h>
#include <kj/async.h>
#include <kj/time.h>
#include "vanetza.capnp.h"

#include <array>

namespace vanetza
{
namespace rpc
{

namespace
{

LinkLayerClient::ErrorCode map_error_code(vanetza::rpc::LinkLayer::ErrorCode in)
{
    switch (in)
    {
        case LinkLayer::ErrorCode::OK:
            return LinkLayerClient::ErrorCode::Ok;
        case LinkLayer::ErrorCode::INVALID_ARGUMENT:
            return LinkLayerClient::ErrorCode::InvalidArgument;
        case LinkLayer::ErrorCode::UNSUPPORTED:
            return LinkLayerClient::ErrorCode::Unsupported;
        case LinkLayer::ErrorCode::INTERNAL_ERROR:
        default:
            return LinkLayerClient::ErrorCode::InternalError;
    };
}

class DataListener : public vanetza::rpc::LinkLayer::DataListener::Server
{
public:
    DataListener(std::function<void(LinkLayerClient::Indication)> callback) :
        callback_(callback)
    {
    }

    kj::Promise<void> onDataIndication(OnDataIndicationContext context) override
    {
        auto frame = context.getParams().getFrame();
        vanetza::ByteBuffer payload { frame.getPayload().begin(), frame.getPayload().end() };
        LinkLayerClient::Indication indication { std::move(payload) };
        assign(indication.source, frame.getSourceAddress());
        assign(indication.destination, frame.getDestinationAddress());
        
        // Extract technology and RSSI from RX parameters
        if (context.getParams().hasRxParams()) {
            auto rxParams = context.getParams().getRxParams();
            
            if (rxParams.isWlan()) {
                indication.technology = LinkLayerClient::Technology::ITS_G5;
            } else if (rxParams.isCv2x()) {
                indication.technology = LinkLayerClient::Technology::LTE_V2X;
                
                // Extract RSSI from CV2X parameters
                auto cv2xParams = rxParams.getCv2x();
                indication.rssi_dbm8 = cv2xParams.getRssi();
            }
        }
        
        callback_(std::move(indication));
        return kj::READY_NOW;
    }

    void assign(MacAddress& into, const capnp::Data::Reader& from)
    {
        if (from.size() == MacAddress::length_bytes)
        {
            std::copy(from.begin(), from.end(), into.octets.begin());
        }
        else if (from.size() < MacAddress::length_bytes)
        {
            auto it = std::next(into.octets.begin(), MacAddress::length_bytes - from.size());
            std::fill(into.octets.begin(), it, 0);
            std::copy(from.begin(), from.end(), it);
        }
        else
        {
            auto it = std::next(from.begin(), from.size() - MacAddress::length_bytes);
            std::copy(it, from.end(), into.octets.begin());
        }
    }

private:
    std::function<void(LinkLayerClient::Indication)> callback_;
};

class CbrListener : public vanetza::rpc::LinkLayer::CbrListener::Server
{
public:
    CbrListener(std::function<void(dcc::ChannelLoad)> callback) :
        callback_(callback)
    {
    }

    kj::Promise<void> onCbrReport(OnCbrReportContext context) override
    {
        auto cbr = context.getParams().getCbr();
        dcc::ChannelLoad channel_load;
        if (cbr.getSamples() > 0 && cbr.getBusy() > 0) {
            if (cbr.getSamples() >= cbr.getBusy()) {
                channel_load = dcc::ChannelLoad(cbr.getBusy(), cbr.getSamples());
            } else {
                channel_load = dcc::ChannelLoad(cbr.getSamples(), cbr.getSamples());
            }
        };
        callback_(channel_load);
        return kj::READY_NOW;
    }

private:
    std::function<void(dcc::ChannelLoad)> callback_;
};

} // namespace

LinkLayerClient::Indication::Indication(vanetza::ByteBuffer buffer) :
    packet(std::move(buffer), OsiLayer::Network)
{
}

class LinkLayerClient::Context : public kj::TaskSet::ErrorHandler
{
public:
    Context(kj::Timer& timer, kj::AsyncIoStream& connection, Logger* logger) :
        logger_(logger),
        timer_(timer),
        task_set_(*this),
        client_(connection),
        link_layer_(client_.bootstrap().castAs<vanetza::rpc::LinkLayer>())
    {
    }

    void taskFailed(kj::Exception&& exception) override
    {
        VANETZA_RPC_LOG_ERROR(logger_, "LinkLayerClient/task", exception.getDescription().cStr());
    }

    void addTask(kj::Promise<void>&& promise, kj::Duration timeout)
    {
        task_set_.add(timer_.timeoutAfter(timeout, kj::mv(promise)));
    }

    Logger* logger_ = nullptr;
    kj::Timer& timer_;
    kj::TaskSet task_set_;
    capnp::TwoPartyClient client_;
    vanetza::rpc::LinkLayer::Client link_layer_;
};

LinkLayerClient::LinkLayerClient(kj::Timer& timer, kj::AsyncIoStream& connection, Logger* logger) :
    context_(std::make_unique<Context>(timer, connection, logger))
{
    auto rx_data = context_->link_layer_.subscribeDataRequest();
    rx_data.setListener(kj::heap<DataListener>(std::bind(&LinkLayerClient::do_indicate, this, std::placeholders::_1)));
    context_->addTask(rx_data.send().ignoreResult(), 1 * kj::SECONDS);

    auto cbr = context_->link_layer_.subscribeCbrRequest();
    cbr.setListener(kj::heap<CbrListener>(std::bind(&LinkLayerClient::do_report, this, std::placeholders::_1)));
    context_->addTask(cbr.send().ignoreResult(), 1 * kj::SECONDS);
}

LinkLayerClient::~LinkLayerClient()
{
}

void LinkLayerClient::configure(Technology technology)
{
    VANETZA_RPC_LOG_DEBUG(context_->logger_, "LinkLayerClient/configure", stringify(technology));
    technology_ = technology;
}

void LinkLayerClient::add_task(kj::Promise<void>&& promise)
{
    VANETZA_RPC_LOG_DEBUG(context_->logger_, "LinkLayerClient/task", "add");
    context_->task_set_.add(kj::mv(promise));
}

kj::Promise<LinkLayerClient::Identity> LinkLayerClient::identify()
{
    auto ident_request = context_->link_layer_.identifyRequest();
    auto promise = ident_request.send().then(
    [](capnp::Response<rpc::LinkLayer::IdentifyResults>&& results) mutable -> kj::Promise<Identity> {
              Identity identity;
              identity.id = results.getId();
              identity.version = results.getVersion();
              if (results.hasInfo()) {
                identity.info = results.getInfo().cStr();
              }
              return identity;
          });
    return promise;
}

void LinkLayerClient::request(const access::DataRequest& request, std::unique_ptr<ChunkPacket> packet)
{
    auto tx_data = context_->link_layer_.transmitDataRequest();

    auto frame = tx_data.initFrame();
    frame.setSourceAddress(kj::ArrayPtr<const kj::byte> { request.source_addr.octets.data(), request.source_addr.octets.size() });
    frame.setDestinationAddress(kj::ArrayPtr<const kj::byte> { request.destination_addr.octets.data(), request.destination_addr.octets.size() });

    auto tx_params = tx_data.initTxParams();
    if (technology_ == Technology::ITS_G5)
    {
        auto wlan = tx_params.initWlan();
        wlan.setPriority(static_cast<uint8_t>(request.access_category));
    }
    else if (technology_ == Technology::LTE_V2X)
    {
        auto cv2x = tx_params.initCv2x();
        cv2x.setPriority(static_cast<uint8_t>(request.access_category));
    }

    std::vector<vanetza::ByteBuffer> buffers;
    for (auto layer : osi_layer_range<OsiLayer::Network, OsiLayer::Application>())
    {
        ByteBuffer buffer;
        packet->layer(layer).convert(buffer);
        buffers.push_back(std::move(buffer));
    }

    std::size_t total_length = 0;
    for (auto& buffer : buffers)
    {
        total_length += buffer.size();
    }
    vanetza::ByteBuffer payload;
    payload.reserve(total_length);
    for (auto& buffer : buffers)
    {
        payload.insert(payload.end(), buffer.begin(), buffer.end());
    }
    frame.setPayload(kj::ArrayPtr<const kj::byte> { payload.data(), payload.size() });

    auto promise = tx_data.send().then([](capnp::Response<rpc::LinkLayer::TransmitDataResults>&& results) mutable {
        auto error = map_error_code(results.getError());
        if (error != ErrorCode::Ok)
        {
            // TODO: log error message
        }
    });
    context_->task_set_.add(kj::mv(promise));
}

void LinkLayerClient::do_indicate(Indication indication)
{
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (indication_callback_)
    {
        indication_callback_(std::move(indication));
    }
}

void LinkLayerClient::indicate(IndicationCallback callback)
{
    std::lock_guard<std::mutex> lock(callback_mutex_);
    indication_callback_ = callback;
}

void LinkLayerClient::do_report(dcc::ChannelLoad load)
{
    std::lock_guard<std::mutex> lock(callback_mutex_);
    if (cbr_callback_)
    {
        cbr_callback_(load);
    }
}

void LinkLayerClient::report_channel_load(ChannelLoadReportCallback callback)
{
    std::lock_guard<std::mutex> lock(callback_mutex_);
    cbr_callback_ = callback;
}

kj::Promise<LinkLayerClient::ErrorCode> LinkLayerClient::set_source_address(const MacAddress& address)
{
    auto request = context_->link_layer_.setSourceAddressRequest();
    request.setAddress(kj::ArrayPtr<const kj::byte> { address.octets.data(), address.octets.size() });
    return request.send().then([](capnp::Response<rpc::LinkLayer::SetSourceAddressResults>&& results) {
        return map_error_code(results.getError());
    });
}

const char* stringify(LinkLayerClient::ErrorCode code)
{
    switch (code)
    {
        case LinkLayerClient::ErrorCode::Ok:
            return "Ok";
        case LinkLayerClient::ErrorCode::InvalidArgument:
            return "InvalidArgument";
        case LinkLayerClient::ErrorCode::Unsupported:
            return "Unsupported";
        case LinkLayerClient::ErrorCode::InternalError:
            return "InternalError";
        default:
            return "Unknown";
    }
}

const char* stringify(LinkLayerClient::Technology technology)
{
    switch (technology)
    {
        case LinkLayerClient::Technology::Unspecified:
            return "Unspecified";
        case LinkLayerClient::Technology::ITS_G5:
            return "ITS-G5";
        case LinkLayerClient::Technology::LTE_V2X:
            return "LTE-V2X";
        default:
            return "Unknown";
    }
}

} // namespace rpc
} // namespace vanetza
