#include <future>
#include <string>
#include <chrono>
#include <thread>

#include "core/core.grpc.pb.h"
#include "mavsdk.h"

using std::chrono::seconds;
using std::this_thread::sleep_for;

namespace mavsdk {
namespace mavsdk_server {

template<typename Mavsdk = Mavsdk>
class CoreServiceImpl final : public mavsdk::rpc::core::CoreService::Service {
public:
    CoreServiceImpl(Mavsdk& mavsdk) :
        _mavsdk(mavsdk),
        _stop_promise(std::promise<void>()),
        _stop_future(_stop_promise.get_future())
    {}

    grpc::Status SubscribeConnectionState(
        grpc::ServerContext* /* context */,
        const rpc::core::SubscribeConnectionStateRequest* /* request */,
        grpc::ServerWriter<rpc::core::ConnectionStateResponse>* writer) override
    {
        _mavsdk.subscribe_on_new_system(
            [this, writer]() { publish_system_state(writer, _connection_state_mutex); });

        // Publish the current state on subscribe
        publish_system_state(writer, _connection_state_mutex);

        _stop_future.wait();
        return grpc::Status::OK;
    }

    grpc::Status SetMavlinkTimeout(
        grpc::ServerContext* /* context */,
        const rpc::core::SetMavlinkTimeoutRequest* request,
        rpc::core::SetMavlinkTimeoutResponse* /* response */) override
    {
        if (request != nullptr) {
            _mavsdk.set_timeout_s(request->timeout_s());
        }

        return grpc::Status::OK;
    }

    void stop() { _stop_promise.set_value(); }

    grpc::Status AddNewConnection(
        grpc::ServerContext* /* context */,
        const rpc::core::AddNewConnectionRequest* request,
        rpc::core::AddNewConnectionResponse* response) override
    {
        bool succeed = false;

        if (request != nullptr) {
            ConnectionResult connection_result = _mavsdk.add_any_connection(request->connection_url());
            if (connection_result != ConnectionResult::Success) {
                std::cerr << "Connection failed: " << connection_result << '\n';
            }
        }

        succeed = discover_new_system(_mavsdk);

        if (response != nullptr) {
            response->set_succeed(succeed);
        }

        return grpc::Status::OK;
    }

    bool discover_new_system(Mavsdk& mavsdk)
    {
        size_t original_num_systems = mavsdk.systems().size();
        bool new_system_discoverd = false;

        std::cout << "Waiting to discover system...\n";
        mavsdk.subscribe_on_new_system([&mavsdk, original_num_systems, &new_system_discoverd]() {
            const auto systems = mavsdk.systems();

            if (systems.size() > original_num_systems) {
                std::cout << "Discovered system\n";
                std::cout << "Original number of systems: " << original_num_systems << "\n";
                std::cout << "Number of systems after discover: " << int(systems.size()) << "\n";
                new_system_discoverd = true;
            }
        });

        // We usually receive heartbeats at 1Hz, therefore we should find a system after around 2
        // seconds.
        sleep_for(std::chrono::seconds(2));

        if (!new_system_discoverd) {
            std::cerr << "Not all systems found, exiting.\n";
            return false;
        } else {
            return true;
        }
    }

private:
    Mavsdk& _mavsdk;
    std::promise<void> _stop_promise;
    std::future<void> _stop_future;
    std::mutex _connection_state_mutex{};

    static mavsdk::rpc::core::ConnectionStateResponse
    createRpcConnectionStateResponse(const bool is_connected)
    {
        mavsdk::rpc::core::ConnectionStateResponse rpc_connection_state_response;

        auto* rpc_connection_state = rpc_connection_state_response.mutable_connection_state();
        rpc_connection_state->set_is_connected(is_connected);

        return rpc_connection_state_response;
    }

    void publish_system_state(
        grpc::ServerWriter<rpc::core::ConnectionStateResponse>* writer,
        std::mutex& connection_state_mutex)
    {
        auto systems = _mavsdk.systems();

        for (auto system : systems) {
            const auto rpc_connection_state_response =
                createRpcConnectionStateResponse(system->is_connected());

            std::lock_guard<std::mutex> lock(connection_state_mutex);
            writer->Write(rpc_connection_state_response);
        }
    }
};

} // namespace mavsdk_server
} // namespace mavsdk
