#pragma once

#include <future>
#include <mutex>
#include <string>

#include "connection_result.h"
#include "log.h"

namespace mavsdk {
namespace mavsdk_server {

std::vector<std::string> split(const std::string &s, char delimiter) {
    std::vector<std::string> tokens;
    std::string token;
    for (char c : s) {
        if (c == delimiter) {
            tokens.push_back(token);
            token.clear();
        } else {
            token += c;
        }
    }
    tokens.push_back(token); // For the last token
    return tokens;
};

template<typename Mavsdk> class ConnectionInitiator {
public:
    ConnectionInitiator() {}
    ~ConnectionInitiator() {}

    bool start(Mavsdk& mavsdk, const std::string& connection_url)
    {
        LogInfo() << "Waiting to discover system on " << connection_url << "...";
        _discovery_future = wrapped_subscribe_on_new_system(mavsdk);

        std::vector<std::string> all_connection_url = split(connection_url, ',');

        // the loop below adds the number of ports the sdk monitors.
        for (const auto& each_connection_url : all_connection_url) {
            if (!add_any_connection(mavsdk, each_connection_url)) {
                return false;
            }
        }
        return true;      
    }

    bool wait() { return _discovery_future.get(); }

    void cancel()
    {
        std::lock_guard<std::mutex> guard(_mutex);
        if (!_is_discovery_finished) {
            _is_discovery_finished = true;
            _discovery_promise->set_value(false);
        }
    }

private:
    bool add_any_connection(Mavsdk& mavsdk, const std::string& connection_url)
    {
        mavsdk::ConnectionResult connection_result = mavsdk.add_any_connection(connection_url);

        if (connection_result != ConnectionResult::Success) {
            LogErr() << "Connection failed: " << connection_result;
            return false;
        }

        return true;
    }

    std::future<bool> wrapped_subscribe_on_new_system(Mavsdk& mavsdk)
    {
        auto future = _discovery_promise->get_future();

        mavsdk.subscribe_on_new_system([this, &mavsdk]() {
            std::lock_guard<std::mutex> guard(_mutex);
            const auto system = mavsdk.systems().back();

            if (!_is_discovery_finished && system->is_connected()) {
                LogInfo() << "System discovered";

                _is_discovery_finished = true;
                _discovery_promise->set_value(true);
            }

            LogInfo() << "Total systems (drones): " << mavsdk.systems().size();
        });

        return future;
    }

    std::mutex _mutex;
    std::atomic<bool> _is_discovery_finished = false;
    std::shared_ptr<std::promise<bool>> _discovery_promise = std::make_shared<std::promise<bool>>();
    std::future<bool> _discovery_future{};
};

} // namespace mavsdk_server
} // namespace mavsdk
