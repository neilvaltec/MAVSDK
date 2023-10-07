#pragma once

#include <memory>
#include <mutex>

#include <mavsdk.h>

namespace mavsdk::mavsdk_server {

template<typename Plugin> class LazyPlugin {
public:
    explicit LazyPlugin(Mavsdk& mavsdk) : _mavsdk(mavsdk) {}

    // Plugin* maybe_plugin()
    // {
    //     std::lock_guard<std::mutex> lock(_mutex);
    //     if (_plugin == nullptr) {
    //         if (_mavsdk.systems().empty()) {
    //             return nullptr;
    //         }
    //         _plugin = std::make_unique<Plugin>(_mavsdk.systems()[0]);
    //     }
    //     return _plugin.get();
    // }

    Plugin* maybe_plugin(int32_t system_id) {
        std::lock_guard<std::mutex> lock(_mutex);
        auto it = _dict_plugin.find(system_id);
        if (it == _dict_plugin.end()) {
            auto system = _mavsdk.systems().at(system_id);
            _dict_plugin[system_id] = std::make_unique<Plugin>(system);
        }
        return _dict_plugin[system_id].get();
    }

private:
    Mavsdk& _mavsdk;
    // std::unique_ptr<Plugin> _plugin{};
    std::unordered_map<int, std::unique_ptr<Plugin>> _dict_plugin{};
    std::mutex _mutex{};
};

} // namespace mavsdk::mavsdk_server
