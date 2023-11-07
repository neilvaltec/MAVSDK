#pragma once

#include <memory>
#include <string>

// This is a struct because it is also exported to the C interface.
struct MavsdkServer {
public:
    MavsdkServer();
    ~MavsdkServer();
    MavsdkServer(MavsdkServer&&) = delete;
    MavsdkServer& operator=(MavsdkServer&&) = delete;

    int startGrpcServer(int port);
    bool connect(const std::string& connection_url = "udp://:14540", bool start_without_init_connection = false);
    void wait();
    void stop();
    int getPort();
    void setMavlinkIds(uint8_t system_id, uint8_t component_id);

private:
    class Impl;
    std::unique_ptr<Impl> _impl;
};
