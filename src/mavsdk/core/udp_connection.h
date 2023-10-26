#pragma once

#include <string>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <vector>
#include <cstdint>
#include "connection.h"

namespace mavsdk {

class UdpConnection : public Connection {
public:
    explicit UdpConnection(
        Connection::receiver_callback_t receiver_callback,
        std::string local_ip,
        int local_port,
        ForwardingOption forwarding_option = ForwardingOption::ForwardingOff);
    ~UdpConnection() override;
    ConnectionResult start() override;
    ConnectionResult stop() override;

    bool send_message(const mavlink_message_t& message) override;

    bool valtec_send_message(const mavlink_message_t& message) override;

    void add_remote(const std::string& remote_ip, int remote_port);

    // Non-copyable
    UdpConnection(const UdpConnection&) = delete;
    const UdpConnection& operator=(const UdpConnection&) = delete;

private:
    ConnectionResult setup_port();
    void start_recv_thread();

    void receive();

    void add_remote_with_remote_sysid(
        const std::string& remote_ip, int remote_port, uint8_t remote_sysid);

    std::string _local_ip;
    int _local_port_number;

    std::mutex _remote_mutex{};
    struct Remote {
        std::string ip{};
        int port_number{0};

        bool operator==(const UdpConnection::Remote& other) const
        {
            return ip == other.ip && port_number == other.port_number;
        }
    };
    std::vector<Remote> _remotes{};

    int _socket_fd{-1};
    std::unique_ptr<std::thread> _recv_thread{};
    std::atomic_bool _should_exit{false};

    const std::string UDP_IP = "127.0.0.1";
    const int UDP_PORT = 8080;
    struct sockaddr_in target_addr;
    int sock;
};

} // namespace mavsdk
