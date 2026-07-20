#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#if defined(_WIN32) || defined(WIN32)
#include <windows.h>
#else
#include <csignal>
#endif

#include "cuarm_udp.h"

namespace {

using namespace robot::platform;

struct UdpTestMessage {
    char message[128];
};

bool pack_message(const UdpTestMessage& message, std::uint8_t* send_buffer, std::size_t buffer_size, std::size_t& written_size)
{
    if (buffer_size == 0) {
        return false;
    }
    written_size = static_cast<std::size_t>(
        std::snprintf(reinterpret_cast<char*>(send_buffer), buffer_size, "%s", message.message));
    return written_size > 0 && written_size < buffer_size;
}

bool unpack_message(const std::uint8_t* receive_buffer, std::size_t size, UdpTestMessage& message)
{
    const std::size_t copy_size = size < 127 ? size : 127;
    std::memcpy(message.message, receive_buffer, copy_size);
    message.message[copy_size] = '\0';
    return true;
}

std::atomic<bool> stop_requested(false);

void handle_sig(int)
{
    stop_requested = true;
}

}  // namespace

int main()
{
#if !defined(_WIN32) && !defined(WIN32)
    std::signal(SIGINT, handle_sig);
#endif

    const std::string kLocalIP = "127.0.0.1";
    const std::string kRemoteIP = "127.0.0.1";
    constexpr int kLocalPort = 30003;
    constexpr int kRemotePort = 30002;
    constexpr int kReceiveTimeoutUsec = 200000;
    constexpr auto kSendInterval = std::chrono::seconds(1);

    std::cout << "Connecting to UDP server at " << kRemoteIP << ":" << kRemotePort
              << " (local " << kLocalIP << ":" << kLocalPort << ")..." << std::endl;
    std::cout << "Sending and receiving continuously. Press Ctrl+C to stop." << std::endl;

    try {
        CuarmUdp<UdpTestMessage, UdpTestMessage> client(
            kLocalIP,
            kLocalPort,
            kRemoteIP,
            kRemotePort,
            unpack_message,
            pack_message);

        int send_count = 0;
        auto last_send_time = std::chrono::steady_clock::now();

        while (!stop_requested) {
            UdpTestMessage incoming{};
            if (client.receive(incoming, kReceiveTimeoutUsec)) {
                std::cout << "Received: " << incoming.message << std::endl;

                if (std::strncmp(incoming.message, "SERVER_HEARTBEAT", 16) == 0) {
                    UdpTestMessage reply{};
                    std::snprintf(reply.message, sizeof(reply.message), "CLIENT_ACK: %s", incoming.message);
                    client.send(reply);
                    std::cout << "Sent: " << reply.message << std::endl;
                }
            }

            const auto now = std::chrono::steady_clock::now();
            if (now - last_send_time >= kSendInterval) {
                UdpTestMessage outgoing{};
                std::snprintf(outgoing.message, sizeof(outgoing.message), "CLIENT_COMMAND #%d", ++send_count);
                client.send(outgoing);
                std::cout << "Sent: " << outgoing.message << std::endl;
                last_send_time = now;
            }
        }

        client.close();
        std::cout << "Connection closed." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "UDP client error: " << e.what() << std::endl;
        return 1;
    }

// Bypassing global exit teardown to prevent runtime library mismatch with Conda dependencies.
#if defined(_DEBUG) || !defined(NDEBUG)
    std::_Exit(0);
#else
    return 0;
#endif
}
