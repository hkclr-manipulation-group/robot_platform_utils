#include <algorithm>
#include <atomic>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>

#if defined(_WIN32) || defined(WIN32)
#include <windows.h>
#else
#include <csignal>
#endif

#include "cuarm_tcp.h"

namespace {

using namespace robot::platform;

struct TcpTestCommand {
    char message[128];
};

struct TcpTestResponse {
    char message[128];
};

bool pack_response(
    const TcpTestResponse& response,
    std::uint8_t* send_buffer,
    std::size_t buffer_size,
    std::size_t& written_size)
{
    const int n = std::snprintf(
        reinterpret_cast<char*>(send_buffer),
        buffer_size,
        "%s",
        response.message);
    if (n < 0 || static_cast<std::size_t>(n) >= buffer_size) {
        return false;
    }
    written_size = static_cast<std::size_t>(n);
    return true;
}

bool unpack_command(
    const std::uint8_t* receive_buffer,
    std::size_t receive_size,
    TcpTestCommand& command)
{
    const std::size_t copy_len = std::min(receive_size, sizeof(command.message) - 1);
    std::memcpy(command.message, receive_buffer, copy_len);
    command.message[copy_len] = '\0';
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

    const std::string kServerIP = "127.0.0.1";
    constexpr int kServerPort = 30001;
    constexpr int kBufferSize = 1024;
    constexpr int kSelectTimeoutUsec = 1000000;

    try {
        CuarmTcp<TcpTestCommand, TcpTestResponse> server(
            kServerIP,
            kServerPort,
            true,
            unpack_command,
            pack_response,
            kBufferSize);

        std::cout << "Initializing TCP Server on port " << kServerPort << "..." << std::endl;

        while (!stop_requested) {
            if (!server.server_has_client()) {
                std::cout << "Waiting for Client..." << std::endl;
                if (!server.server_wait_client(-1)) {
                    continue;
                }
                std::cout << "Client connected. Waiting for commands..." << std::endl;
            } else {
                TcpTestCommand command{};
                const int ret = server.receive(command, kSelectTimeoutUsec);
                if (ret == 0) {
                    std::cout << "Executing: " << command.message << std::endl;

                    TcpTestResponse response{};
                    std::strncpy(response.message, "COMMAND_DONE", 127);
                    server.send(response);
                } else if (ret == -1) {
                    std::cout << "Client disconnected." << std::endl;
                }
            }
        }

        server.close();
        std::cout << "TCP server closed." << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "TCP server error: " << e.what() << std::endl;
        return 1;
    }

#if defined(_DEBUG) || !defined(NDEBUG)
    std::_Exit(0);
#else
    return 0;
#endif
}
