#include <algorithm>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <string>

#include "cuarm_tcp.h"

namespace {

using namespace robot::platform;

struct TcpTestCommand {
    char message[128];
};

struct TcpTestResponse {
    char message[128];
};

bool pack_command(
    const TcpTestCommand& command,
    std::uint8_t* send_buffer,
    std::size_t buffer_size,
    std::size_t& written_size)
{
    const int n = std::snprintf(
        reinterpret_cast<char*>(send_buffer),
        buffer_size,
        "%s",
        command.message);
    if (n < 0 || static_cast<std::size_t>(n) >= buffer_size) {
        return false;
    }
    written_size = static_cast<std::size_t>(n);
    return true;
}

bool unpack_response(
    const std::uint8_t* receive_buffer,
    std::size_t receive_size,
    TcpTestResponse& response)
{
    const std::size_t copy_len = std::min(receive_size, sizeof(response.message) - 1);
    std::memcpy(response.message, receive_buffer, copy_len);
    response.message[copy_len] = '\0';
    return true;
}

}  // namespace

int main()
{
    const std::string kServerIP = "127.0.0.1";
    constexpr int kServerPort = 30001;
    constexpr int kBufferSize = 1024;
    constexpr int kResponseTimeoutUsec = 2000000;
    std::cout << "Connecting to TCP server at 127.0.0.1:" << kServerPort << "..." << std::endl;

    CuarmTcp<TcpTestResponse, TcpTestCommand> client(
        kServerIP,
        kServerPort,
        false,
        unpack_response,
        pack_command,
        kBufferSize);

    TcpTestCommand command{};
    std::strncpy(command.message, "MOVE_J 0,0,0", 127);
    std::cout << "Sending command: " << command.message << std::endl;
    client.send(command);

    TcpTestResponse response{};
    const int ret = client.receive(response, kResponseTimeoutUsec);
    if (ret == 0) {
        std::cout << "Robot response: " << response.message << std::endl;
    } else if (ret == 1) {
        std::cout << "No response or timeout." << std::endl;
        return 1;
    }

    client.close();
    std::cout << "Connection closed." << std::endl;

#if defined(_DEBUG) || !defined(NDEBUG)
    std::_Exit(0);
#else
    return 0;
#endif
}
