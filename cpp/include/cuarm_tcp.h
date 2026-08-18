#ifndef CUARM_TCP_H
#define CUARM_TCP_H

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>

extern "C" {
#include "curi_tcp/c/src/curi_tcp.h"
#include "curi_tcp/c/src/curi_tcp_framed.h"
}

namespace robot::platform {
template <typename UnpackT, typename PackT>
class CuarmTcp {
public:
    CuarmTcp(
        const std::string& server_ip,
        int server_port,
        bool is_server,
        bool (*unpack_func)(const std::uint8_t*, std::size_t, UnpackT&) = nullptr,
        bool (*pack_func)(const PackT&, std::uint8_t*, std::size_t, std::size_t&) = nullptr,
        int buffer_size = 4096);
    ~CuarmTcp();

    void send(const PackT& data);
    /** @return 0 on success, 1 on timeout, -1 on disconnect (server clears client). */
    int receive(UnpackT& data, int timeout_usec);
    bool server_has_client();
    bool server_wait_client(int timeout_usec);
    void close();

private:
    bool (*pack_function_)(const PackT&, std::uint8_t*, std::size_t, std::size_t&) = nullptr;
    bool (*unpack_function_)(const std::uint8_t*, std::size_t, UnpackT&) = nullptr;
    tcp_node* tcp_node_ptr_ = nullptr;
    int buffer_size_ = 0;
    bool is_server_ = false;
    std::string server_ip_;
    bool is_closed_ = false;
};

template <typename UnpackT, typename PackT>
CuarmTcp<UnpackT, PackT>::CuarmTcp(
    const std::string& server_ip,
    int server_port,
    bool is_server,
    bool (*unpack_func)(const std::uint8_t*, std::size_t, UnpackT&),
    bool (*pack_func)(const PackT&, std::uint8_t*, std::size_t, std::size_t&),
    int buffer_size)
{
    if (buffer_size <= 0 || buffer_size > static_cast<int>(CURI_TCP_FRAME_MAX_PAYLOAD)) {
        throw std::invalid_argument("CuarmTcp: buffer_size must be in (0, CURI_TCP_FRAME_MAX_PAYLOAD]");
    }

    buffer_size_ = buffer_size;
    pack_function_ = pack_func;
    unpack_function_ = unpack_func;
    is_server_ = is_server;
    server_ip_ = server_ip;

    tcp_node_ptr_ = static_cast<tcp_node*>(std::malloc(sizeof(tcp_node)));
    if (!tcp_node_ptr_) {
        throw std::runtime_error("Failed to allocate memory for tcp_node struct.");
    }
    std::memset(tcp_node_ptr_, 0, sizeof(tcp_node));

    const int ret = tcp_init(tcp_node_ptr_, server_ip_.c_str(), server_port, buffer_size_, is_server);
    if (ret != 0) {
        std::free(tcp_node_ptr_);
        tcp_node_ptr_ = nullptr;
        if (is_server) {
            throw std::runtime_error(
                "Failed to initialize TCP server, return code: " + std::to_string(ret));
        }
        throw std::runtime_error("TCP Connection Failed, return code: " + std::to_string(ret));
    }
}

template <typename UnpackT, typename PackT>
CuarmTcp<UnpackT, PackT>::~CuarmTcp() {
    close();
}

template <typename UnpackT, typename PackT>
void CuarmTcp<UnpackT, PackT>::send(const PackT& data) {
    if (!pack_function_) {
        throw std::runtime_error("TCP Pack function is not defined.");
    }
    if (CURI_TCP_IS_INVALID_FD(tcp_node_ptr_->client_fd)) {
        throw std::runtime_error("CuarmTcp::send: no active TCP connection.");
    }

    std::size_t written_size = 0;
    if (!pack_function_(
            data,
            reinterpret_cast<std::uint8_t*>(tcp_node_ptr_->send_buffer),
            static_cast<std::size_t>(buffer_size_),
            written_size)) {
        throw std::runtime_error("CuarmTcp::send: failed to pack data.");
    }
    if (written_size == 0 || written_size > static_cast<std::size_t>(buffer_size_)) {
        throw std::runtime_error("CuarmTcp::send: invalid packed size.");
    }

    if (tcp_send_frame(
            tcp_node_ptr_,
            reinterpret_cast<const std::uint8_t*>(tcp_node_ptr_->send_buffer),
            static_cast<std::uint32_t>(written_size)) != 0) {
        throw std::runtime_error("CuarmTcp::send: tcp_send_frame failed.");
    }
}

template <typename UnpackT, typename PackT>
int CuarmTcp<UnpackT, PackT>::receive(UnpackT& data, int timeout_usec) {
    if (!unpack_function_) {
        throw std::runtime_error("TCP Unpack function is not defined.");
    }
    if (CURI_TCP_IS_INVALID_FD(tcp_node_ptr_->client_fd)) {
        return is_server_ ? -1 : -1;
    }

    const int frame_len = tcp_recv_frame(
        tcp_node_ptr_,
        reinterpret_cast<std::uint8_t*>(tcp_node_ptr_->receive_buffer),
        static_cast<std::uint32_t>(buffer_size_),
        timeout_usec);

    if (frame_len > 0) {
        if (!unpack_function_(
                reinterpret_cast<const std::uint8_t*>(tcp_node_ptr_->receive_buffer),
                static_cast<std::size_t>(frame_len),
                data)) {
            throw std::runtime_error("CuarmTcp::receive: failed to unpack frame.");
        }
        return 0;
    }

    if (frame_len == 0) {
        return 1;
    }

    if (is_server_) {
        tcp_server_clear_client(tcp_node_ptr_);
        return -1;
    }
    throw std::runtime_error("TCP Connection lost.");
}

template <typename UnpackT, typename PackT>
bool CuarmTcp<UnpackT, PackT>::server_has_client() {
    return tcp_server_has_client(tcp_node_ptr_) == 1;
}

template <typename UnpackT, typename PackT>
bool CuarmTcp<UnpackT, PackT>::server_wait_client(int timeout_usec) {
    const int ret = tcp_server_wait_client(tcp_node_ptr_, timeout_usec, buffer_size_);
    if (ret < 0) {
        throw std::runtime_error("TCP Error waiting for client: " + std::to_string(ret));
    }
    if (ret == 0) {
        return true;
    }
    return false;
}

template <typename UnpackT, typename PackT>
void CuarmTcp<UnpackT, PackT>::close() {
    if (is_closed_) {
        return;
    }

    if (tcp_node_ptr_) {
        tcp_close(tcp_node_ptr_);
        std::free(tcp_node_ptr_);
        tcp_node_ptr_ = nullptr;
    }
    is_closed_ = true;
}

}  // namespace robot::platform

#endif  // CUARM_TCP_H
