#include "tcp_data_server.h"

#include <algorithm>
#include <cstring>

namespace robot::platform {
namespace {

bool readU32(const std::uint8_t* data, std::size_t size, std::size_t& offset, std::uint32_t& out)
{
    if (offset + sizeof(std::uint32_t) > size) {
        return false;
    }
    std::memcpy(&out, data + offset, sizeof(out));
    offset += sizeof(std::uint32_t);
    return true;
}

}  // namespace

TcpDataServer::TcpDataServer() = default;

TcpDataServer::~TcpDataServer() {
    close();
}

bool TcpDataServer::listen(const std::string& bind_ip, int port, int buffer_size) {
    close();

    buffer_size_ = buffer_size;
    tx_buffer_.assign(static_cast<std::size_t>(buffer_size_), 0);
    rx_buffer_.assign(static_cast<std::size_t>(buffer_size_), 0);

    node_ = static_cast<tcp_node*>(std::malloc(sizeof(tcp_node)));
    if (!node_) {
        return false;
    }
    std::memset(node_, 0, sizeof(tcp_node));

    if (tcp_init(node_, bind_ip.c_str(), port, buffer_size_, true) != 0) {
        close();
        return false;
    }

    listening_ = true;
    return true;
}

bool TcpDataServer::sendAck(std::uint32_t sequence, std::uint32_t status_code, int timeout_usec)
{
    (void)timeout_usec;
    tcp_data::MessageHeader header{};
    header.kind = static_cast<std::uint16_t>(tcp_data::MessageKind::kAck);
    header.sequence = sequence;
    header.payload_size = sizeof(status_code);

    std::memcpy(tx_buffer_.data(), &header, sizeof(header));
    std::memcpy(tx_buffer_.data() + sizeof(header), &status_code, sizeof(status_code));

    return tcp_send_frame(
               node_,
               tx_buffer_.data(),
               static_cast<std::uint32_t>(sizeof(header) + sizeof(status_code))) == 0;
}

bool TcpDataServer::sendError(std::uint32_t sequence, const std::string& message, int timeout_usec)
{
    (void)timeout_usec;
    const std::uint32_t payload_size = static_cast<std::uint32_t>(message.size());
    tcp_data::MessageHeader header{};
    header.kind = static_cast<std::uint16_t>(tcp_data::MessageKind::kError);
    header.sequence = sequence;
    header.payload_size = payload_size;

    const std::size_t frame_size = sizeof(header) + message.size();
    if (frame_size > tx_buffer_.size()) {
        return false;
    }

    std::memcpy(tx_buffer_.data(), &header, sizeof(header));
    if (!message.empty()) {
        std::memcpy(tx_buffer_.data() + sizeof(header), message.data(), message.size());
    }

    return tcp_send_frame(node_, tx_buffer_.data(), static_cast<std::uint32_t>(frame_size)) == 0;
}

bool TcpDataServer::handleClient(const BlobHandler& handler, int timeout_usec)
{
    BlobReceipt active{};
    std::uint32_t active_sequence = 0;
    bool transfer_open = false;

    while (tcp_server_has_client(node_)) {
        const int frame_len = tcp_recv_frame(
            node_,
            rx_buffer_.data(),
            static_cast<std::uint32_t>(rx_buffer_.size()),
            timeout_usec);

        if (frame_len <= 0) {
            if (frame_len < 0) {
                tcp_server_clear_client(node_);
                return false;
            }
            continue;
        }

        if (static_cast<std::size_t>(frame_len) < sizeof(tcp_data::MessageHeader)) {
            sendError(active_sequence, "truncated header", timeout_usec);
            return false;
        }

        const auto* header =
            reinterpret_cast<const tcp_data::MessageHeader*>(rx_buffer_.data());
        if (header->magic != tcp_data::kMagic || header->version != tcp_data::kVersion) {
            sendError(header->sequence, "invalid magic/version", timeout_usec);
            return false;
        }

        const auto kind = static_cast<tcp_data::MessageKind>(header->kind);
        const std::uint8_t* payload = rx_buffer_.data() + sizeof(tcp_data::MessageHeader);
        const std::size_t payload_size = header->payload_size;

        if (sizeof(tcp_data::MessageHeader) + payload_size > static_cast<std::size_t>(frame_len)) {
            sendError(header->sequence, "truncated payload", timeout_usec);
            return false;
        }

        switch (kind) {
            case tcp_data::MessageKind::kPing: {
                tcp_data::MessageHeader pong{};
                pong.kind = static_cast<std::uint16_t>(tcp_data::MessageKind::kPong);
                pong.sequence = header->sequence;
                pong.payload_size = 0;
                std::memcpy(tx_buffer_.data(), &pong, sizeof(pong));
                if (tcp_send_frame(node_, tx_buffer_.data(), sizeof(pong)) != 0) {
                    return false;
                }
                break;
            }
            case tcp_data::MessageKind::kBlobBegin: {
                active = BlobReceipt{};
                active_sequence = header->sequence;
                transfer_open = true;

                std::size_t offset = 0;
                std::uint32_t name_len = 0;
                std::uint32_t total_size = 0;
                if (!readU32(payload, payload_size, offset, name_len)) {
                    sendError(header->sequence, "invalid BlobBegin payload", timeout_usec);
                    return false;
                }
                if (offset + name_len + sizeof(total_size) > payload_size) {
                    sendError(header->sequence, "invalid BlobBegin payload", timeout_usec);
                    return false;
                }
                active.name.assign(
                    reinterpret_cast<const char*>(payload + offset),
                    name_len);
                offset += name_len;
                if (!readU32(payload, payload_size, offset, total_size)) {
                    sendError(header->sequence, "invalid BlobBegin payload", timeout_usec);
                    return false;
                }
                active.data.assign(total_size, 0);
                sendAck(header->sequence, 0, timeout_usec);
                break;
            }
            case tcp_data::MessageKind::kBlobChunk: {
                if (!transfer_open || header->sequence != active_sequence) {
                    sendError(header->sequence, "unexpected BlobChunk", timeout_usec);
                    return false;
                }
                std::size_t offset = 0;
                std::uint32_t chunk_offset = 0;
                if (!readU32(payload, payload_size, offset, chunk_offset) ||
                    offset > payload_size) {
                    sendError(header->sequence, "invalid BlobChunk header", timeout_usec);
                    return false;
                }
                const std::size_t chunk_len = payload_size - offset;
                if (chunk_offset + chunk_len > active.data.size()) {
                    sendError(header->sequence, "BlobChunk out of range", timeout_usec);
                    return false;
                }
                std::memcpy(active.data.data() + chunk_offset, payload + offset, chunk_len);
                break;
            }
            case tcp_data::MessageKind::kBlobEnd: {
                if (!transfer_open || header->sequence != active_sequence) {
                    sendError(header->sequence, "unexpected BlobEnd", timeout_usec);
                    return false;
                }
                std::string error;
                const bool ok = handler ? handler(active, error) : true;
                transfer_open = false;
                if (!ok) {
                    sendError(header->sequence, error.empty() ? "blob handler failed" : error, timeout_usec);
                    return false;
                }
                sendAck(header->sequence, 0, timeout_usec);
                break;
            }
            default:
                sendError(header->sequence, "unsupported message kind", timeout_usec);
                return false;
        }
    }
    return true;
}

void TcpDataServer::serveForever(
    const BlobHandler& handler,
    const std::function<bool()>& stop_requested) {
    constexpr int kTimeoutUsec = 500'000;

    while (true) {
        if (stop_requested && stop_requested()) {
            break;
        }
        if (!tcp_server_has_client(node_)) {
            tcp_server_wait_client(node_, kTimeoutUsec, buffer_size_);
            continue;
        }
        handleClient(handler, kTimeoutUsec);
    }
}

void TcpDataServer::close() {
    listening_ = false;
    if (node_) {
        tcp_close(node_);
        std::free(node_);
        node_ = nullptr;
    }
}

}  // namespace robot::platform
