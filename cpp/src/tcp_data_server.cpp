#include "tcp_data_server.h"

#include "tcp_data_services.h"

#include <algorithm>
#include <cstring>

namespace robot::platform {
namespace {

bool readU32(const std::uint8_t* data, std::size_t size, std::size_t& offset, std::uint32_t& out) {
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

bool TcpDataServer::sendFrame(tcp_data::MessageKind kind, std::uint32_t sequence, const std::uint8_t* payload,
                              std::uint32_t payload_size) {
    const std::size_t frame_size = sizeof(tcp_data::MessageHeader) + payload_size;
    if (frame_size > tx_buffer_.size()) {
        return false;
    }

    auto* header = reinterpret_cast<tcp_data::MessageHeader*>(tx_buffer_.data());
    header->magic = tcp_data::kMagic;
    header->version = tcp_data::kVersion;
    header->kind = static_cast<std::uint16_t>(kind);
    header->sequence = sequence;
    header->payload_size = payload_size;

    if (payload_size > 0 && payload != nullptr) {
        std::memcpy(tx_buffer_.data() + sizeof(tcp_data::MessageHeader), payload, payload_size);
    }

    return tcp_send_frame(node_, tx_buffer_.data(), static_cast<std::uint32_t>(frame_size)) == 0;
}

bool TcpDataServer::sendAck(std::uint32_t sequence, std::uint32_t status_code, int timeout_usec) {
    (void)timeout_usec;
    return sendFrame(tcp_data::MessageKind::kAck, sequence, reinterpret_cast<const std::uint8_t*>(&status_code),
                     sizeof(status_code));
}

bool TcpDataServer::sendError(std::uint32_t sequence, const std::string& message, int timeout_usec) {
    (void)timeout_usec;
    const auto* payload = message.empty() ? nullptr : reinterpret_cast<const std::uint8_t*>(message.data());
    return sendFrame(tcp_data::MessageKind::kError, sequence, payload,
                     static_cast<std::uint32_t>(message.size()));
}

bool TcpDataServer::sendRpcResponse(std::uint32_t sequence, std::uint32_t status_code,
                                    const std::vector<std::uint8_t>& body) {
    std::vector<std::uint8_t> payload;
    if (!tcp_data::encodeRpcResponse(status_code, body, payload)) {
        return false;
    }
    return sendFrame(tcp_data::MessageKind::kRpcResponse, sequence, payload.data(),
                     static_cast<std::uint32_t>(payload.size()));
}

bool TcpDataServer::sendDownloadStream(std::uint32_t sequence, const std::string& name,
                                       const std::vector<std::uint8_t>& data, int timeout_usec) {
    (void)timeout_usec;

    std::vector<std::uint8_t> begin_payload(sizeof(std::uint32_t) + name.size() + sizeof(std::uint32_t));
    std::uint32_t name_len = static_cast<std::uint32_t>(name.size());
    std::uint32_t total_size = static_cast<std::uint32_t>(data.size());
    std::size_t offset = 0;
    std::memcpy(begin_payload.data() + offset, &name_len, sizeof(name_len));
    offset += sizeof(name_len);
    std::memcpy(begin_payload.data() + offset, name.data(), name.size());
    offset += name.size();
    std::memcpy(begin_payload.data() + offset, &total_size, sizeof(total_size));

    if (!sendFrame(tcp_data::MessageKind::kBlobDownloadBegin, sequence, begin_payload.data(),
                   static_cast<std::uint32_t>(begin_payload.size()))) {
        return false;
    }

    std::size_t sent = 0;
    while (sent < data.size()) {
        const std::size_t remaining = data.size() - sent;
        const std::uint32_t chunk_size =
            static_cast<std::uint32_t>(std::min(remaining, static_cast<std::size_t>(tcp_data::kDefaultChunkSize)));

        std::vector<std::uint8_t> chunk_payload(sizeof(std::uint32_t) + chunk_size);
        const std::uint32_t chunk_offset = static_cast<std::uint32_t>(sent);
        std::memcpy(chunk_payload.data(), &chunk_offset, sizeof(chunk_offset));
        std::memcpy(chunk_payload.data() + sizeof(std::uint32_t), data.data() + sent, chunk_size);

        if (!sendFrame(tcp_data::MessageKind::kBlobDownloadChunk, sequence, chunk_payload.data(),
                       static_cast<std::uint32_t>(chunk_payload.size()))) {
            return false;
        }
        sent += chunk_size;
    }

    return sendFrame(tcp_data::MessageKind::kBlobDownloadEnd, sequence, nullptr, 0);
}

bool TcpDataServer::handleClient(DataChannelHandler& handler, int timeout_usec) {
    BlobReceipt active{};
    std::uint32_t active_sequence = 0;
    bool transfer_open = false;

    while (tcp_server_has_client(node_)) {
        const int frame_len = tcp_recv_frame(node_, rx_buffer_.data(),
                                             static_cast<std::uint32_t>(rx_buffer_.size()), timeout_usec);

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

        const auto* header = reinterpret_cast<const tcp_data::MessageHeader*>(rx_buffer_.data());
        if (header->magic != tcp_data::kMagic) {
            sendError(header->sequence, "invalid magic", timeout_usec);
            return false;
        }
        if (header->version != tcp_data::kVersion) {
            sendError(header->sequence, "unsupported protocol version", timeout_usec);
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
                if (!sendFrame(tcp_data::MessageKind::kPong, header->sequence, nullptr, 0)) {
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
                active.name.assign(reinterpret_cast<const char*>(payload + offset), name_len);
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
                if (!readU32(payload, payload_size, offset, chunk_offset) || offset > payload_size) {
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
                const bool ok = handler.onBlobComplete(active, error);
                transfer_open = false;
                if (!ok) {
                    sendError(header->sequence, error.empty() ? "blob handler failed" : error, timeout_usec);
                    return false;
                }
                sendAck(header->sequence, 0, timeout_usec);
                break;
            }
            case tcp_data::MessageKind::kRpcRequest: {
                std::uint32_t service_id = 0;
                std::uint32_t method_id = 0;
                const std::uint8_t* request_body = nullptr;
                std::size_t request_body_size = 0;
                if (!tcp_data::decodeRpcRequest(payload, payload_size, service_id, method_id, request_body,
                                                request_body_size)) {
                    sendError(header->sequence, "invalid RpcRequest payload", timeout_usec);
                    return false;
                }

                RpcResult rpc = handler.onRpcRequest(service_id, method_id, request_body, request_body_size);
                if (!rpc.ok) {
                    sendError(header->sequence, rpc.error.empty() ? "rpc handler failed" : rpc.error, timeout_usec);
                    return false;
                }

                if (!sendRpcResponse(header->sequence, rpc.status_code, rpc.response_body)) {
                    return false;
                }

                if (!rpc.download_data.empty()) {
                    if (!sendDownloadStream(header->sequence, rpc.download_name, rpc.download_data, timeout_usec)) {
                        return false;
                    }
                }
                break;
            }
            default:
                sendError(header->sequence, "unsupported message kind", timeout_usec);
                return false;
        }
    }
    return true;
}

void TcpDataServer::serveForever(DataChannelHandler& handler, const std::function<bool()>& stop_requested) {
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
