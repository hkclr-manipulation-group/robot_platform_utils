#include "tcp_data_client.h"

#include <cstring>
#include <stdexcept>

namespace robot::platform {
namespace {

std::string kindName(tcp_data::MessageKind kind)
{
    switch (kind) {
        case tcp_data::MessageKind::kPing: return "Ping";
        case tcp_data::MessageKind::kPong: return "Pong";
        case tcp_data::MessageKind::kBlobBegin: return "BlobBegin";
        case tcp_data::MessageKind::kBlobChunk: return "BlobChunk";
        case tcp_data::MessageKind::kBlobEnd: return "BlobEnd";
        case tcp_data::MessageKind::kAck: return "Ack";
        case tcp_data::MessageKind::kError: return "Error";
        default: return "Unknown";
    }
}

}  // namespace

TcpDataClient::TcpDataClient() = default;

TcpDataClient::~TcpDataClient() {
    close();
}

bool TcpDataClient::connect(const std::string& host, int port, int buffer_size) {
    close();

    buffer_size_ = buffer_size;
    tx_buffer_.assign(static_cast<std::size_t>(buffer_size_), 0);
    rx_buffer_.assign(static_cast<std::size_t>(buffer_size_), 0);

    node_ = static_cast<tcp_node*>(std::malloc(sizeof(tcp_node)));
    if (!node_) {
        return false;
    }
    std::memset(node_, 0, sizeof(tcp_node));

    if (tcp_init(node_, host.c_str(), port, buffer_size_, false) != 0) {
        close();
        return false;
    }

    connected_ = true;
    next_sequence_ = 1;
    return true;
}

bool TcpDataClient::isConnected() const {
    return connected_ && node_ != nullptr;
}

TcpDataClient::Result TcpDataClient::sendMessage(
    tcp_data::MessageKind kind,
    std::uint32_t sequence,
    const std::uint8_t* payload,
    std::uint32_t payload_size) {
    Result result{};
    if (!isConnected()) {
        result.message = "TcpDataClient: not connected";
        return result;
    }

    const std::size_t frame_size = sizeof(tcp_data::MessageHeader) + payload_size;
    if (frame_size > tx_buffer_.size()) {
        result.message = "TcpDataClient: frame exceeds buffer size";
        return result;
    }

    auto* header = reinterpret_cast<tcp_data::MessageHeader*>(tx_buffer_.data());
    header->magic = tcp_data::kMagic;
    header->version = tcp_data::kVersion;
    header->kind = static_cast<std::uint16_t>(kind);
    header->sequence = sequence;
    header->payload_size = payload_size;

    if (payload_size > 0 && payload != nullptr) {
        std::memcpy(
            tx_buffer_.data() + sizeof(tcp_data::MessageHeader),
            payload,
            payload_size);
    }

    if (tcp_send_frame(node_, tx_buffer_.data(), static_cast<std::uint32_t>(frame_size)) != 0) {
        result.message = "TcpDataClient: tcp_send_frame failed";
        close();
        return result;
    }

    result.ok = true;
    return result;
}

TcpDataClient::Result TcpDataClient::expectAck(std::uint32_t sequence, int timeout_usec) {
    Result result{};
    if (!isConnected()) {
        result.message = "TcpDataClient: not connected";
        return result;
    }

    const int frame_len = tcp_recv_frame(
        node_,
        rx_buffer_.data(),
        static_cast<std::uint32_t>(rx_buffer_.size()),
        timeout_usec);

    if (frame_len <= 0) {
        result.message = frame_len == 0 ? "TcpDataClient: response timeout"
                                        : "TcpDataClient: receive failed";
        if (frame_len < 0) {
            close();
        }
        return result;
    }

    if (static_cast<std::size_t>(frame_len) < sizeof(tcp_data::MessageHeader)) {
        result.message = "TcpDataClient: truncated response header";
        return result;
    }

    const auto* header = reinterpret_cast<const tcp_data::MessageHeader*>(rx_buffer_.data());
    if (header->magic != tcp_data::kMagic || header->version != tcp_data::kVersion) {
        result.message = "TcpDataClient: invalid response magic/version";
        return result;
    }

    const auto kind = static_cast<tcp_data::MessageKind>(header->kind);
    if (kind == tcp_data::MessageKind::kError) {
        result.message = "TcpDataClient: server error";
        if (header->payload_size > 0 &&
            sizeof(tcp_data::MessageHeader) + header->payload_size <=
                static_cast<std::uint32_t>(frame_len)) {
            const char* text = reinterpret_cast<const char*>(
                rx_buffer_.data() + sizeof(tcp_data::MessageHeader));
            result.message.assign(text, text + header->payload_size);
        }
        return result;
    }

    if (kind != tcp_data::MessageKind::kAck) {
        result.message = "TcpDataClient: unexpected response kind " + kindName(kind);
        return result;
    }

    if (header->sequence != sequence) {
        result.message = "TcpDataClient: sequence mismatch in Ack";
        return result;
    }

    if (header->payload_size >= sizeof(std::uint32_t) &&
        sizeof(tcp_data::MessageHeader) + header->payload_size <=
            static_cast<std::uint32_t>(frame_len)) {
        std::memcpy(
            &result.status_code,
            rx_buffer_.data() + sizeof(tcp_data::MessageHeader),
            sizeof(std::uint32_t));
    }

    result.ok = true;
    result.message = "ok";
    return result;
}

TcpDataClient::Result TcpDataClient::ping(int timeout_usec) {
    const std::uint32_t sequence = next_sequence_++;
    Result sent = sendMessage(tcp_data::MessageKind::kPing, sequence, nullptr, 0);
    if (!sent.ok) {
        return sent;
    }

    const int frame_len = tcp_recv_frame(
        node_,
        rx_buffer_.data(),
        static_cast<std::uint32_t>(rx_buffer_.size()),
        timeout_usec);
    if (frame_len <= 0) {
        Result result{};
        result.message = frame_len == 0 ? "TcpDataClient: ping timeout"
                                        : "TcpDataClient: ping receive failed";
        if (frame_len < 0) {
            close();
        }
        return result;
    }

    if (static_cast<std::size_t>(frame_len) < sizeof(tcp_data::MessageHeader)) {
        return Result{false, "TcpDataClient: truncated pong header", 0};
    }

    const auto* header = reinterpret_cast<const tcp_data::MessageHeader*>(rx_buffer_.data());
    if (header->magic != tcp_data::kMagic ||
        static_cast<tcp_data::MessageKind>(header->kind) != tcp_data::MessageKind::kPong ||
        header->sequence != sequence) {
        return Result{false, "TcpDataClient: invalid pong", 0};
    }

    return Result{true, "pong", 0};
}

TcpDataClient::Result TcpDataClient::uploadBlob(
    const std::string& name,
    const std::uint8_t* data,
    std::size_t size,
    std::uint32_t chunk_size,
    int timeout_usec) {
    Result result{};
    if (!isConnected()) {
        result.message = "TcpDataClient: not connected";
        return result;
    }
    if (name.empty()) {
        result.message = "TcpDataClient: blob name is empty";
        return result;
    }
    if (size > 0 && data == nullptr) {
        result.message = "TcpDataClient: blob data is null";
        return result;
    }
    if (chunk_size == 0) {
        chunk_size = tcp_data::kDefaultChunkSize;
    }

    const std::uint32_t transfer_id = next_sequence_++;

    std::vector<std::uint8_t> begin_payload(
        sizeof(std::uint32_t) + name.size() + sizeof(std::uint32_t));
    std::uint32_t name_len = static_cast<std::uint32_t>(name.size());
    std::uint32_t total_size = static_cast<std::uint32_t>(size);
    std::size_t offset = 0;
    std::memcpy(begin_payload.data() + offset, &name_len, sizeof(name_len));
    offset += sizeof(name_len);
    std::memcpy(begin_payload.data() + offset, name.data(), name.size());
    offset += name.size();
    std::memcpy(begin_payload.data() + offset, &total_size, sizeof(total_size));

    Result sent = sendMessage(
        tcp_data::MessageKind::kBlobBegin,
        transfer_id,
        begin_payload.data(),
        static_cast<std::uint32_t>(begin_payload.size()));
    if (!sent.ok) {
        return sent;
    }

    Result begin_ack = expectAck(transfer_id, timeout_usec);
    if (!begin_ack.ok) {
        return begin_ack;
    }

    std::size_t uploaded = 0;
    while (uploaded < size) {
        const std::size_t remaining = size - uploaded;
        const std::uint32_t this_chunk =
            static_cast<std::uint32_t>(std::min<std::size_t>(remaining, chunk_size));

        std::vector<std::uint8_t> chunk_payload(sizeof(std::uint32_t) + this_chunk);
        const std::uint32_t chunk_offset = static_cast<std::uint32_t>(uploaded);
        std::memcpy(chunk_payload.data(), &chunk_offset, sizeof(chunk_offset));
        std::memcpy(
            chunk_payload.data() + sizeof(std::uint32_t),
            data + uploaded,
            this_chunk);

        sent = sendMessage(
            tcp_data::MessageKind::kBlobChunk,
            transfer_id,
            chunk_payload.data(),
            static_cast<std::uint32_t>(chunk_payload.size()));
        if (!sent.ok) {
            return sent;
        }

        uploaded += this_chunk;
    }

    sent = sendMessage(tcp_data::MessageKind::kBlobEnd, transfer_id, nullptr, 0);
    if (!sent.ok) {
        return sent;
    }

    return expectAck(transfer_id, timeout_usec);
}

TcpDataClient::Result TcpDataClient::uploadBlob(
    const std::string& name,
    const std::vector<std::uint8_t>& data,
    std::uint32_t chunk_size,
    int timeout_usec) {
    return uploadBlob(
        name,
        data.empty() ? nullptr : data.data(),
        data.size(),
        chunk_size,
        timeout_usec);
}

void TcpDataClient::close() {
    connected_ = false;
    if (node_) {
        tcp_close(node_);
        std::free(node_);
        node_ = nullptr;
    }
}

}  // namespace robot::platform
