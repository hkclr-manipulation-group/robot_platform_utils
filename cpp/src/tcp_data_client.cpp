#include "tcp_data_client.h"

#include <algorithm>
#include <cstring>

namespace robot::platform {
namespace {

std::string kindName(tcp_data::MessageKind kind) {
    switch (kind) {
        case tcp_data::MessageKind::kPing:
            return "Ping";
        case tcp_data::MessageKind::kPong:
            return "Pong";
        case tcp_data::MessageKind::kBlobBegin:
            return "BlobBegin";
        case tcp_data::MessageKind::kBlobChunk:
            return "BlobChunk";
        case tcp_data::MessageKind::kBlobEnd:
            return "BlobEnd";
        case tcp_data::MessageKind::kRpcRequest:
            return "RpcRequest";
        case tcp_data::MessageKind::kRpcResponse:
            return "RpcResponse";
        case tcp_data::MessageKind::kBlobDownloadBegin:
            return "BlobDownloadBegin";
        case tcp_data::MessageKind::kBlobDownloadChunk:
            return "BlobDownloadChunk";
        case tcp_data::MessageKind::kBlobDownloadEnd:
            return "BlobDownloadEnd";
        case tcp_data::MessageKind::kAck:
            return "Ack";
        case tcp_data::MessageKind::kError:
            return "Error";
        default:
            return "Unknown";
    }
}

bool readU32(const std::uint8_t* data, std::size_t size, std::size_t& offset, std::uint32_t& out) {
    if (offset + sizeof(std::uint32_t) > size) {
        return false;
    }
    std::memcpy(&out, data + offset, sizeof(out));
    offset += sizeof(std::uint32_t);
    return true;
}

}  // namespace

TcpDataClient::TcpDataClient() = default;

TcpDataClient::~TcpDataClient() {
    close();
}

bool TcpDataClient::connect(const std::string& host, int port, int buffer_size, int connect_timeout_usec) {
    close();

    buffer_size_ = buffer_size;
    tx_buffer_.assign(static_cast<std::size_t>(buffer_size_), 0);
    rx_buffer_.assign(static_cast<std::size_t>(buffer_size_), 0);

    node_ = static_cast<tcp_node*>(std::malloc(sizeof(tcp_node)));
    if (!node_) {
        return false;
    }
    std::memset(node_, 0, sizeof(tcp_node));

    if (tcp_init_ex(node_, host.c_str(), port, buffer_size_, false, connect_timeout_usec) != 0) {
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

void TcpDataClient::setSessionCredentials(std::uint16_t client_id, std::uint32_t session_id,
                                          std::function<std::uint32_t()> allocate_core_sequence) {
    session_client_id_ = client_id;
    session_id_ = session_id;
    allocate_core_sequence_ = std::move(allocate_core_sequence);
    has_session_credentials_ = static_cast<bool>(allocate_core_sequence_);
}

void TcpDataClient::clearSessionCredentials() {
    session_client_id_ = 0;
    session_id_ = 0;
    allocate_core_sequence_ = nullptr;
    has_session_credentials_ = false;
}

bool TcpDataClient::hasSessionCredentials() const {
    return has_session_credentials_;
}

TcpDataClient::Result TcpDataClient::requireSessionCredentials() const {
    if (!has_session_credentials_ || !allocate_core_sequence_) {
        return Result{false, "TcpDataClient: session credentials not set", 0};
    }
    return Result{true, "ok", 0};
}

tcp_data::RpcSessionAuth TcpDataClient::nextSessionAuth() {
    tcp_data::RpcSessionAuth auth{};
    auth.client_id = session_client_id_;
    auth.session_id = session_id_;
    auth.sequence_id = allocate_core_sequence_ ? allocate_core_sequence_() : 0;
    return auth;
}

TcpDataClient::Result TcpDataClient::sendMessage(tcp_data::MessageKind kind, std::uint32_t sequence,
                                                 const std::uint8_t* payload, std::uint32_t payload_size) {
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
        std::memcpy(tx_buffer_.data() + sizeof(tcp_data::MessageHeader), payload, payload_size);
    }

    if (tcp_send_frame(node_, tx_buffer_.data(), static_cast<std::uint32_t>(frame_size)) != 0) {
        result.message = "TcpDataClient: tcp_send_frame failed";
        close();
        return result;
    }

    result.ok = true;
    return result;
}

TcpDataClient::Result TcpDataClient::receiveFrame(int timeout_usec) {
    Result result{};
    if (!isConnected()) {
        result.message = "TcpDataClient: not connected";
        return result;
    }

    last_frame_len_ = tcp_recv_frame(node_, rx_buffer_.data(), static_cast<std::uint32_t>(rx_buffer_.size()),
                                     timeout_usec);
    if (last_frame_len_ <= 0) {
        result.message = last_frame_len_ == 0 ? "TcpDataClient: response timeout"
                                              : "TcpDataClient: receive failed";
        if (last_frame_len_ < 0) {
            close();
        }
        return result;
    }

    if (static_cast<std::size_t>(last_frame_len_) < sizeof(tcp_data::MessageHeader)) {
        result.message = "TcpDataClient: truncated response header";
        return result;
    }

    const auto* header = reinterpret_cast<const tcp_data::MessageHeader*>(rx_buffer_.data());
    if (header->magic != tcp_data::kMagic || header->version != tcp_data::kVersion) {
        result.message = "TcpDataClient: invalid response magic/version";
        return result;
    }

    result.ok = true;
    return result;
}

TcpDataClient::Result TcpDataClient::expectAck(std::uint32_t sequence, int timeout_usec) {
    Result received = receiveFrame(timeout_usec);
    if (!received.ok) {
        return received;
    }

    const auto* header = reinterpret_cast<const tcp_data::MessageHeader*>(rx_buffer_.data());
    const auto kind = static_cast<tcp_data::MessageKind>(header->kind);
    if (kind == tcp_data::MessageKind::kError) {
        Result result{};
        result.message = "TcpDataClient: server error";
        if (header->payload_size > 0 &&
            sizeof(tcp_data::MessageHeader) + header->payload_size <= static_cast<std::uint32_t>(last_frame_len_)) {
            const char* text = reinterpret_cast<const char*>(rx_buffer_.data() + sizeof(tcp_data::MessageHeader));
            result.message.assign(text, text + header->payload_size);
        }
        return result;
    }

    if (kind != tcp_data::MessageKind::kAck) {
        Result result{};
        result.message = "TcpDataClient: unexpected response kind " + kindName(kind);
        return result;
    }

    if (header->sequence != sequence) {
        Result result{};
        result.message = "TcpDataClient: sequence mismatch in Ack";
        return result;
    }

    Result result{};
    if (header->payload_size >= sizeof(std::uint32_t) &&
        sizeof(tcp_data::MessageHeader) + header->payload_size <= static_cast<std::uint32_t>(last_frame_len_)) {
        std::memcpy(&result.status_code, rx_buffer_.data() + sizeof(tcp_data::MessageHeader), sizeof(std::uint32_t));
    }
    result.ok = result.status_code == 0;
    result.message = result.ok ? "ok"
                               : ("TcpDataClient: request rejected (status " + std::to_string(result.status_code) + ")");
    return result;
}

TcpDataClient::Result TcpDataClient::expectRpcResponse(std::uint32_t sequence, int timeout_usec) {
    Result received = receiveFrame(timeout_usec);
    if (!received.ok) {
        return received;
    }

    const auto* header = reinterpret_cast<const tcp_data::MessageHeader*>(rx_buffer_.data());
    const auto kind = static_cast<tcp_data::MessageKind>(header->kind);
    if (kind == tcp_data::MessageKind::kError) {
        Result result{};
        result.message = "TcpDataClient: server error";
        if (header->payload_size > 0 &&
            sizeof(tcp_data::MessageHeader) + header->payload_size <= static_cast<std::uint32_t>(last_frame_len_)) {
            const char* text = reinterpret_cast<const char*>(rx_buffer_.data() + sizeof(tcp_data::MessageHeader));
            result.message.assign(text, text + header->payload_size);
        }
        return result;
    }

    if (kind != tcp_data::MessageKind::kRpcResponse || header->sequence != sequence) {
        Result result{};
        result.message = "TcpDataClient: unexpected RpcResponse";
        return result;
    }

    const std::uint8_t* payload = rx_buffer_.data() + sizeof(tcp_data::MessageHeader);
    const std::size_t payload_size = header->payload_size;
    const std::uint8_t* body = nullptr;
    std::size_t body_size = 0;
    if (!tcp_data::decodeRpcResponse(payload, payload_size, received.status_code, body, body_size)) {
        Result result{};
        result.message = "TcpDataClient: invalid RpcResponse payload";
        return result;
    }

    received.response_body.assign(body, body + body_size);
    received.message = "ok";
    return received;
}

TcpDataClient::Result TcpDataClient::ping(int timeout_usec) {
    Result creds = requireSessionCredentials();
    if (!creds.ok) {
        return creds;
    }

    const std::uint32_t sequence = next_sequence_++;
    std::vector<std::uint8_t> payload;
    if (!tcp_data::encodeRpcSessionAuth(nextSessionAuth(), payload)) {
        return Result{false, "TcpDataClient: failed to encode Ping auth", 0};
    }

    Result sent = sendMessage(tcp_data::MessageKind::kPing, sequence, payload.data(),
                              static_cast<std::uint32_t>(payload.size()));
    if (!sent.ok) {
        return sent;
    }

    Result received = receiveFrame(timeout_usec);
    if (!received.ok) {
        return received;
    }

    const auto* header = reinterpret_cast<const tcp_data::MessageHeader*>(rx_buffer_.data());
    const auto kind = static_cast<tcp_data::MessageKind>(header->kind);
    if (kind == tcp_data::MessageKind::kAck) {
        Result ack{};
        if (header->payload_size >= sizeof(std::uint32_t)
            && sizeof(tcp_data::MessageHeader) + header->payload_size
                   <= static_cast<std::uint32_t>(last_frame_len_)) {
            std::memcpy(&ack.status_code, rx_buffer_.data() + sizeof(tcp_data::MessageHeader), sizeof(std::uint32_t));
        }
        ack.ok = ack.status_code == 0;
        if (!ack.ok) {
            ack.message = "TcpDataClient: ping unauthorized (status " + std::to_string(ack.status_code) + ")";
        } else {
            ack.message = "TcpDataClient: unexpected Ack on ping";
            ack.ok = false;
        }
        return ack;
    }

    if (kind != tcp_data::MessageKind::kPong || header->sequence != sequence) {
        return Result{false, "TcpDataClient: invalid pong", 0};
    }

    return Result{true, "pong", 0};
}

TcpDataClient::Result TcpDataClient::uploadBlob(const std::string& name, const std::uint8_t* data, std::size_t size,
                                                std::uint32_t chunk_size, int timeout_usec) {
    Result result{};
    if (!isConnected()) {
        result.message = "TcpDataClient: not connected";
        return result;
    }
    Result creds = requireSessionCredentials();
    if (!creds.ok) {
        return creds;
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

    std::vector<std::uint8_t> begin_payload;
    if (!tcp_data::encodeBlobBeginPayload(nextSessionAuth(), name, static_cast<std::uint32_t>(size), begin_payload)) {
        return Result{false, "TcpDataClient: failed to encode BlobBegin payload", 0};
    }

    Result sent = sendMessage(tcp_data::MessageKind::kBlobBegin, transfer_id, begin_payload.data(),
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
        std::memcpy(chunk_payload.data() + sizeof(std::uint32_t), data + uploaded, this_chunk);

        sent = sendMessage(tcp_data::MessageKind::kBlobChunk, transfer_id, chunk_payload.data(),
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

TcpDataClient::Result TcpDataClient::uploadBlob(const std::string& name, const std::vector<std::uint8_t>& data,
                                                std::uint32_t chunk_size, int timeout_usec) {
    return uploadBlob(name, data.empty() ? nullptr : data.data(), data.size(), chunk_size, timeout_usec);
}

TcpDataClient::Result TcpDataClient::callRpc(tcp_data::ServiceId service, std::uint32_t method,
                                             const std::vector<std::uint8_t>& request_body, int timeout_usec) {
    Result creds = requireSessionCredentials();
    if (!creds.ok) {
        return creds;
    }

    const std::uint32_t sequence = next_sequence_++;
    std::vector<std::uint8_t> payload;
    if (!tcp_data::encodeRpcRequest(nextSessionAuth(), service, method, request_body, payload)) {
        return Result{false, "TcpDataClient: failed to encode RpcRequest", 0};
    }

    Result sent = sendMessage(tcp_data::MessageKind::kRpcRequest, sequence, payload.data(),
                              static_cast<std::uint32_t>(payload.size()));
    if (!sent.ok) {
        return sent;
    }

    Result response = expectRpcResponse(sequence, timeout_usec);
    if (!response.ok) {
        return response;
    }
    if (response.status_code != 0) {
        response.ok = false;
        if (response.message == "ok") {
            response.message = "TcpDataClient: RPC failed with status " + std::to_string(response.status_code);
        }
    }
    return response;
}

TcpDataClient::Result TcpDataClient::listFiles(const std::string& prefix, std::vector<tcp_data::FileEntry>& entries,
                                               int timeout_usec) {
    std::vector<std::uint8_t> request;
    if (!tcp_data::encodePrefixRequest(prefix, request)) {
        return Result{false, "TcpDataClient: failed to encode list request", 0};
    }

    Result rpc = callRpc(tcp_data::ServiceId::kStorage, static_cast<std::uint32_t>(tcp_data::StorageMethod::kList),
                         request, timeout_usec);
    if (!rpc.ok) {
        return rpc;
    }

    if (!tcp_data::decodeFileList(rpc.response_body.data(), rpc.response_body.size(), entries)) {
        return Result{false, "TcpDataClient: failed to decode file list", rpc.status_code};
    }
    return rpc;
}

TcpDataClient::Result TcpDataClient::deleteFile(const std::string& name, int timeout_usec) {
    std::vector<std::uint8_t> request;
    if (!tcp_data::encodeNameRequest(name, request)) {
        return Result{false, "TcpDataClient: failed to encode delete request", 0};
    }

    return callRpc(tcp_data::ServiceId::kStorage, static_cast<std::uint32_t>(tcp_data::StorageMethod::kDelete), request,
                   timeout_usec);
}

TcpDataClient::Result TcpDataClient::statFile(const std::string& name, tcp_data::StatResult& stat,
                                              int timeout_usec) {
    std::vector<std::uint8_t> request;
    if (!tcp_data::encodeNameRequest(name, request)) {
        return Result{false, "TcpDataClient: failed to encode stat request", 0};
    }

    Result rpc = callRpc(tcp_data::ServiceId::kStorage, static_cast<std::uint32_t>(tcp_data::StorageMethod::kStat),
                         request, timeout_usec);
    if (!rpc.ok) {
        return rpc;
    }

    if (!tcp_data::decodeStatResult(rpc.response_body.data(), rpc.response_body.size(), stat)) {
        return Result{false, "TcpDataClient: failed to decode stat response", rpc.status_code};
    }
    return rpc;
}

TcpDataClient::Result TcpDataClient::receiveDownloadStream(std::uint32_t sequence, const std::string& expected_name,
                                                           std::vector<std::uint8_t>& data, int timeout_usec) {
    Result received = receiveFrame(timeout_usec);
    if (!received.ok) {
        return received;
    }

    const auto* header = reinterpret_cast<const tcp_data::MessageHeader*>(rx_buffer_.data());
    if (static_cast<tcp_data::MessageKind>(header->kind) != tcp_data::MessageKind::kBlobDownloadBegin ||
        header->sequence != sequence) {
        return Result{false, "TcpDataClient: expected BlobDownloadBegin", 0};
    }

    const std::uint8_t* payload = rx_buffer_.data() + sizeof(tcp_data::MessageHeader);
    const std::size_t payload_size = header->payload_size;
    std::size_t offset = 0;
    std::uint32_t name_len = 0;
    std::uint32_t total_size = 0;
    if (!readU32(payload, payload_size, offset, name_len) || offset + name_len + sizeof(total_size) > payload_size) {
        return Result{false, "TcpDataClient: invalid BlobDownloadBegin", 0};
    }

    std::string name(reinterpret_cast<const char*>(payload + offset), name_len);
    offset += name_len;
    if (!readU32(payload, payload_size, offset, total_size) || name != expected_name) {
        return Result{false, "TcpDataClient: download name mismatch", 0};
    }

    data.assign(total_size, 0);
    bool ended = false;

    while (!ended) {
        received = receiveFrame(timeout_usec);
        if (!received.ok) {
            return received;
        }

        header = reinterpret_cast<const tcp_data::MessageHeader*>(rx_buffer_.data());
        if (header->sequence != sequence) {
            return Result{false, "TcpDataClient: download sequence mismatch", 0};
        }

        const auto kind = static_cast<tcp_data::MessageKind>(header->kind);
        const std::uint8_t* chunk_payload = rx_buffer_.data() + sizeof(tcp_data::MessageHeader);
        const std::size_t chunk_payload_size = header->payload_size;

        if (kind == tcp_data::MessageKind::kBlobDownloadChunk) {
            offset = 0;
            std::uint32_t chunk_offset = 0;
            if (!readU32(chunk_payload, chunk_payload_size, offset, chunk_offset) || offset > chunk_payload_size) {
                return Result{false, "TcpDataClient: invalid BlobDownloadChunk", 0};
            }
            const std::size_t chunk_len = chunk_payload_size - offset;
            if (chunk_offset + chunk_len > data.size()) {
                return Result{false, "TcpDataClient: BlobDownloadChunk out of range", 0};
            }
            std::memcpy(data.data() + chunk_offset, chunk_payload + offset, chunk_len);
        } else if (kind == tcp_data::MessageKind::kBlobDownloadEnd) {
            ended = true;
        } else {
            return Result{false, "TcpDataClient: unexpected frame during download", 0};
        }
    }

    received.ok = true;
    received.message = "ok";
    return received;
}

TcpDataClient::Result TcpDataClient::downloadBlob(const std::string& name, std::vector<std::uint8_t>& data,
                                                  int timeout_usec) {
    Result creds = requireSessionCredentials();
    if (!creds.ok) {
        return creds;
    }

    std::vector<std::uint8_t> request;
    if (!tcp_data::encodeNameRequest(name, request)) {
        return Result{false, "TcpDataClient: failed to encode download request", 0};
    }

    const std::uint32_t sequence = next_sequence_++;
    std::vector<std::uint8_t> payload;
    if (!tcp_data::encodeRpcRequest(nextSessionAuth(), tcp_data::ServiceId::kStorage,
                                    static_cast<std::uint32_t>(tcp_data::StorageMethod::kDownload), request,
                                    payload)) {
        return Result{false, "TcpDataClient: failed to encode RpcRequest", 0};
    }

    Result sent = sendMessage(tcp_data::MessageKind::kRpcRequest, sequence, payload.data(),
                              static_cast<std::uint32_t>(payload.size()));
    if (!sent.ok) {
        return sent;
    }

    Result rpc = expectRpcResponse(sequence, timeout_usec);
    if (!rpc.ok || rpc.status_code != 0) {
        return rpc;
    }

    std::uint64_t expected_size = 0;
    if (!tcp_data::decodeDownloadMeta(rpc.response_body.data(), rpc.response_body.size(), expected_size)) {
        return Result{false, "TcpDataClient: failed to decode download meta", rpc.status_code};
    }

    Result downloaded = receiveDownloadStream(sequence, name, data, timeout_usec);
    if (!downloaded.ok) {
        return downloaded;
    }
    if (data.size() != expected_size) {
        return Result{false, "TcpDataClient: downloaded size mismatch", 0};
    }
    return downloaded;
}

TcpDataClient::Result TcpDataClient::getCapabilities(std::vector<std::uint8_t>& capabilities_body,
                                                     int timeout_usec) {
    Result rpc =
        callRpc(tcp_data::ServiceId::kSystem, static_cast<std::uint32_t>(tcp_data::SystemMethod::kGetCapabilities), {},
                timeout_usec);
    if (!rpc.ok) {
        return rpc;
    }
    capabilities_body = std::move(rpc.response_body);
    return rpc;
}

TcpDataClient::Result TcpDataClient::callNetworkConfig(tcp_data::NetworkMethod method,
                                                       const std::vector<std::uint8_t>& request_body,
                                                       tcp_data::NetworkConfigData& response, int timeout_usec) {
    Result rpc = callRpc(tcp_data::ServiceId::kNetwork, static_cast<std::uint32_t>(method), request_body, timeout_usec);
    if (!rpc.ok) {
        return rpc;
    }
    if (!tcp_data::decodeNetworkConfigData(rpc.response_body.data(), rpc.response_body.size(), response)) {
        return Result{false, "TcpDataClient: failed to decode network response", rpc.status_code};
    }
    return Result{true, "ok", rpc.status_code, {}};
}

TcpDataClient::Result TcpDataClient::pollLogs(std::uint64_t after_id, tcp_data::LogPollResult& result,
                                              int timeout_usec) {
    std::vector<std::uint8_t> request_body;
    if (!tcp_data::encodeLogPollRequest(after_id, request_body)) {
        return Result{false, "TcpDataClient: failed to encode log poll request", 0};
    }
    Result rpc = callRpc(tcp_data::ServiceId::kLog, static_cast<std::uint32_t>(tcp_data::LogMethod::kPoll),
                         request_body, timeout_usec);
    if (!rpc.ok) {
        return rpc;
    }
    if (!tcp_data::decodeLogPollResponse(rpc.response_body.data(), rpc.response_body.size(), result)) {
        return Result{false, "TcpDataClient: failed to decode log poll response", rpc.status_code};
    }
    return Result{true, "ok", rpc.status_code, {}};
}

TcpDataClient::Result TcpDataClient::getConfig(tcp_data::RtConfigType type, tcp_data::ConfigValuePayload& result,
                                                 int timeout_usec) {
    std::vector<std::uint8_t> request_body;
    if (!tcp_data::encodeConfigValuePayload(type, {}, request_body)) {
        return Result{false, "TcpDataClient: failed to encode config get request", 0};
    }
    Result rpc = callRpc(tcp_data::ServiceId::kConfig, static_cast<std::uint32_t>(tcp_data::ConfigMethod::kGet),
                         request_body, timeout_usec);
    if (!rpc.ok) {
        return rpc;
    }
    if (!tcp_data::decodeConfigValuePayload(rpc.response_body.data(), rpc.response_body.size(), result)) {
        return Result{false, "TcpDataClient: failed to decode config get response", rpc.status_code};
    }
    return Result{true, "ok", rpc.status_code, {}};
}

TcpDataClient::Result TcpDataClient::setConfig(tcp_data::RtConfigType type,
                                               const std::vector<std::int32_t>& values,
                                               tcp_data::ConfigValuePayload& result, int timeout_usec) {
    std::vector<std::uint8_t> request_body;
    if (!tcp_data::encodeConfigValuePayload(type, values, request_body)) {
        return Result{false, "TcpDataClient: failed to encode config set request", 0};
    }
    Result rpc = callRpc(tcp_data::ServiceId::kConfig, static_cast<std::uint32_t>(tcp_data::ConfigMethod::kSet),
                         request_body, timeout_usec);
    if (!rpc.ok) {
        return rpc;
    }
    if (!tcp_data::decodeConfigValuePayload(rpc.response_body.data(), rpc.response_body.size(), result)) {
        return Result{false, "TcpDataClient: failed to decode config set response", rpc.status_code};
    }
    return Result{true, "ok", rpc.status_code, {}};
}

void TcpDataClient::close() {
    connected_ = false;
    clearSessionCredentials();
    if (node_) {
        tcp_close(node_);
        std::free(node_);
        node_ = nullptr;
    }
}

}  // namespace robot::platform
