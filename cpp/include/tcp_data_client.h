#ifndef TCP_DATA_CLIENT_H
#define TCP_DATA_CLIENT_H

#include <cstdint>
#include <string>
#include <vector>

#include "tcp_data_protocol.h"

extern "C" {
#include "curi_tcp/c/src/curi_tcp.h"
#include "curi_tcp/c/src/curi_tcp_framed.h"
}

namespace robot::platform {

/**
 * Cold-path TCP client for bulk / non-real-time data (teach files, logs, etc.).
 *
 * Transport: curi_tcp length-prefixed frames (see curi_tcp_framed.h).
 * Application: tcp_data::MessageHeader + payload (see tcp_data_protocol.h).
 *
 * Real-time motion remains on UDP; this class is intentionally separate from CoreUdpClient.
 */
class TcpDataClient {
public:
    struct Result {
        bool ok = false;
        std::string message;
        std::uint32_t status_code = 0;
    };

    static constexpr int kDefaultPort = 8890;
    static constexpr int kDefaultTimeoutUsec = 5'000'000;

    TcpDataClient();
    ~TcpDataClient();

    TcpDataClient(const TcpDataClient&) = delete;
    TcpDataClient& operator=(const TcpDataClient&) = delete;

    /** Connect as TCP client. */
    bool connect(const std::string& host, int port = kDefaultPort, int buffer_size = 65536);

    bool isConnected() const;

    /** Round-trip latency check. */
    Result ping(int timeout_usec = kDefaultTimeoutUsec);

    /**
     * Upload a binary blob in chunked frames (BlobBegin → BlobChunk* → BlobEnd).
     * Suitable for teach programs, calibration files, logs, etc.
     */
    Result uploadBlob(
        const std::string& name,
        const std::uint8_t* data,
        std::size_t size,
        std::uint32_t chunk_size = tcp_data::kDefaultChunkSize,
        int timeout_usec = kDefaultTimeoutUsec);

    Result uploadBlob(
        const std::string& name,
        const std::vector<std::uint8_t>& data,
        std::uint32_t chunk_size = tcp_data::kDefaultChunkSize,
        int timeout_usec = kDefaultTimeoutUsec);

    void close();

private:
    Result sendMessage(
        tcp_data::MessageKind kind,
        std::uint32_t sequence,
        const std::uint8_t* payload,
        std::uint32_t payload_size);

    Result expectAck(std::uint32_t sequence, int timeout_usec);

    tcp_node* node_ = nullptr;
    int buffer_size_ = 65536;
    std::vector<std::uint8_t> tx_buffer_;
    std::vector<std::uint8_t> rx_buffer_;
    std::uint32_t next_sequence_ = 1;
    bool connected_ = false;
};

}  // namespace robot::platform

#endif
