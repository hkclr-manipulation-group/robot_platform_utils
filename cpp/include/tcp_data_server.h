#ifndef TCP_DATA_SERVER_H
#define TCP_DATA_SERVER_H

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "tcp_data_protocol.h"

extern "C" {
#include "curi_tcp/c/src/curi_tcp.h"
#include "curi_tcp/c/src/curi_tcp_framed.h"
}

namespace robot::platform {

/**
 * Minimal cold-path TCP server for bulk data ingestion (teach files, logs, …).
 * Pair with TcpDataClient on the SDK side. Not used for real-time motion.
 */
class TcpDataServer {
public:
    struct BlobReceipt {
        std::string name;
        std::vector<std::uint8_t> data;
    };

    using BlobHandler = std::function<bool(const BlobReceipt& receipt, std::string& error_message)>;

    static constexpr int kDefaultPort = 8890;

    TcpDataServer();
    ~TcpDataServer();

    TcpDataServer(const TcpDataServer&) = delete;
    TcpDataServer& operator=(const TcpDataServer&) = delete;

    bool listen(const std::string& bind_ip, int port = kDefaultPort, int buffer_size = 65536);

    /** Block until @p stop_requested becomes true. Handles one client at a time. */
    void serveForever(const BlobHandler& handler, const std::function<bool()>& stop_requested);

    void close();

private:
    bool sendAck(std::uint32_t sequence, std::uint32_t status_code, int timeout_usec);
    bool sendError(std::uint32_t sequence, const std::string& message, int timeout_usec);
    bool handleClient(const BlobHandler& handler, int timeout_usec);

    tcp_node* node_ = nullptr;
    int buffer_size_ = 65536;
    std::vector<std::uint8_t> tx_buffer_;
    std::vector<std::uint8_t> rx_buffer_;
    bool listening_ = false;
};

}  // namespace robot::platform

#endif
