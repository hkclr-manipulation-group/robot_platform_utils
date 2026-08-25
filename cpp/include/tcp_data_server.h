#ifndef TCP_DATA_SERVER_H
#define TCP_DATA_SERVER_H

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "tcp_data_protocol.h"
#include "tcp_data_services.h"

extern "C" {
#include "curi_tcp/c/src/curi_tcp.h"
#include "curi_tcp/c/src/curi_tcp_framed.h"
}

namespace robot::platform {

/**
 * TCP data channel server (v3): Ping, blob upload, RPC, blob download.
 * Requires UDP session credentials on authenticated requests (Scheme A).
 * Pair with TcpDataClient on the SDK side. Not used for real-time motion.
 */
class TcpDataServer {
public:
    /** Returns 0 when authorized; otherwise an application auth status (401/403/409). */
    using SessionAuthValidator = std::function<std::uint32_t(const tcp_data::RpcSessionAuth& auth, std::string& error_message)>;

    struct BlobReceipt {
        std::string name;
        std::vector<std::uint8_t> data;
    };

    struct RpcResult {
        bool ok = false;
        std::uint32_t status_code = 0;
        std::vector<std::uint8_t> response_body;
        std::string error;

        /** When non-empty, server streams BlobDownload* after RpcResponse. */
        std::string download_name;
        std::vector<std::uint8_t> download_data;
    };

    class DataChannelHandler {
    public:
        virtual ~DataChannelHandler() = default;
        virtual bool onBlobComplete(const BlobReceipt& receipt, std::string& error_message) = 0;
        virtual RpcResult onRpcRequest(std::uint32_t service_id, std::uint32_t method_id,
                                       const std::uint8_t* request_body, std::size_t request_size) = 0;
    };

    static constexpr int kDefaultPort = 8890;

    TcpDataServer();
    ~TcpDataServer();

    TcpDataServer(const TcpDataServer&) = delete;
    TcpDataServer& operator=(const TcpDataServer&) = delete;

    bool listen(const std::string& bind_ip, int port = kDefaultPort, int buffer_size = 65536);

    /** Block until @p stop_requested becomes true. Handles one client at a time. */
    void serveForever(DataChannelHandler& handler, const std::function<bool()>& stop_requested,
                      SessionAuthValidator auth_validator = {});

    void close();

private:
    bool sendFrame(tcp_data::MessageKind kind, std::uint32_t sequence, const std::uint8_t* payload,
                   std::uint32_t payload_size);
    bool sendAck(std::uint32_t sequence, std::uint32_t status_code, int timeout_usec);
    bool sendError(std::uint32_t sequence, const std::string& message, int timeout_usec);
    bool sendRpcResponse(std::uint32_t sequence, std::uint32_t status_code,
                         const std::vector<std::uint8_t>& body);
    bool sendDownloadStream(std::uint32_t sequence, const std::string& name,
                              const std::vector<std::uint8_t>& data, int timeout_usec);
    bool handleClient(DataChannelHandler& handler, int timeout_usec, const SessionAuthValidator& auth_validator);
    std::uint32_t authorizeRequest(const SessionAuthValidator& auth_validator, const tcp_data::RpcSessionAuth& auth,
                                   std::string& error_message) const;

    tcp_node* node_ = nullptr;
    int buffer_size_ = 65536;
    std::vector<std::uint8_t> tx_buffer_;
    std::vector<std::uint8_t> rx_buffer_;
    bool listening_ = false;
};

}  // namespace robot::platform

#endif
