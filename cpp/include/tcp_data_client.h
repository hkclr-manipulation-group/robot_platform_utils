#ifndef TCP_DATA_CLIENT_H
#define TCP_DATA_CLIENT_H

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
 * TCP data channel client (v3): Ping, blob upload/download, RPC services.
 * Authenticated requests carry UDP session credentials (Scheme A).
 * Real-time motion remains on UDP.
 */
class TcpDataClient {
public:
    struct Result {
        bool ok = false;
        std::string message;
        std::uint32_t status_code = 0;
        std::vector<std::uint8_t> response_body;
    };

    static constexpr int kDefaultPort = 8890;
    static constexpr int kDefaultTimeoutUsec = 5'000'000;
    /** Fast-fail connect budget when probing optional TcpDataServer (legacy rt_control). */
    static constexpr int kDefaultConnectTimeoutUsec = 500'000;

    TcpDataClient();
    ~TcpDataClient();

    TcpDataClient(const TcpDataClient&) = delete;
    TcpDataClient& operator=(const TcpDataClient&) = delete;

    bool connect(const std::string& host, int port = kDefaultPort, int buffer_size = 65536,
                 int connect_timeout_usec = kDefaultConnectTimeoutUsec);
    bool isConnected() const;

    /**
     * Bind UDP session credentials used on authenticated TCP requests.
     * @p next_sequence should allocate the next UDP core sequence id (shared with CoreUdpClient).
     */
    void setSessionCredentials(std::uint16_t client_id, std::uint32_t session_id,
                               std::function<std::uint32_t()> allocate_core_sequence);
    void clearSessionCredentials();
    bool hasSessionCredentials() const;

    Result ping(int timeout_usec = kDefaultTimeoutUsec);

    Result uploadBlob(const std::string& name, const std::uint8_t* data, std::size_t size,
                      std::uint32_t chunk_size = tcp_data::kDefaultChunkSize,
                      int timeout_usec = kDefaultTimeoutUsec);

    Result uploadBlob(const std::string& name, const std::vector<std::uint8_t>& data,
                      std::uint32_t chunk_size = tcp_data::kDefaultChunkSize,
                      int timeout_usec = kDefaultTimeoutUsec);

    Result callRpc(tcp_data::ServiceId service, std::uint32_t method, const std::vector<std::uint8_t>& request_body,
                   int timeout_usec = kDefaultTimeoutUsec);

    Result listFiles(const std::string& prefix, std::vector<tcp_data::FileEntry>& entries,
                     int timeout_usec = kDefaultTimeoutUsec);

    Result deleteFile(const std::string& name, int timeout_usec = kDefaultTimeoutUsec);

    Result statFile(const std::string& name, tcp_data::StatResult& stat, int timeout_usec = kDefaultTimeoutUsec);

    Result downloadBlob(const std::string& name, std::vector<std::uint8_t>& data,
                        int timeout_usec = kDefaultTimeoutUsec);

    Result getCapabilities(std::vector<std::uint8_t>& capabilities_body, int timeout_usec = kDefaultTimeoutUsec);

    Result callNetworkConfig(tcp_data::NetworkMethod method, const std::vector<std::uint8_t>& request_body,
                             tcp_data::NetworkConfigData& response, int timeout_usec = kDefaultTimeoutUsec);

    Result pollLogs(std::uint64_t after_id, tcp_data::LogPollResult& result,
                    int timeout_usec = kDefaultTimeoutUsec);

    Result getConfig(tcp_data::RtConfigType type, tcp_data::ConfigValuePayload& result,
                     int timeout_usec = kDefaultTimeoutUsec);

    Result setConfig(tcp_data::RtConfigType type, const std::vector<std::int32_t>& values,
                     tcp_data::ConfigValuePayload& result, int timeout_usec = kDefaultTimeoutUsec);

    /** Re-baseline encoder_last_position.bin (requires Recovery system state). */
    Result resetEncoderLastPosition(int timeout_usec = kDefaultTimeoutUsec);

    /** Read persisted arm serial number (empty string when unset). */
    Result getArmSerialNumber(std::string& out_serial, int timeout_usec = kDefaultTimeoutUsec);

    /** Persist arm serial number (non-empty, max 64 chars). */
    Result setArmSerialNumber(const std::string& serial, int timeout_usec = kDefaultTimeoutUsec);

    /** Persist current runtime settings to disk (System::FlushRuntimeConfig). */
    Result flushRuntimeConfig(int timeout_usec = kDefaultTimeoutUsec);

    Result flushRuntimeSettings(int timeout_usec = kDefaultTimeoutUsec);

    Result listRuntimeProfiles(std::string& profiles_json, int timeout_usec = kDefaultTimeoutUsec);

    Result activateRuntimeProfile(const std::string& profile_id, int timeout_usec = kDefaultTimeoutUsec);

    Result createRuntimeProfile(const std::string& name, std::string& profile_id,
                              int timeout_usec = kDefaultTimeoutUsec);

    Result getRuntimeConfigJson(std::string& json, int timeout_usec = kDefaultTimeoutUsec);

    Result getArmEffectiveSettingsJson(std::uint32_t arm_index, std::string& json,
                                       int timeout_usec = kDefaultTimeoutUsec);

    Result setArmSafetySettingsJson(std::uint32_t arm_index, const std::string& json,
                                    int timeout_usec = kDefaultTimeoutUsec);

    void close();

private:
    Result sendMessage(tcp_data::MessageKind kind, std::uint32_t sequence, const std::uint8_t* payload,
                       std::uint32_t payload_size);

    Result expectAck(std::uint32_t sequence, int timeout_usec);
    Result expectRpcResponse(std::uint32_t sequence, int timeout_usec);
    Result receiveFrame(int timeout_usec);
    Result receiveDownloadStream(std::uint32_t sequence, const std::string& expected_name,
                                 std::vector<std::uint8_t>& data, int timeout_usec);

    Result requireSessionCredentials() const;
    tcp_data::RpcSessionAuth nextSessionAuth();

    std::uint16_t session_client_id_ = 0;
    std::uint32_t session_id_ = 0;
    std::function<std::uint32_t()> allocate_core_sequence_;
    bool has_session_credentials_ = false;

    tcp_node* node_ = nullptr;
    int buffer_size_ = 65536;
    std::vector<std::uint8_t> tx_buffer_;
    std::vector<std::uint8_t> rx_buffer_;
    int last_frame_len_ = 0;
    std::uint32_t next_sequence_ = 1;
    bool connected_ = false;
};

}  // namespace robot::platform

#endif
