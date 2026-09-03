#ifndef TCP_DATA_SERVICES_H
#define TCP_DATA_SERVICES_H

#include <cstdint>
#include <string>
#include <vector>

namespace robot::platform::tcp_data {

/** UDP session credentials carried on TCP data-channel requests (Scheme A). */
struct RpcSessionAuth {
    std::uint16_t client_id = 0;
    std::uint32_t session_id = 0;
    std::uint32_t sequence_id = 0;
};

constexpr std::size_t kRpcSessionAuthSize = sizeof(std::uint16_t) + sizeof(std::uint32_t) + sizeof(std::uint32_t);

enum class ServiceId : std::uint32_t {
    kStorage = 1,
    kTeach = 2,
    kLog = 3,
    kSystem = 4,
    kNetwork = 5,
    kConfig = 6,
    /** Runtime settings persistence and profile management. */
    kRuntimeSettings = 7,
};

/** Mirrors robot::platform::NetworkConfigAction values. */
enum class NetworkMethod : std::uint32_t {
    kGet = 0,
    kSetWifi = 1,
    kSetEthStatic = 2,
    kFactoryReset = 3,
};

enum class StorageMethod : std::uint32_t {
    kList = 1,
    kDelete = 2,
    kStat = 3,
    kDownload = 4,
};

enum class SystemMethod : std::uint32_t {
    kGetCapabilities = 1,
    kGetVersion = 2,
    /** Re-baseline encoder_last_position.bin from current joint readings. Request body empty.
     *  Requires Recovery state and pending encoder baseline confirmation (HTTP 409 otherwise). */
    kResetEncoderLastPosition = 3,
    /** Request body empty. Response: encoded serial string (may be empty). */
    kGetArmSerialNumber = 4,
    /** Request body: encoded serial string. Response empty on success. */
    kSetArmSerialNumber = 5,
    /** Request body empty. Persist current runtime settings to disk. */
    kFlushRuntimeConfig = 6,
};

enum class RuntimeSettingsMethod : std::uint32_t {
    kFlush = 1,
    kListProfiles = 2,
    kActivateProfile = 3,
    kCreateProfile = 4,
    kGetRuntimeConfigYaml = 5,
    /** Request: uint32 arm_index. Response: effective arm safety YAML. */
    kGetArmEffectiveYaml = 6,
    /** Request: uint32 arm_index + YAML safety overlay. Response empty on success. */
    kSetArmSafetyYaml = 7,
};

enum class LogMethod : std::uint32_t {
    kPoll = 1,
};

enum class ConfigMethod : std::uint32_t {
    kGet = 1,
    kSet = 2,
};

/**
 * Runtime rt_control parameters exposed via ServiceId::kConfig.
 * Each type documents its expected value count and semantics.
 */
enum class RtConfigType : std::uint32_t {
    /** values[0]: LogLevel 0=trace .. 5=off */
    kLogLevel = 1,
    /** values[0]: 0|1 */
    kLogToFile = 2,
    /** values[0]: 0|1 */
    kLogToStderr = 3,
    /** values[0]: 0|1 */
    kTelemetryEnabled = 4,
    /** values[0]: sample_every_n >= 1 */
    kTelemetrySampleEveryN = 5,
    /** values[0]: heartbeat_timeout_ms > 0 */
    kHeartbeatTimeoutMs = 6,
    /** values[0]: arm_failed_threshold >= 1 */
    kArmFailedThreshold = 7,
    /** values[0]: gripper_failed_threshold >= 1 */
    kGripperFailedThreshold = 8,
    /** values[0]: button_failed_threshold >= 1 */
    kButtonFailedThreshold = 9,
    /** values[0]: joint_follow_threshold >= 1 */
    kJointFollowThreshold = 10,
};

struct FileEntry {
    std::string name;
    std::uint64_t size = 0;
    std::uint64_t mtime_sec = 0;
};

struct StatResult {
    std::uint64_t size = 0;
    std::uint64_t mtime_sec = 0;
};

struct LogPollResult {
    std::uint64_t latest_id = 0;
    std::vector<std::string> lines;
};

struct ConfigValuePayload {
    RtConfigType type = RtConfigType::kLogLevel;
    std::vector<std::int32_t> values;
};

/** Network configuration snapshot returned by ServiceId::kNetwork RPCs. */
struct NetworkConfigData {
    std::uint32_t command_status = 0;
    std::uint8_t action = 0;
    std::uint8_t wifi_enable = 0;
    std::string wifi_ssid;
    std::string wifi_password;
    std::string eth_ip;
    std::uint8_t eth_prefix = 24;
    std::string eth_netmask;
    std::string eth_gateway;
    std::string eth_dns;
};

bool encodeSetWifiRequest(std::uint8_t wifi_enable, const std::string& ssid, const std::string& password,
                          std::vector<std::uint8_t>& out);
bool decodeSetWifiRequest(const std::uint8_t* data, std::size_t size, std::uint8_t& wifi_enable, std::string& ssid,
                          std::string& password);

bool encodeSetEthStaticRequest(const std::string& eth_ip, std::uint8_t eth_prefix, const std::string& eth_netmask,
                               const std::string& eth_gateway, const std::string& eth_dns,
                               std::vector<std::uint8_t>& out);
bool decodeSetEthStaticRequest(const std::uint8_t* data, std::size_t size, std::string& eth_ip, std::uint8_t& eth_prefix,
                               std::string& eth_netmask, std::string& eth_gateway, std::string& eth_dns);

bool encodeNetworkConfigData(const NetworkConfigData& data, std::vector<std::uint8_t>& out);
bool decodeNetworkConfigData(const std::uint8_t* data, std::size_t size, NetworkConfigData& out);

bool encodeNameRequest(const std::string& name, std::vector<std::uint8_t>& out);
bool decodeNameRequest(const std::uint8_t* data, std::size_t size, std::string& name);

bool encodePrefixRequest(const std::string& prefix, std::vector<std::uint8_t>& out);
bool decodePrefixRequest(const std::uint8_t* data, std::size_t size, std::string& prefix);

bool encodeFileList(const std::vector<FileEntry>& entries, std::vector<std::uint8_t>& out);
bool decodeFileList(const std::uint8_t* data, std::size_t size, std::vector<FileEntry>& entries);

bool encodeStatResult(const StatResult& stat, std::vector<std::uint8_t>& out);
bool decodeStatResult(const std::uint8_t* data, std::size_t size, StatResult& stat);

bool encodeDownloadMeta(std::uint64_t size, std::vector<std::uint8_t>& out);
bool decodeDownloadMeta(const std::uint8_t* data, std::size_t size, std::uint64_t& size_out);

bool encodeLogPollRequest(std::uint64_t after_id, std::vector<std::uint8_t>& out);
bool decodeLogPollRequest(const std::uint8_t* data, std::size_t size, std::uint64_t& after_id);

bool encodeLogPollResponse(std::uint64_t latest_id, const std::vector<std::string>& lines,
                           std::vector<std::uint8_t>& out);
bool decodeLogPollResponse(const std::uint8_t* data, std::size_t size, LogPollResult& out);

bool encodeConfigValuePayload(RtConfigType type, const std::vector<std::int32_t>& values,
                              std::vector<std::uint8_t>& out);
bool decodeConfigValuePayload(const std::uint8_t* data, std::size_t size, ConfigValuePayload& out);

bool encodeRpcSessionAuth(const RpcSessionAuth& auth, std::vector<std::uint8_t>& out);
bool decodeRpcSessionAuth(const std::uint8_t* data, std::size_t size, RpcSessionAuth& auth);

bool encodeRpcRequest(const RpcSessionAuth& auth, ServiceId service, std::uint32_t method,
                      const std::vector<std::uint8_t>& body, std::vector<std::uint8_t>& out);
bool decodeRpcRequest(const std::uint8_t* data, std::size_t size, RpcSessionAuth& auth, std::uint32_t& service_id,
                      std::uint32_t& method_id, const std::uint8_t*& body, std::size_t& body_size);

bool encodeBlobBeginPayload(const RpcSessionAuth& auth, const std::string& name, std::uint32_t total_size,
                            std::vector<std::uint8_t>& out);
bool decodeBlobBeginPayload(const std::uint8_t* data, std::size_t size, RpcSessionAuth& auth, std::string& name,
                            std::uint32_t& total_size);

bool encodeRpcResponse(std::uint32_t status_code, const std::vector<std::uint8_t>& body,
                         std::vector<std::uint8_t>& out);
bool decodeRpcResponse(const std::uint8_t* data, std::size_t size, std::uint32_t& status_code,
                       const std::uint8_t*& body, std::size_t& body_size);

}  // namespace robot::platform::tcp_data

#endif
