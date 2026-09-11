#ifndef TCP_DATA_PAYLOADS_H
#define TCP_DATA_PAYLOADS_H

#include "tcp_data_service_ids.h"

#include <cstdint>
#include <string>
#include <vector>

namespace robot::platform::tcp_data {

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

/** One registered RPC service for GetCapabilities encoding. */
struct RpcServiceCapability {
    std::uint32_t service_id = 0;
    std::vector<std::uint32_t> method_ids;
};

}  // namespace robot::platform::tcp_data

#endif
