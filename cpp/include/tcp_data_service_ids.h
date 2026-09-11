#ifndef TCP_DATA_SERVICE_IDS_H
#define TCP_DATA_SERVICE_IDS_H

#include <cstdint>

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
    /** Reserved; not registered on rt_control server. */
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
    /** Re-baseline encoder_last_position.bin from current joint readings. Request body empty. */
    kResetEncoderLastPosition = 3,
    /** Request body empty. Response: encoded serial string (may be empty). */
    kGetArmSerialNumber = 4,
    /** Request body: encoded serial string. Response empty on success. */
    kSetArmSerialNumber = 5,
    /** Request body empty. Persist current runtime settings to disk (same as RuntimeSettings::Flush). */
    kFlushRuntimeConfig = 6,
};

enum class RuntimeSettingsMethod : std::uint32_t {
    kFlush = 1,
    kListProfiles = 2,
    kActivateProfile = 3,
    kCreateProfile = 4,
    kGetRuntimeConfigJson = 5,
    /** Request: uint32 arm_index. Response: effective arm settings JSON. */
    kGetArmEffectiveJson = 6,
    /** Request: uint32 arm_index + JSON safety overlay. Response empty on success. */
    kSetArmSafetyJson = 7,
};

enum class LogMethod : std::uint32_t {
    kPoll = 1,
};

enum class ConfigMethod : std::uint32_t {
    kGet = 1,
    kSet = 2,
};

}  // namespace robot::platform::tcp_data

#endif
