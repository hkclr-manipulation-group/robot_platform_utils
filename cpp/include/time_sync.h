#ifndef TIME_SYNC_H
#define TIME_SYNC_H

#include <cstdint>

namespace robot::platform {

/** Four timestamps from a UDP SdkHandshake round trip (all microseconds, wall clock). */
struct HandshakeTimestamps {
    int64_t client_send_us = 0;
    int64_t server_recv_us = 0;
    int64_t server_send_us = 0;
    int64_t client_recv_us = 0;
};

struct TimeSyncResult {
    /** robot_timestamp_us = local_timestamp_us + offset_us */
    int64_t offset_us = 0;
    int64_t rtt_us = 0;
};

TimeSyncResult computeTimeSyncFromHandshake(const HandshakeTimestamps& timestamps);

inline int64_t robotTimestampToLocalUs(int64_t offset_us, int64_t robot_timestamp_us) {
    return robot_timestamp_us - offset_us;
}

inline int64_t localTimestampToRobotUs(int64_t offset_us, int64_t local_timestamp_us) {
    return local_timestamp_us + offset_us;
}

/** True when offset looks like a valid clock skew (not an epoch-sized bogus value). */
inline bool isPlausibleTimeSyncOffset(int64_t offset_us) {
    constexpr int64_t kMaxAbsOffsetUs = 86'400'000'000LL;  // 1 day
    return offset_us >= -kMaxAbsOffsetUs && offset_us <= kMaxAbsOffsetUs;
}

inline int64_t sanitizeTimeSyncOffset(int64_t offset_us) {
    return isPlausibleTimeSyncOffset(offset_us) ? offset_us : 0;
}

}  // namespace robot::platform

#endif
