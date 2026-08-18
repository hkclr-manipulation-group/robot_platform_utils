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

}  // namespace robot::platform

#endif
