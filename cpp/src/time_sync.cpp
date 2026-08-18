#include "time_sync.h"

namespace robot::platform {

TimeSyncResult computeTimeSyncFromHandshake(const HandshakeTimestamps& timestamps) {
    TimeSyncResult result{};
    const int64_t t1 = timestamps.client_send_us;
    const int64_t t2 = timestamps.server_recv_us;
    const int64_t t3 = timestamps.server_send_us;
    const int64_t t4 = timestamps.client_recv_us;

    result.offset_us = ((t2 - t1) + (t3 - t4)) / 2;
    result.rtt_us = (t4 - t1) - (t3 - t2);
    if (result.rtt_us < 0) {
        result.rtt_us = 0;
    }
    return result;
}

}  // namespace robot::platform
