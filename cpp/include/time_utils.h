#ifndef TIME_UTILS_H
#define TIME_UTILS_H

#include <cstdint>
#include <chrono>

namespace robot::platform {
    inline int64_t get_monotonic_time_us() {
        return std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    int64_t get_time_now();

    void sleep_period(int sleep_time_in_us);
} // namespace robot::platform
#endif
