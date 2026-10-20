#ifndef TIME_UTILS_H
#define TIME_UTILS_H

#include <cstdint>

namespace robot::platform {
    int64_t get_time_now();

    void sleep_period(int sleep_time_in_us);
} // namespace robot::platform
#endif
