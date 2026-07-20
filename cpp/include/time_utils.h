#ifndef TIME_UTILS_H
#define TIME_UTILS_H

namespace robot::platform {
    long get_time_now();

    void sleep_period(int sleep_time_in_us);
} // namespace robot::platform
#endif