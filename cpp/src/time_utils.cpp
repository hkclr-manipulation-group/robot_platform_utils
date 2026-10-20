#include <time_utils.h>
#include <time.h>

#if defined(_WIN32) || defined(WIN32)
    #include <chrono>
    #include <thread>
#else
    #include <sys/time.h>
#endif

namespace robot::platform {
    void sleep_period(int sleep_time_in_us)
    {
        if (sleep_time_in_us < 0)
            return;
    #if defined(_WIN32) || defined(WIN32)
        std::this_thread::sleep_for(std::chrono::microseconds(sleep_time_in_us));
    #else
        struct timespec ts;
        ts.tv_sec = sleep_time_in_us / 1000000L;
        ts.tv_nsec = (sleep_time_in_us % 1000000L) * 1000L;
        clock_nanosleep(CLOCK_REALTIME, 0, &ts, NULL);
    #endif
    }

    int64_t get_time_now(){
        #ifdef KERNEL_XENOMAI
            RTIME now = rt_timer_read();
            return now / 1000;
        #elif defined(_WIN32) || defined(WIN32)
            return std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count();
        #else // preempt_rt, linux
            struct timeval t;
            gettimeofday(&t, NULL);
            return static_cast<int64_t>(t.tv_sec) * 1000000 + t.tv_usec;
        #endif
    }
}
