#include <time_utils.h>

#include <time.h>
#include <sys/time.h>

void sleep_period(int sleep_time_in_us)
{
    if (sleep_time_in_us < 0)
        return;
    struct timespec ts;
    ts.tv_sec = sleep_time_in_us / 1000000L;
    ts.tv_nsec = (sleep_time_in_us % 1000000L) * 1000L;
    clock_nanosleep(CLOCK_REALTIME, 0, &ts, NULL);
}

long get_time_now(){
    #ifdef KERNEL_XENOMAI
        RTIME now = rt_timer_read();
        return now / 1000;
    #else // preempt_rt & linux
        struct timeval t;
        gettimeofday(&t, NULL);
        return t.tv_sec * 1000000 + t.tv_usec;
    #endif
}
