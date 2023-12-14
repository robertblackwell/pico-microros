#include <stdint.h>
#include <sys/types.h> // added by robert blackwell for clockid_t
#include <time.h> // added by robert blackwell
#include <sys/time.h>
#include <pico/error.h> // added by robert blackwell
#include "pico/stdlib.h"

void usleep(uint64_t us)
{
    sleep_us(us);
}

int clock_gettime(clockid_t unused, struct timespec *tp)
{
    uint64_t m = time_us_64();
    tp->tv_sec = m / 1000000;
    tp->tv_nsec = (m % 1000000) * 1000;
    return 0;
}
