#include "util.h"

void wait(int usec)
{
    volatile int64_t time_start = g_timestamp;

    while(g_timestamp < time_start + usec)
    {
        //do nothing
    }
}

