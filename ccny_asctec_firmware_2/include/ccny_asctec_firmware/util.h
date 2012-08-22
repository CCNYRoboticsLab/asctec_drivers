#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>
#include "LPC214x.h"

extern volatile int64_t g_timestamp;

void wait(int usec);

#endif // UTIL_H
