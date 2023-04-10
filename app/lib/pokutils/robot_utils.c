#include "pokutils/robot_utils.h"

#include <math.h>
#include <stdlib.h>
#include <zephyr.h>

#include <random/rand32.h>

inline uint32_t utils_get_rand32(void) {
    return sys_rand32_get();
}
