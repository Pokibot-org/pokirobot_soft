#include "pokutils/robot_utils.h"

#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/random/rand32.h>

inline uint32_t utils_get_rand32(void)
{
    return sys_rand32_get();
}

inline float utils_get_randf(void)
{
    return (float)utils_get_rand32() / UINT32_MAX;
}

inline float utils_get_randf_in_range(float min, float max)
{
    return utils_get_randf() * (max - min) + min;
}
