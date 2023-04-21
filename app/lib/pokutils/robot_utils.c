#include "pokutils/robot_utils.h"

#include <math.h>
#include <zephyr/kernel.h>

#if defined(CONFIG_ARCH_POSIX)
#include <stdlib.h>
#else
#include <zephyr/random/rand32.h>
#endif

inline uint32_t utils_get_rand32(void)
{

#if defined(CONFIG_ARCH_POSIX)
	return rand();
#else
	return sys_rand32_get();
#endif
}

inline float utils_get_randf(void)
{
	return (float)utils_get_rand32() / UINT32_MAX;
}

inline float utils_get_randf_in_range(float min, float max)
{
	return utils_get_randf() * (max - min) + min;
}
