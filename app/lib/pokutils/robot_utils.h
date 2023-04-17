#ifndef PATHFINDING_UTILS_H
#define PATHFINDING_UTILS_H
#include "stdint.h"

#define SQUARE(a) ((a) * (a))
uint32_t utils_get_rand32();
float utils_get_randf(void);
float utils_get_randf_in_range(float min, float max);
#endif
