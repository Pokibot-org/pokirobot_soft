#ifndef PATHFINDING_UTILS_H
#define PATHFINDING_UTILS_H

#include "pokutils/common_types.h"

#define SQUARE(a) ((a) * (a))
#define ABS(a) ((a) < 0 ? -(a) : (a))

uint32_t utils_get_rand32();

#endif
