#include "pokutils/robot_utils.h"

#include <math.h>
#include <stdlib.h>
#include <zephyr.h>

#include <random/rand32.h>

//// TODO: Octile distance h(x) = max( (x1 – x2), (y1 – y2) + (sqrt(2) -1) * min( (x1 – x2), (y1 – y2))
//
// inline float utils_distance(const coordinates_t *a, const coordinates_t *b)
//{
//    // TODO: only safe is output is at least 33 bit
//    return sqrtf(SQUARE((int32_t)a->x - b->x) + SQUARE((int32_t)a->y - b->y));
//}
//
// inline float utils_distance_squared(const coordinates_t *a, const coordinates_t *b)
//{
//    return SQUARE((int32_t)a->x - b->x) + SQUARE((int32_t)a->y - b->y);
//}
//
// inline float utils_distance_summed(const coordinates_t *a, const coordinates_t *b)
//{
//    return abs((int32_t)a->x - b->x) + abs((int32_t)a->y - b->y);
//}
//
// inline float utils_distance_aproximated(const coordinates_t *a, const coordinates_t *b)
//{
//    // https://www.flipcode.com/archives/Fast_Approximate_Distance_Functions.shtml
//    // https://gamedev.stackexchange.com/questions/69241/how-to-optimize-the-distance-function
//    uint32_t dx = abs(a->x - b->x);
//    uint32_t dy = abs(a->y - b->y);
//    return (dx + dy + MAX(dx, dy)) >> 1;
//}

inline uint32_t utils_get_rand32(void) {
    return sys_rand32_get();
}
