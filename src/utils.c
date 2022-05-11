#include "utils.h"

#include <math.h>
#include <zephyr.h>

#include <logging/log.h>

LOG_MODULE_REGISTER(utils);

vec2_t point2_diff(point2_t target, point2_t orig) {
    vec2_t delta = {
        .dx = target.x - orig.x,
        .dy = target.y - orig.y,
    };
    return delta;
}

float vec2_abs(vec2_t val) {
    return sqrtf(val.dx * val.dx + val.dy * val.dy);
}

float vec2_fast_abs(vec2_t val) {
    // https://www.flipcode.com/archives/Fast_Approximate_Distance_Functions.shtml
    // https://gamedev.stackexchange.com/questions/69241/how-to-optimize-the-distance-function
    return 0.5 * (val.dx + val.dy + Z_MAX(val.dx, val.dy));
}

pos2_t pos2_diff(pos2_t target, pos2_t orig) {
    pos2_t delta = {
        .x = target.x - orig.x,
        .y = target.y - orig.y,
        .a = target.a - orig.a,
    };
    return delta;
}
