#ifndef UTILS_H
#define UTILS_H


#include <zephyr.h>


// needed with newlib
// https://github.com/zephyrproject-rtos/zephyr/blob/main/lib/libc/minimal/include/math.h
#define MAXFLOAT 3.40282347e+38F

#define M_E 2.7182818284590452354
#define M_LOG2E 1.4426950408889634074
#define M_LOG10E 0.43429448190325182765
#define M_LN2 0.693147180559945309417
#define M_LN10 2.30258509299404568402
#define M_PI 3.14159265358979323846
#define M_PI_2 1.57079632679489661923
#define M_PI_4 0.78539816339744830962
#define M_1_PI 0.31830988618379067154
#define M_2_PI 0.63661977236758134308
#define M_2_SQRTPI 1.12837916709551257390
#define M_SQRT2 1.41421356237309504880
#define M_SQRT1_2 0.70710678118654752440
#define M_SQRT3 1.73205080756887729352

#define SIDE_YELLOW 1
#define SIDE_PURPLE 0

#define LOCKVAR(_type)                                                         \
    struct {                                                                   \
        struct k_mutex lock;                                                   \
        _type val;                                                             \
    }

#define INIT_LOCKVAR(_var) k_mutex_init(&((_var).lock))

#define READ_LOCKVAR(_var, _dst, _err, _timeout)                               \
    (_err) = k_mutex_lock(&((_var).lock), (_timeout));                         \
    if (!(_err)) {                                                             \
        (_dst) = (_var).val;                                                   \
        k_mutex_unlock(&((_var).lock));                                        \
    }

#define SET_LOCKVAR(_var, _src, _err, _timeout)                                \
    (_err) = k_mutex_lock(&((_var).lock), (_timeout));                         \
    if (!(_err)) {                                                             \
        (_var).val = (_src);                                                   \
        k_mutex_unlock(&((_var).lock));                                        \
    }


typedef struct point2 {
    float x;
    float y;
} point2_t;

typedef struct vec2 {
    float dx;
    float dy;
} vec2_t;


typedef struct pos2 {
    float x;
    float y;
    float a;
} pos2_t;

typedef struct vel2 {
    float vx;
    float vy;
    float w;
} vel2_t;


vec2_t point2_diff(const point2_t terminal, const point2_t initial);
float vec2_abs(vec2_t vec);
float vec2_fast_abs(vec2_t vec);

pos2_t pos2_diff(const pos2_t target, const pos2_t orig);

#endif // UTILS_H
