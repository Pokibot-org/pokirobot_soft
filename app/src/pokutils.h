#ifndef POKUTILS_H
#define POKUTILS_H

#include <zephyr/kernel.h>
#include <math.h>

// needed with newlib
// https://github.com/zephyrproject-rtos/zephyr/blob/main/lib/libc/minimal/include/math.h
#define MAXFLOAT 3.40282347e+38F

#if !(defined __USE_MISC || defined __USE_XOPEN)
#define M_E        2.7182818284590452354f
#define M_LOG2E    1.4426950408889634074f
#define M_LOG10E   0.43429448190325182765f
#define M_LN2      0.693147180559945309417f
#define M_LN10     2.30258509299404568402f
#define M_PI       3.14159265358979323846f
#define M_PI_2     1.57079632679489661923f
#define M_PI_4     0.78539816339744830962f
#define M_1_PI     0.31830988618379067154f
#define M_2_PI     0.63661977236758134308f
#define M_2_SQRTPI 1.12837916709551257390f
#define M_SQRT2    1.41421356237309504880f
#define M_SQRT1_2  0.70710678118654752440f
#define M_SQRT3    1.73205080756887729352f
#endif

#define RAD_TO_DEG(_rad) ((_rad) * 180.0f / M_PI)
#define DEG_TO_RAD(_deg) ((_deg) / 180.0f * M_PI)

#define SIGNF(_val)     (signbit(_val) ? -1.0f : 1.0f)
#define NEG_SQRTF(_val) (SIGNF(_val) * sqrtf(fabsf(_val)))

#define LOCKVAR(_type)                                                                             \
    struct {                                                                                       \
        struct k_mutex lock;                                                                       \
        _type val;                                                                                 \
    }

#define INIT_LOCKVAR(_var) k_mutex_init(&((_var).lock))

#define READ_LOCKVAR(_var, _dst, _err, _timeout)                                                   \
    (_err) = k_mutex_lock(&((_var).lock), (_timeout));                                             \
    if (!(_err)) {                                                                                 \
        (_dst) = (_var).val;                                                                       \
        k_mutex_unlock(&((_var).lock));                                                            \
    }

#define SET_LOCKVAR(_var, _src, _err, _timeout)                                                    \
    (_err) = k_mutex_lock(&((_var).lock), (_timeout));                                             \
    if (!(_err)) {                                                                                 \
        (_var).val = (_src);                                                                       \
        k_mutex_unlock(&((_var).lock));                                                            \
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
vec2_t vec2_normalize(vec2_t vec);
float vec2_abs(vec2_t vec);
float vec2_dot(vec2_t a, vec2_t b);
float vec2_angle(vec2_t a, vec2_t b);
float vec2_distance(point2_t a, point2_t b);

pos2_t pos2_diff(const pos2_t target, const pos2_t orig);
pos2_t pos2_add(const pos2_t a, const pos2_t b);
float angle_modulo(float a);
#endif // UTILS_H
