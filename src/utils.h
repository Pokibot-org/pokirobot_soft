#ifndef UTILS_H
#define UTILS_H


#include <zephyr.h>


#define LOCKVAR(_type)                                                                                                 \
    struct {                                                                                                           \
        struct k_mutex lock;                                                                                           \
        _type val;                                                                                                     \
    }

#define INIT_LOCKVAR(_var) k_mutex_init(&((_var).lock))

#define READ_LOCKVAR(_var, _dst, _err, _timeout)                                                                       \
    (_err) = k_mutex_lock(&((_var).lock), (_timeout));                                                                 \
    if (!(_err)) {                                                                                                     \
        (_dst) = (_var).val;                                                                                           \
        k_mutex_unlock(&((_var).lock));                                                                                \
    }

#define SET_LOCKVAR(_var, _src, _err, _timeout)                                                                        \
    (_err) = k_mutex_lock(&((_var).lock), (_timeout));                                                                 \
    if (!(_err)) {                                                                                                     \
        (_var).val = (_src);                                                                                           \
        k_mutex_unlock(&((_var).lock));                                                                                \
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


vec2_t point2_diff(point2_t target, point2_t orig);
float vec2_abs(vec2_t val);
float vec2_fast_abs(vec2_t val);

pos2_t pos2_diff(pos2_t target, pos2_t orig);

#endif // UTILS_H
