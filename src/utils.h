#ifndef UTILS_H
#define UTILS_H


#include <zephyr.h>


#define LOCKVAR(_type) \
    struct {                    \
        struct k_mutex lock;    \
        _type val;              \
    }

#define INIT_LOCKVAR(_var) \
    k_mutex_init(&((_var).lock))

#define READ_LOCKVAR(_var, _dst, _err, _timeout) \
    (_err) = k_mutex_lock(&((_var).lock), (_timeout));  \
    if (!(_err)) {                                      \
        (_dst) = (_var).val;                            \
        k_mutex_unlock(&((_var).lock));                 \
    }

#define SET_LOCKVAR(_var, _src, _err, _timeout) \
    (_err) = k_mutex_lock(&((_var).lock), (_timeout));  \
    if (!(_err)) {                                      \
        (_var).val = (_src);                            \
        k_mutex_unlock(&((_var).lock));                 \
    }

#endif // UTILS_H
