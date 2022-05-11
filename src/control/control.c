#include "control.h"

#include <math.h>
#include <zephyr.h>

#include "utils.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(control);


int control_init(control_t* dev) {
    int ret = 0;
    INIT_LOCKVAR(dev->pos);
    INIT_LOCKVAR(dev->target);
    INIT_LOCKVAR(dev->motors_v);
    return ret;
}


#define CONTROL_LOCKVAR_SETTER(_var, _type)                                                                            \
    int control_set_##_var(control_t* dev, _type _var) {                                                               \
        int err = 0;                                                                                                   \
        SET_LOCKVAR(dev->_var, _var, err, CONTROL_MUTEX_TIMEOUT);                                                      \
        if (err) {                                                                                                     \
            LOG_ERR("could not lock _var mutex for write access");                                                     \
            goto exit_error;                                                                                           \
        }                                                                                                              \
        return 0;                                                                                                      \
    exit_error:                                                                                                        \
        return -1;                                                                                                     \
    }

#define CONTROL_LOCKVAR_GETTER(_var, _type)                                                                            \
    int control_get_##_var(control_t* dev, _type* _var) {                                                              \
        int err = 0;                                                                                                   \
        SET_LOCKVAR(dev->_var, *_var, err, CONTROL_MUTEX_TIMEOUT);                                                     \
        if (err) {                                                                                                     \
            LOG_ERR("could not lock _var mutex for write access");                                                     \
            goto exit_error;                                                                                           \
        }                                                                                                              \
        return 0;                                                                                                      \
    exit_error:                                                                                                        \
        return -1;                                                                                                     \
    }

CONTROL_LOCKVAR_SETTER(pos, pos2_t)
CONTROL_LOCKVAR_SETTER(target, pos2_t)
CONTROL_LOCKVAR_SETTER(motors_v, omni3_t)

CONTROL_LOCKVAR_GETTER(pos, pos2_t)
CONTROL_LOCKVAR_GETTER(target, pos2_t)
CONTROL_LOCKVAR_GETTER(motors_v, omni3_t)


vel2_t world_vel2_from_delta(pos2_t delta, vel2_t prev_vel) {
    // TODO clamp + ramp
    vel2_t vel = {
        .vx = 1.0f * delta.x,
        .vy = 1.0f * delta.y,
        .w = 1.0f * delta.a,
    };
    return vel;
}

vel2_t local_vel2_from_world(pos2_t pos, vel2_t world_vel) {
    vel2_t local_vel = {
        .vx = cos(pos.a) * world_vel.vx - sin(pos.a) * world_vel.vy,
        .vy = -sin(pos.a) * world_vel.vx + cos(pos.a) * world_vel.vy,
        .w = world_vel.w,
    };
    return local_vel;
}

omni3_t omni3_from_vel2(vel2_t vel) {
    omni3_t omni3 = {
        .v1 = -vel.vx / 2 - sqrtf(3) * vel.vy / 2 + ROBOT_L * vel.w,
        .v2 = vel.vx / 2 + ROBOT_L * vel.w,
        .v3 = -vel.vx / 2 + sqrtf(3) * vel.vy / 2 + ROBOT_L * vel.w,
    };
    return omni3;
}
