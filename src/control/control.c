#include "control.h"

#include <math.h>
#include <zephyr.h>

#include "utils.h"
#include <logging/log.h>
#include <sys/util.h>

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


vel2_t world_vel_from_delta(pos2_t delta, vel2_t prev_vel) {
    // planar speed capping + acceleration ramp
    float vx = PLANAR_FACTOR * delta.x;
    float vy = PLANAR_FACTOR * delta.y;
    const float planar_speed = sqrtf(vx * vx + vy * vy);
    const float planar_speed_prev = sqrtf(prev_vel.vx * prev_vel.vx + prev_vel.vy * prev_vel.vy);
    const float planar_speed_clamped = MIN(planar_speed, MIN(planar_speed_prev + PLANAR_RAMP, PLANAR_VMAX));
    if (planar_speed > planar_speed_clamped) {
        const float planar_factor = planar_speed_clamped / planar_speed;
        vx *= planar_factor;
        vy *= planar_factor;
    }
    // angular speed capping + acceleration ramp
    const float angular_speed_ramped = fabsf(prev_vel.w) + ANGULAR_RAMP;
    float w = Z_CLAMP(
        ANGULAR_FACTOR * delta.a, MAX(-angular_speed_ramped, -ANGULAR_VMAX), MIN(angular_speed_ramped, ANGULAR_VMAX));
    // returning built vel
    vel2_t world_vel = {
        .vx = vx,
        .vy = vy,
        .w = w,
    };
    return world_vel;
}

vel2_t world_vel_from_local(pos2_t pos, vel2_t local_vel) {
    vel2_t world_vel = {
        .vx = cosf(pos.a) * local_vel.vx - sinf(pos.a) * local_vel.vy,
        .vy = sinf(pos.a) * local_vel.vx + cosf(pos.a) * local_vel.vy,
        .w = local_vel.w,
    };
    return world_vel;
}

vel2_t local_vel_from_world(pos2_t pos, vel2_t world_vel) {
    vel2_t local_vel = {
        .vx = cosf(pos.a) * world_vel.vx + sinf(pos.a) * world_vel.vy,
        .vy = -sinf(pos.a) * world_vel.vx + cosf(pos.a) * world_vel.vy,
        .w = world_vel.w,
    };
    return local_vel;
}

vel2_t local_vel_from_omni(omni3_t omni) {
    vel2_t local_vel = {
        .vx = (2.0f * omni.v2 - omni.v1 - omni.v3) / 3.0f,
        .vy = M_SQRT3 * (omni.v3 - omni.v1) / 3.0f,
        .w = omni.v1 + omni.v2 + omni.v3 / (3.0f * ROBOT_L),
    };
    return local_vel;
}

omni3_t omni_from_local_vel(vel2_t local_vel) {
    const float w_factor = ROBOT_L * local_vel.w;
    omni3_t omni3 = {
        .v1 = (-local_vel.vx - M_SQRT3 * local_vel.vy) / 2.0 + w_factor,
        .v2 = local_vel.vx + w_factor,
        .v3 = (-local_vel.vx + M_SQRT3 * local_vel.vy) / 2.0 + w_factor,
    };
    return omni3;
}
