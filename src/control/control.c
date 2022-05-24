#include "control.h"

#include <kernel.h>
#include <math.h>
#include <zephyr.h>

#include "shared.h"
#include "tmc2209/tmc2209.h"
#include "utils.h"
#include <logging/log.h>
#include <sys/util.h>

LOG_MODULE_REGISTER(control);


tmc2209_t train_motor_1;
tmc2209_t train_motor_2;
tmc2209_t train_motor_3;
control_t shared_ctrl;


#define CONTROL_LOCKVAR_SETTER(_var, _type)                                    \
    int control_set_##_var(control_t* dev, _type _var) {                       \
        int err = 0;                                                           \
        SET_LOCKVAR(dev->_var, _var, err, CONTROL_MUTEX_TIMEOUT);              \
        if (err) {                                                             \
            LOG_ERR("could not lock _var mutex for write access");             \
            goto exit_error;                                                   \
        }                                                                      \
        return 0;                                                              \
    exit_error:                                                                \
        return -1;                                                             \
    }

#define CONTROL_LOCKVAR_GETTER(_var, _type)                                    \
    int control_get_##_var(control_t* dev, _type* _var) {                      \
        int err = 0;                                                           \
        SET_LOCKVAR(dev->_var, *_var, err, CONTROL_MUTEX_TIMEOUT);             \
        if (err) {                                                             \
            LOG_ERR("could not lock _var mutex for write access");             \
            goto exit_error;                                                   \
        }                                                                      \
        return 0;                                                              \
    exit_error:                                                                \
        return -1;                                                             \
    }

CONTROL_LOCKVAR_SETTER(pos, pos2_t)
CONTROL_LOCKVAR_SETTER(target, pos2_t)

CONTROL_LOCKVAR_GETTER(pos, pos2_t)
CONTROL_LOCKVAR_GETTER(target, pos2_t)


int control_init(control_t* ctrl, tmc2209_t* m1, tmc2209_t* m2, tmc2209_t* m3) {
    int ret = 0;
    ctrl->start = false;
    ctrl->brake = false;
    INIT_LOCKVAR(ctrl->pos);
    INIT_LOCKVAR(ctrl->target);
    control_set_pos(ctrl, (pos2_t){0, 0, 0});
    control_set_target(ctrl, (pos2_t){0, 0, 0});
    ctrl->m1 = m1;
    ctrl->m2 = m2;
    ctrl->m3 = m3;
    if (tmc2209_init(ctrl->m1, &steppers_uart_hdb, 0)) {
        LOG_ERR("failed to init control motor 1");
        ret = -1;
    }
    if (tmc2209_init(ctrl->m2, &steppers_uart_hdb, 1)) {
        LOG_ERR("failed to init control motor 2");
        ret = -1;
    }
    if (tmc2209_init(ctrl->m3, &steppers_uart_hdb, 2)) {
        LOG_ERR("failed to init control motor 3");
        ret = -1;
    }
    return ret;
}

vel2_t world_vel_from_delta(pos2_t delta, vel2_t prev_vel) {
    // planar speed capping + acceleration ramp
    float vx = PLANAR_FACTOR * delta.x;
    float vy = PLANAR_FACTOR * delta.y;
    const float planar_speed = sqrtf(vx * vx + vy * vy);
    const float planar_speed_prev =
        sqrtf(prev_vel.vx * prev_vel.vx + prev_vel.vy * prev_vel.vy);
    const float planar_speed_clamped =
        MIN(planar_speed, MIN(planar_speed_prev + PLANAR_RAMP, PLANAR_VMAX));
    if (planar_speed > planar_speed_clamped) {
        const float planar_factor = planar_speed_clamped / planar_speed;
        vx *= planar_factor;
        vy *= planar_factor;
    }
    // angular speed capping + acceleration ramp
    const float angular_speed_ramped = fabsf(prev_vel.w) + ANGULAR_RAMP;
    float w = Z_CLAMP(ANGULAR_FACTOR * delta.a,
        MAX(-angular_speed_ramped, -ANGULAR_VMAX),
        MIN(angular_speed_ramped, ANGULAR_VMAX));
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


static int control_task(void) {
    LOG_INF("control task init");
    int ret = 0;
    pos2_t pos = {0};
    vel2_t world_vel = {0};
    vel2_t local_vel = {0};
    omni3_t motors_v = {0};
    if (control_init(
            &shared_ctrl, &train_motor_1, &train_motor_2, &train_motor_3)) {
        LOG_ERR("failed to init control object");
        ret = -1;
    }
    LOG_INF("control task wait");
    while (!shared_ctrl.start) {
        k_sleep(K_MSEC((uint64_t)CONTROL_PERIOD_MS));
    }
    LOG_INF("control task start");
    while (1) {
        pos2_t target;
        // update pos
        // control_get_motors_v(&shared_ctrl, &motors_v);
        // local_vel = local_vel_from_omni(motors_v);
        // world_vel = world_vel_from_local(old_pos, local_vel);
        pos = (pos2_t){
            .x = pos.x + world_vel.vx * CONTROL_PERIOD_MS / 1000.0f,
            .y = pos.y + world_vel.vy * CONTROL_PERIOD_MS / 1000.0f,
            .a = pos.a + world_vel.w * CONTROL_PERIOD_MS / 1000.0f,
        };
        // update speed
        if (shared_ctrl.brake) {
            world_vel = (vel2_t){0};
            local_vel = (vel2_t){0};
            motors_v = (omni3_t){0};
        } else {
            control_get_target(&shared_ctrl, &target);
            world_vel = world_vel_from_delta(pos2_diff(target, pos), world_vel);
            local_vel = local_vel_from_world(pos, world_vel);
            motors_v = omni_from_local_vel(local_vel);
        }
        // commit transaction
        control_set_pos(&shared_ctrl, pos);
        tmc2209_set_speed(shared_ctrl.m1,
            (int32_t)(motors_v.v1 * (float)256 / WHEEL_PERIMETER)); // FIXME
        tmc2209_set_speed(shared_ctrl.m2,
            (int32_t)(motors_v.v2 * (float)256 / WHEEL_PERIMETER)); // FIXME
        tmc2209_set_speed(shared_ctrl.m3,
            (int32_t)(motors_v.v3 * (float)256 / WHEEL_PERIMETER)); // FIXME
        // sleep
        k_sleep(K_MSEC((uint64_t)CONTROL_PERIOD_MS));
    }
    LOG_INF("control task done (ret=%d)", ret);
    return ret;
}

#if CONFIG_CONTROL_TASK
K_THREAD_DEFINE(control_task_name, CONFIG_CONTROL_THREAD_STACK, control_task,
    NULL, NULL, NULL, CONFIG_CONTROL_THREAD_PRIORITY, 0, 0);
#endif
