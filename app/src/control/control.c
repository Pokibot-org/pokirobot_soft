#include "control.h"

#include <zephyr/kernel.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

#include "shared.h"
#include "tmc2209/tmc2209.h"
#include "pokutils.h"
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(control);

tmc2209_t train_motor_1;
tmc2209_t train_motor_2;
tmc2209_t train_motor_3;
control_t shared_ctrl;

#define CONTROL_LOCKVAR_SETTER(_var, _type)                                                        \
    int control_set_##_var(control_t *dev, _type _var)                                             \
    {                                                                                              \
        int err = 0;                                                                               \
        SET_LOCKVAR(dev->_var, _var, err, CONTROL_MUTEX_TIMEOUT);                                  \
        if (err) {                                                                                 \
            LOG_ERR("could not lock _var mutex for write access");                                 \
            goto exit_error;                                                                       \
        }                                                                                          \
        return 0;                                                                                  \
    exit_error:                                                                                    \
        return -1;                                                                                 \
    }

#define CONTROL_LOCKVAR_GETTER(_var, _type)                                                        \
    int control_get_##_var(control_t *dev, _type *_var)                                            \
    {                                                                                              \
        int err = 0;                                                                               \
        READ_LOCKVAR(dev->_var, *_var, err, CONTROL_MUTEX_TIMEOUT);                                \
        if (err) {                                                                                 \
            LOG_ERR("could not lock _var mutex for write access");                                 \
            goto exit_error;                                                                       \
        }                                                                                          \
        return 0;                                                                                  \
    exit_error:                                                                                    \
        return -1;                                                                                 \
    }

CONTROL_LOCKVAR_SETTER(pos, pos2_t)
CONTROL_LOCKVAR_GETTER(pos, pos2_t)

int control_set_brake(control_t *dev, bool brake)
{
    dev->brake = brake;
    return 0;
}

int control_set_waypoints(control_t *dev, pos2_t *src, int n)
{
    int err = k_mutex_lock(&dev->waypoints.lock, CONTROL_MUTEX_TIMEOUT);
    if (!err) {
        dev->waypoints.idx = 0;
        dev->waypoints.n = n;
        memcpy(dev->waypoints.wps, src, n * sizeof(pos2_t));
        k_mutex_unlock(&dev->waypoints.lock);
    } else {
        LOG_ERR("could not lock waypoints mutex for write access");
        goto exit_error;
    }
    return 0;
exit_error:
    return -1;
}

int control_init(control_t *ctrl, tmc2209_t *m1, tmc2209_t *m2, tmc2209_t *m3)
{
    k_sleep(K_MSEC(200));
    int ret = 0;
    ctrl->start = false;
    ctrl->brake = false;
    ctrl->at_target = false;
    ctrl->planar_target_sensivity = CONTROL_PLANAR_TARGET_SENSITIVITY_DEFAULT;
    ctrl->angular_target_sensivity = CONTROL_ANGULAR_TARGET_SENSITIVITY_DEFAULT;
    INIT_LOCKVAR(ctrl->pos);
    k_mutex_init(&ctrl->waypoints.lock);
    control_set_pos(ctrl, (pos2_t){0.0f, 0.0f, 0.0f});
    shared_ctrl.waypoints.wps = k_malloc(CONTROL_WAYPOINTS_N * sizeof(pos2_t));
    pos2_t target = (pos2_t){0.0f, 0.0f, 0.0f};
    control_set_waypoints(ctrl, &target, 1);
    ctrl->m1 = m1;
    ctrl->m2 = m2;
    ctrl->m3 = m3;
    for (int i = 0; i < 10; i++) {
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
        k_sleep(K_MSEC(10));
    }
    ctrl->ready = true;
    return ret;
}

void control_force_motor_stop(void)
{
    shared_ctrl.ready = false;
    for (int i = 0; i < 10; i++) {
        tmc2209_set_speed(&train_motor_1, 0);
        tmc2209_set_speed(&train_motor_2, 0);
        tmc2209_set_speed(&train_motor_3, 0);
    }
    // delay used to be sure that tmc task send the messages
    // to be changed !
    k_sleep(K_MSEC(1000));
}

vel2_t world_vel_from_delta(pos2_t delta, vel2_t prev_vel)
{
    // planar speed capping + acceleration ramp
    float vx = PLANAR_FACTOR * NEG_SQRTF(delta.x);
    float vy = PLANAR_FACTOR * NEG_SQRTF(delta.y);
    const float planar_speed = sqrtf(vx * vx + vy * vy);
    const float planar_speed_prev = sqrtf(prev_vel.vx * prev_vel.vx + prev_vel.vy * prev_vel.vy);
    const float planar_speed_clamped =
        MIN(planar_speed, MIN(planar_speed_prev + PLANAR_RAMP, PLANAR_VMAX));
    if (planar_speed > planar_speed_clamped) {
        const float planar_factor = planar_speed_clamped / planar_speed;
        vx *= planar_factor;
        vy *= planar_factor;
    }
    // angular speed capping + acceleration ramp
    const float angular_speed_ramped = fabsf(prev_vel.w) + ANGULAR_RAMP;
    float w =
        Z_CLAMP(ANGULAR_FACTOR * NEG_SQRTF(delta.a), MAX(-angular_speed_ramped, -ANGULAR_VMAX),
                MIN(angular_speed_ramped, ANGULAR_VMAX));
    // returning built vel
    vel2_t world_vel = {
        .vx = vx,
        .vy = vy,
        .w = w,
    };
    return world_vel;
}

vel2_t world_vel_from_delta2(pos2_t delta1, pos2_t delta2, vel2_t prev_vel)
{
    // compute biases
    float dist1 = sqrtf(delta1.x * delta1.x + delta1.y * delta1.y);
    float bias1 = MIN(dist1 / WP_DIST_BIAS, 1.0f);
    float bias2 = 1.0f - bias1;
    // planar speed capping + acceleration ramp
    float vx = PLANAR_FACTOR * (bias1 * NEG_SQRTF(delta1.x) + bias2 * NEG_SQRTF(delta2.x));
    float vy = PLANAR_FACTOR * (bias1 * NEG_SQRTF(delta1.y) + bias2 * NEG_SQRTF(delta2.y));
    const float planar_speed = sqrtf(vx * vx + vy * vy);
    const float planar_speed_prev = sqrtf(prev_vel.vx * prev_vel.vx + prev_vel.vy * prev_vel.vy);
    const float planar_speed_clamped =
        MIN(planar_speed, MIN(planar_speed_prev + PLANAR_RAMP, PLANAR_VMAX));
    if (planar_speed > planar_speed_clamped) {
        const float planar_factor = planar_speed_clamped / planar_speed;
        vx *= planar_factor;
        vy *= planar_factor;
    }
    // angular speed capping + acceleration ramp
    const float angular_speed_ramped = fabsf(prev_vel.w) + ANGULAR_RAMP;
    float w =
        Z_CLAMP(ANGULAR_FACTOR * NEG_SQRTF(delta1.a), MAX(-angular_speed_ramped, -ANGULAR_VMAX),
                MIN(angular_speed_ramped, ANGULAR_VMAX));
    // returning built vel
    vel2_t world_vel = {
        .vx = vx,
        .vy = vy,
        .w = w,
    };
    return world_vel;
}

vel2_t world_vel_from_local(pos2_t pos, vel2_t local_vel)
{
    vel2_t world_vel = {
        .vx = cosf(pos.a) * local_vel.vx - sinf(pos.a) * local_vel.vy,
        .vy = sinf(pos.a) * local_vel.vx + cosf(pos.a) * local_vel.vy,
        .w = -local_vel.w,
    };
    return world_vel;
}

vel2_t local_vel_from_world(pos2_t pos, vel2_t world_vel)
{
    vel2_t local_vel = {
        .vx = cosf(pos.a) * world_vel.vx + sinf(pos.a) * world_vel.vy,
        .vy = -sinf(pos.a) * world_vel.vx + cosf(-pos.a) * world_vel.vy,
        .w = -world_vel.w,
    };
    return local_vel;
}

vel2_t local_vel_from_omni(omni3_t omni)
{
    vel2_t local_vel = {
        .vx = (2.0f * omni.v2 - omni.v1 - omni.v3) / 3.0f,
        .vy = M_SQRT3 * (omni.v3 - omni.v1) / 3.0f,
        .w = omni.v1 + omni.v2 + omni.v3 / (3.0f * ROBOT_L),
    };
    return local_vel;
}

omni3_t omni_from_local_vel(vel2_t local_vel)
{
    const float w_factor = ROBOT_L * local_vel.w;
    omni3_t omni3 = {
        .v1 = (-local_vel.vx - M_SQRT3 * local_vel.vy) / 2.0 + w_factor,
        .v2 = local_vel.vx + w_factor,
        .v3 = (-local_vel.vx + M_SQRT3 * local_vel.vy) / 2.0 + w_factor,
    };
    return omni3;
}

void control_task_wait_ready()
{
    while (!shared_ctrl.ready) {
        k_sleep(K_MSEC(10));
    }
}

int control_task_wait_target(float planar_sensivity, float angular_sensivity, uint32_t timeout_target_ms, uint32_t timeout_brake_ms)
{
    uint32_t target_timeout_cnt = 0;
    uint32_t brake_timeout_cnt = 0;
    shared_ctrl.planar_target_sensivity = planar_sensivity;
    shared_ctrl.angular_target_sensivity = angular_sensivity;
    shared_ctrl.at_target = false;
    while (!shared_ctrl.at_target) {
        target_timeout_cnt += 1;
        brake_timeout_cnt = shared_ctrl.brake ? brake_timeout_cnt + 1 : 0;
        if (target_timeout_cnt > timeout_target_ms) {
            return CONTROL_WAIT_TIMEOUT_TARGET;
        }
        if (brake_timeout_cnt > timeout_brake_ms) {
            return CONTROL_WAIT_TIMEOUT_BRAKE;
        }
        k_sleep(K_MSEC(1));
    }
    return CONTROL_WAIT_OK;
}

static void control_task_wait_start()
{
    while (!shared_ctrl.start) {
        k_sleep(K_MSEC(10));
    }
}

static int control_task(void)
{
    LOG_INF("control task init");
    int ret = 0;
    pos2_t pos = {.x = 0.0f, .y = 0.0f, .a = 0.0f};
    vel2_t world_vel = {.vx = 0.0f, .vy = 0.0f, .w = 0.0f};
    vel2_t local_vel = {.vx = 0.0f, .vy = 0.0f, .w = 0.0f};
    omni3_t motors_v = {.v1 = 0.0f, .v2 = 0.0f, .v3 = 0.0f};
    float wp_dist = -1.0f;
    float dist_prev = -1.0f;
    while (!shared_ctrl.start_init) {
        k_sleep(K_MSEC((uint64_t)CONTROL_PERIOD_MS));
    }

    if (control_init(&shared_ctrl, &train_motor_1, &train_motor_2, &train_motor_3)) {
        LOG_ERR("failed to init control object");
        ret = -1;
    }
    LOG_INF("control task wait");
    control_task_wait_start();
    LOG_INF("control task start");
    while (shared_ctrl.ready) {
        int n, idx;
        pos2_t wp1, wp2;
        // lock the waypoint structure
        int err = k_mutex_lock(&shared_ctrl.waypoints.lock, CONTROL_MUTEX_TIMEOUT);
        if (err) {
            LOG_ERR("could not lock waypoints mutex for read access");
            goto continue_nocommit;
        }
        // get next waypoints
        n = shared_ctrl.waypoints.n;
        idx = shared_ctrl.waypoints.idx;
        wp1 = shared_ctrl.waypoints.wps[MIN(idx, n - 1)];
        wp2 = shared_ctrl.waypoints.wps[MIN(idx + 1, n - 1)];
        // update pos
        control_get_pos(&shared_ctrl, &pos);
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
            world_vel = (vel2_t){.vx = 0.0f, .vy = 0.0f, .w = 0.0f};
            local_vel = (vel2_t){.vx = 0.0f, .vy = 0.0f, .w = 0.0f};
            motors_v = (omni3_t){.v1 = 0.0f, .v2 = 0.0f, .v3 = 0.0f};
        } else {
            pos2_t delta1 = pos2_diff(wp1, pos);
            pos2_t delta2 = pos2_diff(wp2, pos);
            wp_dist = vec2_abs((vec2_t){delta1.x, delta1.y});
            if (idx >= (n - 1) && wp_dist < CONTROL_PLANAR_TARGET_SENSITIVITY_DEFAULT &&
                delta2.a < CONTROL_ANGULAR_TARGET_SENSITIVITY_DEFAULT) {
                shared_ctrl.at_target = true;
            } else {
                shared_ctrl.at_target = false;
            }
            // world_vel = world_vel_from_delta(delta1, world_vel);
            world_vel = world_vel_from_delta2(delta1, delta2, world_vel);
            local_vel = local_vel_from_world(pos, world_vel);
            motors_v = omni_from_local_vel(local_vel);
        }
        // update next waypoints
        LOG_DBG("dist: %.2f | prev: %.2f | delta: %e", wp_dist, dist_prev, wp_dist - dist_prev);
        if (dist_prev >= 0.0f && ((wp_dist > dist_prev && wp_dist <= WP_SENSITIVITY) || wp_dist < CONTROL_PLANAR_TARGET_SENSITIVITY_DEFAULT)
            // (wp_dist-dist_prev <= WP_DELTA_THRESHOLD && wp_dist <= WP_SENSITIVITY)
            // || wp_dist <= CONTROL_PLANAR_TARGET_SENSITIVITY_DEFAULT)
        ) {
            shared_ctrl.waypoints.idx = MIN(shared_ctrl.waypoints.idx+1, shared_ctrl.waypoints.n);
            dist_prev = -1.0f;
            LOG_INF("idx: %d", shared_ctrl.waypoints.idx);
        } else {
            dist_prev = wp_dist;
        }
        // release mutex and commit transaction
        k_mutex_unlock(&shared_ctrl.waypoints.lock);
        control_set_pos(&shared_ctrl, pos);
        tmc2209_set_speed(shared_ctrl.m1, (int32_t)(motors_v.v1 * MM_TO_USTEPS / WHEEL_PERIMETER));
        tmc2209_set_speed(shared_ctrl.m2, (int32_t)(motors_v.v2 * MM_TO_USTEPS / WHEEL_PERIMETER));
        tmc2209_set_speed(shared_ctrl.m3, (int32_t)(motors_v.v3 * MM_TO_USTEPS / WHEEL_PERIMETER));
    continue_nocommit:
        // sleep
        LOG_DBG("pos: %.2f %.2f %.2f", pos.x, pos.y, pos.a);
        LOG_DBG("target: %.2f %.2f %.2f", wp1.x, wp1.y, wp1.a);
        LOG_DBG("next: %.2f %.2f %.2f", wp2.x, wp2.y, wp2.a);
        LOG_DBG("speed: %.2f %.2f %.2f", motors_v.v1, motors_v.v2, motors_v.v3);
        k_sleep(K_MSEC((uint64_t)CONTROL_PERIOD_MS));
    }
    LOG_INF("control task done (ret=%d)", ret);
    return ret;
}

#if CONFIG_CONTROL_TASK
K_THREAD_DEFINE(control_task_name, CONFIG_CONTROL_THREAD_STACK, control_task, NULL, NULL, NULL,
                CONFIG_CONTROL_THREAD_PRIORITY, 0, 0);
#endif

void _test_gconf()
{
    uint32_t gconf;
    // control_init(&shared_ctrl, &train_motor_1, &train_motor_2, &train_motor_3);
    k_sleep(K_MSEC(1000));
    while (1) {
        tmc2209_get_gconf(&train_motor_1, &gconf);
        k_sleep(K_MSEC(8000));
    }
}

void _test_motor_cmd()
{
    static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
    // control_init(&shared_ctrl, &train_motor_1, &train_motor_2, &train_motor_3);
    k_sleep(K_MSEC(1000));
    while (1) {
        gpio_pin_toggle(led.port, led.pin);
        tmc2209_set_speed(&train_motor_1, 0);
        tmc2209_set_speed(&train_motor_2, 0);
        tmc2209_set_speed(&train_motor_3, 0);
        k_sleep(K_MSEC(1000));
        gpio_pin_toggle(led.port, led.pin);
        tmc2209_set_speed(&train_motor_1, 10000);
        tmc2209_set_speed(&train_motor_2, 20000);
        tmc2209_set_speed(&train_motor_3, 40000);
        k_sleep(K_MSEC(1000));
        gpio_pin_toggle(led.port, led.pin);
        tmc2209_set_speed(&train_motor_1, 0);
        tmc2209_set_speed(&train_motor_2, 0);
        tmc2209_set_speed(&train_motor_3, 0);
        k_sleep(K_MSEC(1000));
        gpio_pin_toggle(led.port, led.pin);
        tmc2209_set_speed(&train_motor_1, 10000);
        tmc2209_set_speed(&train_motor_2, 0);
        tmc2209_set_speed(&train_motor_3, 0);
        k_sleep(K_MSEC(1000));
        gpio_pin_toggle(led.port, led.pin);
        tmc2209_set_speed(&train_motor_1, 0);
        tmc2209_set_speed(&train_motor_2, 10000);
        tmc2209_set_speed(&train_motor_3, 0);
        k_sleep(K_MSEC(1000));
        gpio_pin_toggle(led.port, led.pin);
        tmc2209_set_speed(&train_motor_1, 0);
        tmc2209_set_speed(&train_motor_2, 0);
        tmc2209_set_speed(&train_motor_3, 10000);
        k_sleep(K_MSEC(1000));
    }
}

void _test_target()
{
    LOG_INF("_test_target");
    static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#if !(CONFIG_CONTROL_TASK)
    LOG_ERR("control task not launched");
#endif
    while (1) {
        if (!shared_ctrl.ready) {
            k_sleep(K_MSEC(100));
            continue;
        }
        break;
    }
    shared_ctrl.start = true;
    k_sleep(K_MSEC(5000));
    pos2_t target;
    while (1) {
        gpio_pin_toggle(led.port, led.pin);
        target = (pos2_t){100.0f, 100.0f, 1.0f * M_PI};
        control_set_waypoints(&shared_ctrl, &target, 1);
        LOG_DBG("1");
        k_sleep(K_MSEC(5000));
        gpio_pin_toggle(led.port, led.pin);
        target = (pos2_t){0.0f, 0.0f, 0.0f * M_PI};
        control_set_waypoints(&shared_ctrl, &target, 1);
        LOG_DBG("2");
        k_sleep(K_MSEC(5000));
        gpio_pin_toggle(led.port, led.pin);
        target = (pos2_t){100.0f, -100.0f, -2.0f * M_PI};
        control_set_waypoints(&shared_ctrl, &target, 1);
        LOG_DBG("3");
        k_sleep(K_MSEC(5000));
    }
}

void _test_calibration_distance()
{
    LOG_INF("_test_calibration");
    // static const struct gpio_dt_spec led =
    //     GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
    // int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
#if !(CONFIG_CONTROL_TASK)
    LOG_ERR("control task not launched");
#endif
    shared_ctrl.start_init = true;
    while (1) {
        if (!shared_ctrl.ready) {
            k_sleep(K_MSEC(100));
            continue;
        }
        break;
    }
    LOG_DBG("alive");
    // gpio_pin_toggle(led.port, led.pin);
    pos2_t target = (pos2_t){0.0f, 0.0f, 0.0f};
    control_set_pos(&shared_ctrl, (pos2_t){0.0f, 0.0f, 0.0f});
    control_set_waypoints(&shared_ctrl, &target, 1);
    LOG_DBG("pos: %.2f %.2f %.2f", shared_ctrl.pos.val.x, shared_ctrl.pos.val.y,
            shared_ctrl.pos.val.a);
    LOG_DBG("target: %.2f %.2f %.2f", shared_ctrl.waypoints.wps[0].x,
            shared_ctrl.waypoints.wps[0].y, shared_ctrl.waypoints.wps[0].a);
    k_sleep(K_MSEC(1000));
    shared_ctrl.start = true;
    // gpio_pin_toggle(led.port, led.pin);
    target = (pos2_t){0.0f, 3000.0f, 0.0f * M_PI};
    control_set_waypoints(&shared_ctrl, &target, 1);
    LOG_DBG("pos: %.2f %.2f %.2f", shared_ctrl.pos.val.x, shared_ctrl.pos.val.y,
            shared_ctrl.pos.val.a);
    LOG_DBG("target: %.2f %.2f %.2f", shared_ctrl.waypoints.wps[0].x,
            shared_ctrl.waypoints.wps[0].y, shared_ctrl.waypoints.wps[0].a);
    k_sleep(K_MSEC(15000));
}

void _test_calibration_angle()
{
    LOG_INF("_test_calibration");
    // static const struct gpio_dt_spec led =
    //     GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
    // int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
#if !(CONFIG_CONTROL_TASK)
    LOG_ERR("control task not launched");
#endif
    shared_ctrl.start_init = true;
    while (1) {
        if (!shared_ctrl.ready) {
            k_sleep(K_MSEC(100));
            continue;
        }
        break;
    }
    LOG_DBG("alive");
    // gpio_pin_toggle(led.port, led.pin);
    pos2_t target = (pos2_t){0.0f, 0.0f, 0.0f};
    control_set_pos(&shared_ctrl, (pos2_t){0.0f, 0.0f, 0.0f});
    control_set_waypoints(&shared_ctrl, &target, 1);
    LOG_DBG("pos: %.2f %.2f %.2f", shared_ctrl.pos.val.x, shared_ctrl.pos.val.y,
            shared_ctrl.pos.val.a);
    LOG_DBG("target: %.2f %.2f %.2f", shared_ctrl.waypoints.wps[0].x,
            shared_ctrl.waypoints.wps[0].y, shared_ctrl.waypoints.wps[0].a);
    k_sleep(K_MSEC(1000));
    shared_ctrl.start = true;
    // gpio_pin_toggle(led.port, led.pin);
    target = (pos2_t){0.0f, 0.0f, 20.0f * M_PI};
    control_set_waypoints(&shared_ctrl, &target, 1);
    k_sleep(K_MSEC(15000));
    LOG_DBG("pos: %.2f %.2f %.2f", shared_ctrl.pos.val.x, shared_ctrl.pos.val.y,
            shared_ctrl.pos.val.a);
    LOG_DBG("target: %.2f %.2f %.2f", shared_ctrl.waypoints.wps[0].x,
            shared_ctrl.waypoints.wps[0].y, shared_ctrl.waypoints.wps[0].a);
}

void _test_calibration_mix()
{
    LOG_INF("_test_calibration");
    // static const struct gpio_dt_spec led =
    //     GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
    // int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
#if !(CONFIG_CONTROL_TASK)
    LOG_ERR("control task not launched");
#endif
    shared_ctrl.start_init = true;
    while (1) {
        if (!shared_ctrl.ready) {
            k_sleep(K_MSEC(100));
            continue;
        }
        break;
    }
    LOG_DBG("alive");
    // gpio_pin_toggle(led.port, led.pin);
    pos2_t target = (pos2_t){0.0f, 0.0f, 0.0f};
    control_set_pos(&shared_ctrl, (pos2_t){0.0f, 0.0f, 0.0f});
    control_set_waypoints(&shared_ctrl, &target, 1);
    LOG_DBG("pos: %.2f %.2f %.2f", shared_ctrl.pos.val.x, shared_ctrl.pos.val.y,
            shared_ctrl.pos.val.a);
    LOG_DBG("target: %.2f %.2f %.2f", shared_ctrl.waypoints.wps[0].x,
            shared_ctrl.waypoints.wps[0].y, shared_ctrl.waypoints.wps[0].a);
    k_sleep(K_MSEC(1000));
    shared_ctrl.start = true;
    target = (pos2_t){0.0f, 0.0f, 1.0f * M_PI};
    control_set_waypoints(&shared_ctrl, &target, 1);
    k_sleep(K_MSEC(5000));
    LOG_DBG("pos: %.2f %.2f %.2f", shared_ctrl.pos.val.x, shared_ctrl.pos.val.y,
            shared_ctrl.pos.val.a);
    LOG_DBG("target: %.2f %.2f %.2f", shared_ctrl.waypoints.wps[0].x,
            shared_ctrl.waypoints.wps[0].y, shared_ctrl.waypoints.wps[0].a);
    target = (pos2_t){0.0f, 1000.0f, -1.0f * M_PI};
    control_set_waypoints(&shared_ctrl, &target, 1);
    k_sleep(K_MSEC(5000));
    LOG_DBG("pos: %.2f %.2f %.2f", shared_ctrl.pos.val.x, shared_ctrl.pos.val.y,
            shared_ctrl.pos.val.a);
    LOG_DBG("target: %.2f %.2f %.2f", shared_ctrl.waypoints.wps[0].x,
            shared_ctrl.waypoints.wps[0].y, shared_ctrl.waypoints.wps[0].a);
}

void _test_connerie()
{
    LOG_INF("_test_connerie");
#if !(CONFIG_CONTROL_TASK)
    LOG_ERR("control task not launched");
#endif
    shared_ctrl.start_init = true;
    while (1) {
        if (!shared_ctrl.ready) {
            k_sleep(K_MSEC(100));
            continue;
        }
        break;
    }
    shared_ctrl.start = true;
    LOG_DBG("alive");
    // gpio_pin_toggle(led.port, led.pin);
    pos2_t target = (pos2_t){0.0f, 0.0f, 100.0f};
    control_set_pos(&shared_ctrl, (pos2_t){0.0f, 0.0f, 0.0f});
    control_set_waypoints(&shared_ctrl, &target, 1);
    k_sleep(K_MSEC(15000));
    shared_ctrl.brake = true;
}

void _test_drawing()
{
    LOG_INF("_test_drawing");
#if !(CONFIG_CONTROL_TASK)
    LOG_ERR("control task not launched");
#endif
    shared_ctrl.start_init = true;
    while (1) {
        if (!shared_ctrl.ready) {
            k_sleep(K_MSEC(100));
            continue;
        }
        break;
    }
    LOG_DBG("alive");
    // gpio_pin_toggle(led.port, led.pin);
    pos2_t target = (pos2_t){0.0f, 0.0f, 0.0f};
    control_set_pos(&shared_ctrl, (pos2_t){0.0f, 0.0f, 0.0f});
    control_set_waypoints(&shared_ctrl, &target, 1);
    k_sleep(K_MSEC(1000));
    shared_ctrl.start = true;

    // start of drawing
    pos2_t draw_wps[] = {
        (pos2_t){ .x = 46.0f,  .y = 202.0f, .a = 0.0f},
        (pos2_t){ .x = 44.0f,  .y = 202.0f, .a = 0.0f},
        (pos2_t){ .x = 40.0f,  .y = 199.0f, .a = 0.0f},
        (pos2_t){ .x = 36.0f,  .y = 195.0f, .a = 0.0f},
        (pos2_t){ .x = 34.0f,  .y = 190.0f, .a = 0.0f},
        (pos2_t){ .x = 34.0f,  .y = 190.0f, .a = 0.0f},
        (pos2_t){ .x = 34.0f,  .y = 168.0f, .a = 0.0f},
        (pos2_t){ .x = 26.0f,  .y = 161.0f, .a = 0.0f},
        (pos2_t){ .x = 19.0f,  .y = 154.0f, .a = 0.0f},
        (pos2_t){ .x = 13.0f,  .y = 146.0f, .a = 0.0f},
        (pos2_t){ .x = 8.0f,  .y = 138.0f, .a = 0.0f},
        (pos2_t){ .x = 5.0f,  .y = 129.0f, .a = 0.0f},
        (pos2_t){ .x = 2.0f,  .y = 119.0f, .a = 0.0f},
        (pos2_t){ .x = 0.0f,  .y = 109.0f, .a = 0.0f},
        (pos2_t){ .x = 0.0f,  .y = 98.0f, .a = 0.0f},
        (pos2_t){ .x = 0.0f,  .y = 98.0f, .a = 0.0f},
        (pos2_t){ .x = 0.0f,  .y = 98.0f, .a = 0.0f},
        (pos2_t){ .x = 0.0f,  .y = 89.0f, .a = 0.0f},
        (pos2_t){ .x = 1.0f,  .y = 80.0f, .a = 0.0f},
        (pos2_t){ .x = 3.0f,  .y = 71.0f, .a = 0.0f},
        (pos2_t){ .x = 6.0f,  .y = 61.0f, .a = 0.0f},
        (pos2_t){ .x = 12.0f,  .y = 52.0f, .a = 0.0f},
        (pos2_t){ .x = 18.0f,  .y = 43.0f, .a = 0.0f},
        (pos2_t){ .x = 27.0f,  .y = 34.0f, .a = 0.0f},
        (pos2_t){ .x = 38.0f,  .y = 24.0f, .a = 0.0f},
        (pos2_t){ .x = 38.0f,  .y = 12.0f, .a = 0.0f},
        (pos2_t){ .x = 38.0f,  .y = 12.0f, .a = 0.0f},
        (pos2_t){ .x = 40.0f,  .y = 7.0f, .a = 0.0f},
        (pos2_t){ .x = 42.0f,  .y = 3.0f, .a = 0.0f},
        (pos2_t){ .x = 47.0f,  .y = 0.0f, .a = 0.0f},
        (pos2_t){ .x = 47.0f,  .y = 0.0f, .a = 0.0f},
        (pos2_t){ .x = 47.0f,  .y = 0.0f, .a = 0.0f},
        (pos2_t){ .x = 51.0f,  .y = 1.0f, .a = 0.0f},
        (pos2_t){ .x = 53.0f,  .y = 2.0f, .a = 0.0f},
        (pos2_t){ .x = 55.0f,  .y = 4.0f, .a = 0.0f},
        (pos2_t){ .x = 56.0f,  .y = 6.0f, .a = 0.0f},
        (pos2_t){ .x = 57.0f,  .y = 10.0f, .a = 0.0f},
        (pos2_t){ .x = 58.0f,  .y = 14.0f, .a = 0.0f},
        (pos2_t){ .x = 63.0f,  .y = 12.0f, .a = 0.0f},
        (pos2_t){ .x = 70.0f,  .y = 10.0f, .a = 0.0f},
        (pos2_t){ .x = 79.0f,  .y = 9.0f, .a = 0.0f},
        (pos2_t){ .x = 88.0f,  .y = 9.0f, .a = 0.0f},
        (pos2_t){ .x = 95.0f,  .y = 8.0f, .a = 0.0f},
        (pos2_t){ .x = 103.0f,  .y = 9.0f, .a = 0.0f},
        (pos2_t){ .x = 110.0f,  .y = 11.0f, .a = 0.0f},
        (pos2_t){ .x = 117.0f,  .y = 14.0f, .a = 0.0f},
        (pos2_t){ .x = 119.0f,  .y = 7.0f, .a = 0.0f},
        (pos2_t){ .x = 121.0f,  .y = 3.0f, .a = 0.0f},
        (pos2_t){ .x = 124.0f,  .y = 1.0f, .a = 0.0f},
        (pos2_t){ .x = 127.0f,  .y = 0.0f, .a = 0.0f},
        (pos2_t){ .x = 127.0f,  .y = 0.0f, .a = 0.0f},
        (pos2_t){ .x = 127.0f,  .y = 0.0f, .a = 0.0f},
        (pos2_t){ .x = 130.0f,  .y = 0.0f, .a = 0.0f},
        (pos2_t){ .x = 132.0f,  .y = 1.0f, .a = 0.0f},
        (pos2_t){ .x = 135.0f,  .y = 4.0f, .a = 0.0f},
        (pos2_t){ .x = 138.0f,  .y = 11.0f, .a = 0.0f},
        (pos2_t){ .x = 138.0f,  .y = 11.0f, .a = 0.0f},
        (pos2_t){ .x = 137.0f,  .y = 25.0f, .a = 0.0f},
        (pos2_t){ .x = 151.0f,  .y = 37.0f, .a = 0.0f},
        (pos2_t){ .x = 157.0f,  .y = 44.0f, .a = 0.0f},
        (pos2_t){ .x = 163.0f,  .y = 52.0f, .a = 0.0f},
        (pos2_t){ .x = 169.0f,  .y = 61.0f, .a = 0.0f},
        (pos2_t){ .x = 173.0f,  .y = 72.0f, .a = 0.0f},
        (pos2_t){ .x = 174.0f,  .y = 77.0f, .a = 0.0f},
        (pos2_t){ .x = 176.0f,  .y = 84.0f, .a = 0.0f},
        (pos2_t){ .x = 176.0f,  .y = 90.0f, .a = 0.0f},
        (pos2_t){ .x = 177.0f,  .y = 97.0f, .a = 0.0f},
        (pos2_t){ .x = 177.0f,  .y = 97.0f, .a = 0.0f},
        (pos2_t){ .x = 177.0f,  .y = 97.0f, .a = 0.0f},
        (pos2_t){ .x = 176.0f,  .y = 108.0f, .a = 0.0f},
        (pos2_t){ .x = 174.0f,  .y = 118.0f, .a = 0.0f},
        (pos2_t){ .x = 171.0f,  .y = 128.0f, .a = 0.0f},
        (pos2_t){ .x = 167.0f,  .y = 137.0f, .a = 0.0f},
        (pos2_t){ .x = 163.0f,  .y = 145.0f, .a = 0.0f},
        (pos2_t){ .x = 157.0f,  .y = 153.0f, .a = 0.0f},
        (pos2_t){ .x = 150.0f,  .y = 161.0f, .a = 0.0f},
        (pos2_t){ .x = 142.0f,  .y = 168.0f, .a = 0.0f},
        (pos2_t){ .x = 142.0f,  .y = 189.0f, .a = 0.0f},
        (pos2_t){ .x = 142.0f,  .y = 189.0f, .a = 0.0f},
        (pos2_t){ .x = 140.0f,  .y = 194.0f, .a = 0.0f},
        (pos2_t){ .x = 138.0f,  .y = 199.0f, .a = 0.0f},
        (pos2_t){ .x = 134.0f,  .y = 202.0f, .a = 0.0f},
        (pos2_t){ .x = 129.0f,  .y = 204.0f, .a = 0.0f},
        (pos2_t){ .x = 129.0f,  .y = 204.0f, .a = 0.0f},
        (pos2_t){ .x = 129.0f,  .y = 204.0f, .a = 0.0f},
        (pos2_t){ .x = 125.0f,  .y = 203.0f, .a = 0.0f},
        (pos2_t){ .x = 122.0f,  .y = 202.0f, .a = 0.0f},
        (pos2_t){ .x = 119.0f,  .y = 200.0f, .a = 0.0f},
        (pos2_t){ .x = 117.0f,  .y = 197.0f, .a = 0.0f},
        (pos2_t){ .x = 116.0f,  .y = 194.0f, .a = 0.0f},
        (pos2_t){ .x = 114.0f,  .y = 191.0f, .a = 0.0f},
        (pos2_t){ .x = 113.0f,  .y = 183.0f, .a = 0.0f},
        (pos2_t){ .x = 107.0f,  .y = 184.0f, .a = 0.0f},
        (pos2_t){ .x = 101.0f,  .y = 185.0f, .a = 0.0f},
        (pos2_t){ .x = 95.0f,  .y = 186.0f, .a = 0.0f},
        (pos2_t){ .x = 89.0f,  .y = 186.0f, .a = 0.0f},
        (pos2_t){ .x = 76.0f,  .y = 185.0f, .a = 0.0f},
        (pos2_t){ .x = 63.0f,  .y = 183.0f, .a = 0.0f},
        (pos2_t){ .x = 62.0f,  .y = 189.0f, .a = 0.0f},
        (pos2_t){ .x = 60.0f,  .y = 195.0f, .a = 0.0f},
        (pos2_t){ .x = 59.0f,  .y = 197.0f, .a = 0.0f},
        (pos2_t){ .x = 57.0f,  .y = 200.0f, .a = 0.0f},
        (pos2_t){ .x = 53.0f,  .y = 202.0f, .a = 0.0f},
        (pos2_t){ .x = 49.0f,  .y = 202.0f, .a = 0.0f},
    };
    int draw_wps_len = 104;
    // end of drawing
    // scale
    float scaling = 3.0f;
    for (int i = 0; i < draw_wps_len; i++) {
        draw_wps[i].x *= scaling;
        draw_wps[i].y *= scaling;
        LOG_DBG("p[%d]: %.2f %.2f %.2f", i, draw_wps[i].x, draw_wps[i].y, draw_wps[i].a);
    }
    control_set_waypoints(&shared_ctrl, draw_wps, draw_wps_len);
    control_task_wait_target_default(100000.0f, 10.0f);
}
