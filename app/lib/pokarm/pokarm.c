#include "pokarm.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

#include "servo_pwm/servo_pwm.h"
#include "shared.h"
#include "pokutils.h"
#include "control/control.h"
#include "tmc2209/tmc2209.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pokarm);

/**
 * @brief pokarm
 * Module contolling the robot arm of the pokibot
 * AXES:
 * - Z : stepper
 * - rotation on Z axis : servo
 * - rotation on Y axis : servo
 */

typedef struct pokarm {
    servo_pwm_t servo_orientation;
    servo_pwm_t servo_arm;
    tmc2209_t z_stepper;
} pokarm_t;
static pokarm_t shared_pokarm = {
    .servo_orientation.spec = PWM_DT_SPEC_GET(DT_ALIAS(servo_orientation)),
    .servo_arm.spec = PWM_DT_SPEC_GET(DT_ALIAS(servo_arm)),
};

int pokarm_init(void)
{
    int err = 0;

    const servo_pwm_config_t servo_config = {.period = NSEC_PER_SEC / 50,
                                             .min_angle = 0,
                                             .max_angle = M_PI,
                                             .min_pulse = 500000,
                                             .max_pulse = 2500000};
    shared_pokarm.servo_orientation.config = servo_config;
    shared_pokarm.servo_arm.config = servo_config;

    err |= servo_pwm_init(&shared_pokarm.servo_orientation);
    err |= servo_pwm_init(&shared_pokarm.servo_arm);
    err |= tmc2209_init(&shared_pokarm.z_stepper, &steppers_uart_hdb, 3);
    err |= tmc2209_set_ihold_irun(&shared_pokarm.z_stepper, 1, 6);
    err |= tmc2209_set_mres(&shared_pokarm.z_stepper, TMC2209_MRES_1);
    if (err) {
        LOG_ERR("Error in init %d", err);
    }
    LOG_INF("Init done");
    return err;
}

int pokarm_up(void)
{
    int err = 0;
    err |= servo_pwm_set_angle(&shared_pokarm.servo_arm, 0);
    return err;
}

int pokarm_pos_flat_hexagone(void)
{
    int err = 0;
    err |= servo_pwm_set_angle(&shared_pokarm.servo_arm, M_PI * 3 / 4);
    return err;
}

int pokarm_pos_put_haxagone_display(void)
{
    int err = 0;
    err |= servo_pwm_set_angle(&shared_pokarm.servo_arm, M_PI / 2);
    return err;
}

void pokarm_test(void)
{
    LOG_INF("---------- TEST -------------");
    int err;
    uint8_t steps = 6;
    for (float angle = 0; angle < M_PI; angle += (M_PI / steps)) {
        err = servo_pwm_set_angle(&shared_pokarm.servo_arm, angle);
        if (err) {
            LOG_WRN("err %d", err);
        }
        k_sleep(K_MSEC(1000));
    }
    while (1) {
        k_sleep(K_MSEC(1000));
    }
}

void _test_pokarm_stepper(void)
{
    LOG_INF("test pokarm stepper");
    pokarm_init();
    LOG_INF("0");
    tmc2209_set_speed(&shared_pokarm.z_stepper, 0);
    k_sleep(K_MSEC(1000));
    LOG_INF("10");
    tmc2209_set_speed(&shared_pokarm.z_stepper, 48000);
    k_sleep(K_MSEC(3000));
    LOG_INF("0");
    tmc2209_set_speed(&shared_pokarm.z_stepper, 0);
    k_sleep(K_MSEC(1000));
    LOG_INF("-10");
    tmc2209_set_speed(&shared_pokarm.z_stepper, -42000);
    k_sleep(K_MSEC(3000));
    LOG_INF("0");
    tmc2209_set_speed(&shared_pokarm.z_stepper, 0);
    tmc2209_set_ihold(&shared_pokarm.z_stepper, 0);
    k_sleep(K_MSEC(1000));
    LOG_INF("test done");
}

void pen_up(void)
{
    tmc2209_set_speed(&shared_pokarm.z_stepper, 48000);
    k_sleep(K_MSEC(500));
    tmc2209_set_speed(&shared_pokarm.z_stepper, 0);
}

void pen_down(void)
{
    tmc2209_set_speed(&shared_pokarm.z_stepper, -49000);
    k_sleep(K_MSEC(500));
    tmc2209_set_speed(&shared_pokarm.z_stepper, -500);
}

void drawing_poki2(void) {
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
    pokarm_init();
    LOG_DBG("alive");
    // gpio_pin_toggle(led.port, led.pin);
    pos2_t target = (pos2_t){0.0f, 0.0f, 0.0f};
    control_set_pos(&shared_ctrl, (pos2_t){0.0f, 0.0f, 0.0f});
    control_set_waypoints(&shared_ctrl, &target, 1);
    k_sleep(K_MSEC(1000));
    shared_ctrl.start = true;


    // COPY HERE
    int wps_len;
    pen_up();
    pen_down();
    pen_up();

    // drawing start
    pos2_t wps0[] = {
        (pos2_t){.x = 132, .y = 570, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps0);
    control_set_waypoints(&shared_ctrl, wps0, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps1[] = {
        (pos2_t){.x = 132, .y = 522, .a = 0.0f},
        (pos2_t){.x = 132, .y = 516, .a = 0.0f},
        (pos2_t){.x = 135, .y = 513, .a = 0.0f},
        (pos2_t){.x = 138, .y = 507, .a = 0.0f},
        (pos2_t){.x = 144, .y = 507, .a = 0.0f},
        (pos2_t){.x = 147, .y = 507, .a = 0.0f},
        (pos2_t){.x = 153, .y = 510, .a = 0.0f},
        (pos2_t){.x = 156, .y = 516, .a = 0.0f},
        (pos2_t){.x = 156, .y = 519, .a = 0.0f},
        (pos2_t){.x = 156, .y = 570, .a = 0.0f},
        (pos2_t){.x = 156, .y = 576, .a = 0.0f},
        (pos2_t){.x = 153, .y = 579, .a = 0.0f},
        (pos2_t){.x = 150, .y = 582, .a = 0.0f},
        (pos2_t){.x = 144, .y = 582, .a = 0.0f},
        (pos2_t){.x = 138, .y = 582, .a = 0.0f},
        (pos2_t){.x = 135, .y = 579, .a = 0.0f},
        (pos2_t){.x = 132, .y = 576, .a = 0.0f},
        (pos2_t){.x = 132, .y = 570, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps1);
    control_set_waypoints(&shared_ctrl, wps1, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps18[] = {
        (pos2_t){.x = 384, .y = 582, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps18);
    control_set_waypoints(&shared_ctrl, wps18, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps19[] = {
        (pos2_t){.x = 387, .y = 582, .a = 0.0f},
        (pos2_t){.x = 393, .y = 579, .a = 0.0f},
        (pos2_t){.x = 396, .y = 576, .a = 0.0f},
        (pos2_t){.x = 396, .y = 570, .a = 0.0f},
        (pos2_t){.x = 396, .y = 522, .a = 0.0f},
        (pos2_t){.x = 396, .y = 516, .a = 0.0f},
        (pos2_t){.x = 393, .y = 513, .a = 0.0f},
        (pos2_t){.x = 390, .y = 507, .a = 0.0f},
        (pos2_t){.x = 384, .y = 507, .a = 0.0f},
        (pos2_t){.x = 378, .y = 507, .a = 0.0f},
        (pos2_t){.x = 375, .y = 510, .a = 0.0f},
        (pos2_t){.x = 372, .y = 516, .a = 0.0f},
        (pos2_t){.x = 372, .y = 519, .a = 0.0f},
        (pos2_t){.x = 372, .y = 570, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps19);
    control_set_waypoints(&shared_ctrl, wps19, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps2[] = {
        (pos2_t){.x = 183, .y = 396, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps2);
    control_set_waypoints(&shared_ctrl, wps2, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps3[] = {
        (pos2_t){.x = 177, .y = 396, .a = 0.0f},
        (pos2_t){.x = 171, .y = 393, .a = 0.0f},
        (pos2_t){.x = 165, .y = 387, .a = 0.0f},
        (pos2_t){.x = 159, .y = 375, .a = 0.0f},
        (pos2_t){.x = 156, .y = 369, .a = 0.0f},
        (pos2_t){.x = 159, .y = 321, .a = 0.0f},
        (pos2_t){.x = 159, .y = 312, .a = 0.0f},
        (pos2_t){.x = 165, .y = 303, .a = 0.0f},
        (pos2_t){.x = 171, .y = 294, .a = 0.0f},
        (pos2_t){.x = 180, .y = 291, .a = 0.0f},
        (pos2_t){.x = 192, .y = 294, .a = 0.0f},
        (pos2_t){.x = 201, .y = 300, .a = 0.0f},
        (pos2_t){.x = 207, .y = 309, .a = 0.0f},
        (pos2_t){.x = 210, .y = 318, .a = 0.0f},
        (pos2_t){.x = 210, .y = 372, .a = 0.0f},
        (pos2_t){.x = 207, .y = 378, .a = 0.0f},
        (pos2_t){.x = 204, .y = 387, .a = 0.0f},
        (pos2_t){.x = 195, .y = 393, .a = 0.0f},
        (pos2_t){.x = 192, .y = 396, .a = 0.0f},
        (pos2_t){.x = 183, .y = 396, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps3);
    control_set_waypoints(&shared_ctrl, wps3, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps4[] = {
        (pos2_t){.x = 195, .y = 384, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps4);
    control_set_waypoints(&shared_ctrl, wps4, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps5[] = {
        (pos2_t){.x = 207, .y = 366, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps5);
    control_set_waypoints(&shared_ctrl, wps5, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps6[] = {
        (pos2_t){.x = 342, .y = 396, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps6);
    control_set_waypoints(&shared_ctrl, wps6, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps7[] = {
        (pos2_t){.x = 348, .y = 396, .a = 0.0f},
        (pos2_t){.x = 354, .y = 393, .a = 0.0f},
        (pos2_t){.x = 363, .y = 387, .a = 0.0f},
        (pos2_t){.x = 369, .y = 375, .a = 0.0f},
        (pos2_t){.x = 369, .y = 369, .a = 0.0f},
        (pos2_t){.x = 369, .y = 321, .a = 0.0f},
        (pos2_t){.x = 366, .y = 312, .a = 0.0f},
        (pos2_t){.x = 363, .y = 303, .a = 0.0f},
        (pos2_t){.x = 357, .y = 294, .a = 0.0f},
        (pos2_t){.x = 345, .y = 291, .a = 0.0f},
        (pos2_t){.x = 336, .y = 294, .a = 0.0f},
        (pos2_t){.x = 327, .y = 300, .a = 0.0f},
        (pos2_t){.x = 321, .y = 309, .a = 0.0f},
        (pos2_t){.x = 318, .y = 318, .a = 0.0f},
        (pos2_t){.x = 318, .y = 372, .a = 0.0f},
        (pos2_t){.x = 318, .y = 378, .a = 0.0f},
        (pos2_t){.x = 324, .y = 387, .a = 0.0f},
        (pos2_t){.x = 330, .y = 393, .a = 0.0f},
        (pos2_t){.x = 336, .y = 396, .a = 0.0f},
        (pos2_t){.x = 342, .y = 396, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps7);
    control_set_waypoints(&shared_ctrl, wps7, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps8[] = {
        (pos2_t){.x = 348, .y = 387, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps8);
    control_set_waypoints(&shared_ctrl, wps8, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps9[] = {
        (pos2_t){.x = 360, .y = 366, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps9);
    control_set_waypoints(&shared_ctrl, wps9, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps10[] = {
        (pos2_t){.x = 288, .y = 210, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps10);
    control_set_waypoints(&shared_ctrl, wps10, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps11[] = {
        (pos2_t){.x = 288, .y = 201, .a = 0.0f},
        (pos2_t){.x = 282, .y = 192, .a = 0.0f},
        (pos2_t){.x = 273, .y = 186, .a = 0.0f},
        (pos2_t){.x = 264, .y = 186, .a = 0.0f},
        (pos2_t){.x = 252, .y = 186, .a = 0.0f},
        (pos2_t){.x = 246, .y = 192, .a = 0.0f},
        (pos2_t){.x = 240, .y = 201, .a = 0.0f},
        (pos2_t){.x = 237, .y = 210, .a = 0.0f},
        (pos2_t){.x = 240, .y = 222, .a = 0.0f},
        (pos2_t){.x = 246, .y = 228, .a = 0.0f},
        (pos2_t){.x = 252, .y = 234, .a = 0.0f},
        (pos2_t){.x = 264, .y = 237, .a = 0.0f},
        (pos2_t){.x = 273, .y = 234, .a = 0.0f},
        (pos2_t){.x = 282, .y = 228, .a = 0.0f},
        (pos2_t){.x = 288, .y = 222, .a = 0.0f},
        (pos2_t){.x = 288, .y = 210, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps11);
    control_set_waypoints(&shared_ctrl, wps11, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps12[] = {
        (pos2_t){.x = 342, .y = 171, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps12);
    control_set_waypoints(&shared_ctrl, wps12, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps13[] = {
        (pos2_t){.x = 339, .y = 156, .a = 0.0f},
        (pos2_t){.x = 336, .y = 141, .a = 0.0f},
        (pos2_t){.x = 327, .y = 129, .a = 0.0f},
        (pos2_t){.x = 318, .y = 117, .a = 0.0f},
        (pos2_t){.x = 306, .y = 108, .a = 0.0f},
        (pos2_t){.x = 294, .y = 99, .a = 0.0f},
        (pos2_t){.x = 279, .y = 96, .a = 0.0f},
        (pos2_t){.x = 264, .y = 93, .a = 0.0f},
        (pos2_t){.x = 249, .y = 96, .a = 0.0f},
        (pos2_t){.x = 234, .y = 99, .a = 0.0f},
        (pos2_t){.x = 222, .y = 108, .a = 0.0f},
        (pos2_t){.x = 210, .y = 117, .a = 0.0f},
        (pos2_t){.x = 201, .y = 129, .a = 0.0f},
        (pos2_t){.x = 192, .y = 141, .a = 0.0f},
        (pos2_t){.x = 189, .y = 156, .a = 0.0f},
        (pos2_t){.x = 186, .y = 171, .a = 0.0f},
        (pos2_t){.x = 189, .y = 186, .a = 0.0f},
        (pos2_t){.x = 192, .y = 201, .a = 0.0f},
        (pos2_t){.x = 201, .y = 213, .a = 0.0f},
        (pos2_t){.x = 210, .y = 225, .a = 0.0f},
        (pos2_t){.x = 222, .y = 234, .a = 0.0f},
        (pos2_t){.x = 234, .y = 243, .a = 0.0f},
        (pos2_t){.x = 249, .y = 246, .a = 0.0f},
        (pos2_t){.x = 264, .y = 249, .a = 0.0f},
        (pos2_t){.x = 279, .y = 246, .a = 0.0f},
        (pos2_t){.x = 294, .y = 243, .a = 0.0f},
        (pos2_t){.x = 306, .y = 234, .a = 0.0f},
        (pos2_t){.x = 318, .y = 225, .a = 0.0f},
        (pos2_t){.x = 327, .y = 213, .a = 0.0f},
        (pos2_t){.x = 336, .y = 201, .a = 0.0f},
        (pos2_t){.x = 339, .y = 186, .a = 0.0f},
        (pos2_t){.x = 342, .y = 171, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps13);
    control_set_waypoints(&shared_ctrl, wps13, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps14[] = {
        (pos2_t){.x = 219, .y = 135, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps14);
    control_set_waypoints(&shared_ctrl, wps14, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps15[] = {
        (pos2_t){.x = 216, .y = 132, .a = 0.0f},
        (pos2_t){.x = 219, .y = 129, .a = 0.0f},
        (pos2_t){.x = 228, .y = 117, .a = 0.0f},
        (pos2_t){.x = 243, .y = 108, .a = 0.0f},
        (pos2_t){.x = 252, .y = 105, .a = 0.0f},
        (pos2_t){.x = 264, .y = 105, .a = 0.0f},
        (pos2_t){.x = 273, .y = 105, .a = 0.0f},
        (pos2_t){.x = 282, .y = 108, .a = 0.0f},
        (pos2_t){.x = 297, .y = 117, .a = 0.0f},
        (pos2_t){.x = 306, .y = 129, .a = 0.0f},
        (pos2_t){.x = 309, .y = 132, .a = 0.0f},
        (pos2_t){.x = 309, .y = 135, .a = 0.0f},
        (pos2_t){.x = 306, .y = 138, .a = 0.0f},
        (pos2_t){.x = 306, .y = 138, .a = 0.0f},
        (pos2_t){.x = 297, .y = 132, .a = 0.0f},
        (pos2_t){.x = 285, .y = 123, .a = 0.0f},
        (pos2_t){.x = 276, .y = 120, .a = 0.0f},
        (pos2_t){.x = 264, .y = 120, .a = 0.0f},
        (pos2_t){.x = 252, .y = 120, .a = 0.0f},
        (pos2_t){.x = 243, .y = 123, .a = 0.0f},
        (pos2_t){.x = 231, .y = 129, .a = 0.0f},
        (pos2_t){.x = 222, .y = 135, .a = 0.0f},
        (pos2_t){.x = 219, .y = 138, .a = 0.0f},
        (pos2_t){.x = 219, .y = 135, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps15);
    control_set_waypoints(&shared_ctrl, wps15, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps16[] = {
        (pos2_t){.x = 114, .y = 72, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps16);
    control_set_waypoints(&shared_ctrl, wps16, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps17[] = {
        (pos2_t){.x = 87, .y = 93, .a = 0.0f},
        (pos2_t){.x = 63, .y = 117, .a = 0.0f},
        (pos2_t){.x = 45, .y = 141, .a = 0.0f},
        (pos2_t){.x = 27, .y = 168, .a = 0.0f},
        (pos2_t){.x = 15, .y = 195, .a = 0.0f},
        (pos2_t){.x = 6, .y = 225, .a = 0.0f},
        (pos2_t){.x = 0, .y = 258, .a = 0.0f},
        (pos2_t){.x = 0, .y = 291, .a = 0.0f},
        (pos2_t){.x = 0, .y = 318, .a = 0.0f},
        (pos2_t){.x = 3, .y = 342, .a = 0.0f},
        (pos2_t){.x = 9, .y = 369, .a = 0.0f},
        (pos2_t){.x = 18, .y = 396, .a = 0.0f},
        (pos2_t){.x = 30, .y = 423, .a = 0.0f},
        (pos2_t){.x = 48, .y = 450, .a = 0.0f},
        (pos2_t){.x = 72, .y = 477, .a = 0.0f},
        (pos2_t){.x = 102, .y = 504, .a = 0.0f},
        (pos2_t){.x = 102, .y = 570, .a = 0.0f},
        (pos2_t){.x = 105, .y = 576, .a = 0.0f},
        (pos2_t){.x = 105, .y = 585, .a = 0.0f},
        (pos2_t){.x = 117, .y = 597, .a = 0.0f},
        (pos2_t){.x = 129, .y = 606, .a = 0.0f},
        (pos2_t){.x = 135, .y = 609, .a = 0.0f},
        (pos2_t){.x = 144, .y = 612, .a = 0.0f},
        (pos2_t){.x = 156, .y = 609, .a = 0.0f},
        (pos2_t){.x = 168, .y = 603, .a = 0.0f},
        (pos2_t){.x = 177, .y = 594, .a = 0.0f},
        (pos2_t){.x = 180, .y = 585, .a = 0.0f},
        (pos2_t){.x = 183, .y = 573, .a = 0.0f},
        (pos2_t){.x = 186, .y = 564, .a = 0.0f},
        (pos2_t){.x = 186, .y = 546, .a = 0.0f},
        (pos2_t){.x = 207, .y = 552, .a = 0.0f},
        (pos2_t){.x = 225, .y = 555, .a = 0.0f},
        (pos2_t){.x = 246, .y = 555, .a = 0.0f},
        (pos2_t){.x = 264, .y = 555, .a = 0.0f},
        (pos2_t){.x = 303, .y = 555, .a = 0.0f},
        (pos2_t){.x = 321, .y = 552, .a = 0.0f},
        (pos2_t){.x = 339, .y = 546, .a = 0.0f},
        (pos2_t){.x = 339, .y = 561, .a = 0.0f},
        (pos2_t){.x = 342, .y = 573, .a = 0.0f},
        (pos2_t){.x = 348, .y = 585, .a = 0.0f},
        (pos2_t){.x = 351, .y = 594, .a = 0.0f},
        (pos2_t){.x = 360, .y = 600, .a = 0.0f},
        (pos2_t){.x = 366, .y = 606, .a = 0.0f},
        (pos2_t){.x = 375, .y = 609, .a = 0.0f},
        (pos2_t){.x = 384, .y = 609, .a = 0.0f},
        (pos2_t){.x = 396, .y = 609, .a = 0.0f},
        (pos2_t){.x = 402, .y = 606, .a = 0.0f},
        (pos2_t){.x = 411, .y = 600, .a = 0.0f},
        (pos2_t){.x = 417, .y = 594, .a = 0.0f},
        (pos2_t){.x = 420, .y = 585, .a = 0.0f},
        (pos2_t){.x = 423, .y = 576, .a = 0.0f},
        (pos2_t){.x = 426, .y = 558, .a = 0.0f},
        (pos2_t){.x = 426, .y = 504, .a = 0.0f},
        (pos2_t){.x = 453, .y = 477, .a = 0.0f},
        (pos2_t){.x = 477, .y = 453, .a = 0.0f},
        (pos2_t){.x = 495, .y = 426, .a = 0.0f},
        (pos2_t){.x = 507, .y = 399, .a = 0.0f},
        (pos2_t){.x = 519, .y = 372, .a = 0.0f},
        (pos2_t){.x = 525, .y = 345, .a = 0.0f},
        (pos2_t){.x = 528, .y = 318, .a = 0.0f},
        (pos2_t){.x = 531, .y = 288, .a = 0.0f},
        (pos2_t){.x = 528, .y = 252, .a = 0.0f},
        (pos2_t){.x = 519, .y = 216, .a = 0.0f},
        (pos2_t){.x = 510, .y = 186, .a = 0.0f},
        (pos2_t){.x = 495, .y = 159, .a = 0.0f},
        (pos2_t){.x = 477, .y = 132, .a = 0.0f},
        (pos2_t){.x = 456, .y = 111, .a = 0.0f},
        (pos2_t){.x = 435, .y = 90, .a = 0.0f},
        (pos2_t){.x = 411, .y = 72, .a = 0.0f},
        (pos2_t){.x = 411, .y = 27, .a = 0.0f},
        (pos2_t){.x = 408, .y = 15, .a = 0.0f},
        (pos2_t){.x = 402, .y = 6, .a = 0.0f},
        (pos2_t){.x = 393, .y = 0, .a = 0.0f},
        (pos2_t){.x = 384, .y = 0, .a = 0.0f},
        (pos2_t){.x = 378, .y = 0, .a = 0.0f},
        (pos2_t){.x = 372, .y = 3, .a = 0.0f},
        (pos2_t){.x = 363, .y = 12, .a = 0.0f},
        (pos2_t){.x = 357, .y = 24, .a = 0.0f},
        (pos2_t){.x = 354, .y = 42, .a = 0.0f},
        (pos2_t){.x = 336, .y = 33, .a = 0.0f},
        (pos2_t){.x = 312, .y = 27, .a = 0.0f},
        (pos2_t){.x = 288, .y = 24, .a = 0.0f},
        (pos2_t){.x = 264, .y = 24, .a = 0.0f},
        (pos2_t){.x = 240, .y = 24, .a = 0.0f},
        (pos2_t){.x = 213, .y = 30, .a = 0.0f},
        (pos2_t){.x = 192, .y = 36, .a = 0.0f},
        (pos2_t){.x = 174, .y = 42, .a = 0.0f},
        (pos2_t){.x = 171, .y = 27, .a = 0.0f},
        (pos2_t){.x = 165, .y = 12, .a = 0.0f},
        (pos2_t){.x = 156, .y = 3, .a = 0.0f},
        (pos2_t){.x = 150, .y = 0, .a = 0.0f},
        (pos2_t){.x = 144, .y = 0, .a = 0.0f},
        (pos2_t){.x = 132, .y = 0, .a = 0.0f},
        (pos2_t){.x = 123, .y = 9, .a = 0.0f},
        (pos2_t){.x = 117, .y = 18, .a = 0.0f},
        (pos2_t){.x = 114, .y = 33, .a = 0.0f},
        (pos2_t){.x = 114, .y = 72, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps17);
    control_set_waypoints(&shared_ctrl, wps17, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps20[] = {
        (pos2_t){.x = 0, .y = 0, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps20);
    pen_up();
    control_set_waypoints(&shared_ctrl, wps20, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    // drawing end

    pos2_t wps21[] = {
        (pos2_t){.x = 0.0f, .y = 0.0f, .a = 10.0f  * M_PI},
    };
    control_set_waypoints(&shared_ctrl, wps21, wps_len);
    control_task_wait_target_default(15.0f, 10.0f);
}

void drawing_pokicake(void) {
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
    pokarm_init();
    LOG_DBG("alive");
    // gpio_pin_toggle(led.port, led.pin);
    pos2_t target = (pos2_t){0.0f, 0.0f, 0.0f};
    control_set_pos(&shared_ctrl, (pos2_t){0.0f, 0.0f, 0.0f});
    control_set_waypoints(&shared_ctrl, &target, 1);
    k_sleep(K_MSEC(1000));
    shared_ctrl.start = true;


    // COPY HERE
    int wps_len;
    pen_up();
    pen_down();
    pen_up();

    // drawing start
    pos2_t wps0[] = {
        (pos2_t){.x = 16, .y = 0, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps0);
    control_set_waypoints(&shared_ctrl, wps0, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps1[] = {
        (pos2_t){.x = 14, .y = 2, .a = 0.0f},
        (pos2_t){.x = 16, .y = 8, .a = 0.0f},
        (pos2_t){.x = 20, .y = 12, .a = 0.0f},
        (pos2_t){.x = 24, .y = 12, .a = 0.0f},
        (pos2_t){.x = 28, .y = 8, .a = 0.0f},
        (pos2_t){.x = 30, .y = 6, .a = 0.0f},
        (pos2_t){.x = 28, .y = 2, .a = 0.0f},
        (pos2_t){.x = 24, .y = 0, .a = 0.0f},
        (pos2_t){.x = 20, .y = 0, .a = 0.0f},
        (pos2_t){.x = 16, .y = 0, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps1);
    control_set_waypoints(&shared_ctrl, wps1, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps2[] = {
        (pos2_t){.x = 18, .y = 2, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps2);
    control_set_waypoints(&shared_ctrl, wps2, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps3[] = {
        (pos2_t){.x = 18, .y = 6, .a = 0.0f},
        (pos2_t){.x = 22, .y = 8, .a = 0.0f},
        (pos2_t){.x = 24, .y = 8, .a = 0.0f},
        (pos2_t){.x = 26, .y = 8, .a = 0.0f},
        (pos2_t){.x = 26, .y = 8, .a = 0.0f},
        (pos2_t){.x = 28, .y = 8, .a = 0.0f},
        (pos2_t){.x = 28, .y = 8, .a = 0.0f},
        (pos2_t){.x = 28, .y = 6, .a = 0.0f},
        (pos2_t){.x = 28, .y = 6, .a = 0.0f},
        (pos2_t){.x = 24, .y = 4, .a = 0.0f},
        (pos2_t){.x = 20, .y = 2, .a = 0.0f},
        (pos2_t){.x = 18, .y = 2, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps3);
    control_set_waypoints(&shared_ctrl, wps3, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps4[] = {
        (pos2_t){.x = 44, .y = 0, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps4);
    control_set_waypoints(&shared_ctrl, wps4, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps5[] = {
        (pos2_t){.x = 40, .y = 2, .a = 0.0f},
        (pos2_t){.x = 38, .y = 4, .a = 0.0f},
        (pos2_t){.x = 38, .y = 6, .a = 0.0f},
        (pos2_t){.x = 38, .y = 8, .a = 0.0f},
        (pos2_t){.x = 40, .y = 14, .a = 0.0f},
        (pos2_t){.x = 44, .y = 16, .a = 0.0f},
        (pos2_t){.x = 50, .y = 18, .a = 0.0f},
        (pos2_t){.x = 56, .y = 18, .a = 0.0f},
        (pos2_t){.x = 58, .y = 18, .a = 0.0f},
        (pos2_t){.x = 62, .y = 18, .a = 0.0f},
        (pos2_t){.x = 64, .y = 16, .a = 0.0f},
        (pos2_t){.x = 66, .y = 14, .a = 0.0f},
        (pos2_t){.x = 66, .y = 10, .a = 0.0f},
        (pos2_t){.x = 64, .y = 6, .a = 0.0f},
        (pos2_t){.x = 58, .y = 2, .a = 0.0f},
        (pos2_t){.x = 50, .y = 0, .a = 0.0f},
        (pos2_t){.x = 44, .y = 0, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps5);
    control_set_waypoints(&shared_ctrl, wps5, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps6[] = {
        (pos2_t){.x = 44, .y = 4, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps6);
    control_set_waypoints(&shared_ctrl, wps6, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps7[] = {
        (pos2_t){.x = 42, .y = 4, .a = 0.0f},
        (pos2_t){.x = 40, .y = 6, .a = 0.0f},
        (pos2_t){.x = 44, .y = 10, .a = 0.0f},
        (pos2_t){.x = 50, .y = 14, .a = 0.0f},
        (pos2_t){.x = 58, .y = 16, .a = 0.0f},
        (pos2_t){.x = 60, .y = 16, .a = 0.0f},
        (pos2_t){.x = 62, .y = 14, .a = 0.0f},
        (pos2_t){.x = 62, .y = 12, .a = 0.0f},
        (pos2_t){.x = 60, .y = 8, .a = 0.0f},
        (pos2_t){.x = 56, .y = 6, .a = 0.0f},
        (pos2_t){.x = 52, .y = 4, .a = 0.0f},
        (pos2_t){.x = 48, .y = 2, .a = 0.0f},
        (pos2_t){.x = 44, .y = 4, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps7);
    control_set_waypoints(&shared_ctrl, wps7, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps8[] = {
        (pos2_t){.x = 230, .y = 10, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps8);
    control_set_waypoints(&shared_ctrl, wps8, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps9[] = {
        (pos2_t){.x = 218, .y = 14, .a = 0.0f},
        (pos2_t){.x = 214, .y = 16, .a = 0.0f},
        (pos2_t){.x = 212, .y = 18, .a = 0.0f},
        (pos2_t){.x = 212, .y = 22, .a = 0.0f},
        (pos2_t){.x = 214, .y = 24, .a = 0.0f},
        (pos2_t){.x = 216, .y = 26, .a = 0.0f},
        (pos2_t){.x = 222, .y = 26, .a = 0.0f},
        (pos2_t){.x = 230, .y = 24, .a = 0.0f},
        (pos2_t){.x = 234, .y = 22, .a = 0.0f},
        (pos2_t){.x = 238, .y = 20, .a = 0.0f},
        (pos2_t){.x = 240, .y = 16, .a = 0.0f},
        (pos2_t){.x = 242, .y = 14, .a = 0.0f},
        (pos2_t){.x = 240, .y = 12, .a = 0.0f},
        (pos2_t){.x = 236, .y = 8, .a = 0.0f},
        (pos2_t){.x = 230, .y = 10, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps9);
    control_set_waypoints(&shared_ctrl, wps9, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps10[] = {
        (pos2_t){.x = 232, .y = 12, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps10);
    control_set_waypoints(&shared_ctrl, wps10, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps11[] = {
        (pos2_t){.x = 224, .y = 14, .a = 0.0f},
        (pos2_t){.x = 218, .y = 18, .a = 0.0f},
        (pos2_t){.x = 214, .y = 20, .a = 0.0f},
        (pos2_t){.x = 216, .y = 22, .a = 0.0f},
        (pos2_t){.x = 218, .y = 22, .a = 0.0f},
        (pos2_t){.x = 228, .y = 22, .a = 0.0f},
        (pos2_t){.x = 232, .y = 22, .a = 0.0f},
        (pos2_t){.x = 234, .y = 18, .a = 0.0f},
        (pos2_t){.x = 238, .y = 14, .a = 0.0f},
        (pos2_t){.x = 238, .y = 14, .a = 0.0f},
        (pos2_t){.x = 238, .y = 12, .a = 0.0f},
        (pos2_t){.x = 236, .y = 12, .a = 0.0f},
        (pos2_t){.x = 234, .y = 12, .a = 0.0f},
        (pos2_t){.x = 232, .y = 12, .a = 0.0f},
        (pos2_t){.x = 232, .y = 12, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps11);
    control_set_waypoints(&shared_ctrl, wps11, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps12[] = {
        (pos2_t){.x = 70, .y = 22, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps12);
    control_set_waypoints(&shared_ctrl, wps12, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps13[] = {
        (pos2_t){.x = 66, .y = 22, .a = 0.0f},
        (pos2_t){.x = 62, .y = 24, .a = 0.0f},
        (pos2_t){.x = 60, .y = 26, .a = 0.0f},
        (pos2_t){.x = 60, .y = 30, .a = 0.0f},
        (pos2_t){.x = 60, .y = 34, .a = 0.0f},
        (pos2_t){.x = 62, .y = 38, .a = 0.0f},
        (pos2_t){.x = 64, .y = 42, .a = 0.0f},
        (pos2_t){.x = 62, .y = 44, .a = 0.0f},
        (pos2_t){.x = 60, .y = 44, .a = 0.0f},
        (pos2_t){.x = 56, .y = 42, .a = 0.0f},
        (pos2_t){.x = 50, .y = 42, .a = 0.0f},
        (pos2_t){.x = 46, .y = 42, .a = 0.0f},
        (pos2_t){.x = 42, .y = 44, .a = 0.0f},
        (pos2_t){.x = 40, .y = 46, .a = 0.0f},
        (pos2_t){.x = 36, .y = 50, .a = 0.0f},
        (pos2_t){.x = 32, .y = 56, .a = 0.0f},
        (pos2_t){.x = 28, .y = 56, .a = 0.0f},
        (pos2_t){.x = 22, .y = 56, .a = 0.0f},
        (pos2_t){.x = 16, .y = 56, .a = 0.0f},
        (pos2_t){.x = 10, .y = 56, .a = 0.0f},
        (pos2_t){.x = 4, .y = 58, .a = 0.0f},
        (pos2_t){.x = 0, .y = 62, .a = 0.0f},
        (pos2_t){.x = 0, .y = 68, .a = 0.0f},
        (pos2_t){.x = 0, .y = 74, .a = 0.0f},
        (pos2_t){.x = 6, .y = 78, .a = 0.0f},
        (pos2_t){.x = 10, .y = 82, .a = 0.0f},
        (pos2_t){.x = 20, .y = 86, .a = 0.0f},
        (pos2_t){.x = 30, .y = 86, .a = 0.0f},
        (pos2_t){.x = 34, .y = 86, .a = 0.0f},
        (pos2_t){.x = 34, .y = 88, .a = 0.0f},
        (pos2_t){.x = 34, .y = 92, .a = 0.0f},
        (pos2_t){.x = 36, .y = 94, .a = 0.0f},
        (pos2_t){.x = 38, .y = 96, .a = 0.0f},
        (pos2_t){.x = 42, .y = 96, .a = 0.0f},
        (pos2_t){.x = 46, .y = 96, .a = 0.0f},
        (pos2_t){.x = 50, .y = 92, .a = 0.0f},
        (pos2_t){.x = 54, .y = 90, .a = 0.0f},
        (pos2_t){.x = 56, .y = 92, .a = 0.0f},
        (pos2_t){.x = 58, .y = 94, .a = 0.0f},
        (pos2_t){.x = 60, .y = 94, .a = 0.0f},
        (pos2_t){.x = 62, .y = 94, .a = 0.0f},
        (pos2_t){.x = 62, .y = 98, .a = 0.0f},
        (pos2_t){.x = 62, .y = 100, .a = 0.0f},
        (pos2_t){.x = 60, .y = 102, .a = 0.0f},
        (pos2_t){.x = 58, .y = 102, .a = 0.0f},
        (pos2_t){.x = 58, .y = 104, .a = 0.0f},
        (pos2_t){.x = 58, .y = 106, .a = 0.0f},
        (pos2_t){.x = 58, .y = 112, .a = 0.0f},
        (pos2_t){.x = 62, .y = 116, .a = 0.0f},
        (pos2_t){.x = 70, .y = 124, .a = 0.0f},
        (pos2_t){.x = 70, .y = 124, .a = 0.0f},
        (pos2_t){.x = 72, .y = 134, .a = 0.0f},
        (pos2_t){.x = 80, .y = 144, .a = 0.0f},
        (pos2_t){.x = 82, .y = 144, .a = 0.0f},
        (pos2_t){.x = 80, .y = 144, .a = 0.0f},
        (pos2_t){.x = 76, .y = 144, .a = 0.0f},
        (pos2_t){.x = 74, .y = 144, .a = 0.0f},
        (pos2_t){.x = 70, .y = 150, .a = 0.0f},
        (pos2_t){.x = 68, .y = 156, .a = 0.0f},
        (pos2_t){.x = 68, .y = 160, .a = 0.0f},
        (pos2_t){.x = 70, .y = 164, .a = 0.0f},
        (pos2_t){.x = 72, .y = 166, .a = 0.0f},
        (pos2_t){.x = 74, .y = 166, .a = 0.0f},
        (pos2_t){.x = 72, .y = 172, .a = 0.0f},
        (pos2_t){.x = 72, .y = 186, .a = 0.0f},
        (pos2_t){.x = 72, .y = 196, .a = 0.0f},
        (pos2_t){.x = 74, .y = 206, .a = 0.0f},
        (pos2_t){.x = 78, .y = 220, .a = 0.0f},
        (pos2_t){.x = 84, .y = 234, .a = 0.0f},
        (pos2_t){.x = 94, .y = 246, .a = 0.0f},
        (pos2_t){.x = 106, .y = 256, .a = 0.0f},
        (pos2_t){.x = 120, .y = 264, .a = 0.0f},
        (pos2_t){.x = 122, .y = 266, .a = 0.0f},
        (pos2_t){.x = 120, .y = 270, .a = 0.0f},
        (pos2_t){.x = 118, .y = 282, .a = 0.0f},
        (pos2_t){.x = 116, .y = 294, .a = 0.0f},
        (pos2_t){.x = 118, .y = 306, .a = 0.0f},
        (pos2_t){.x = 118, .y = 318, .a = 0.0f},
        (pos2_t){.x = 122, .y = 328, .a = 0.0f},
        (pos2_t){.x = 126, .y = 336, .a = 0.0f},
        (pos2_t){.x = 132, .y = 344, .a = 0.0f},
        (pos2_t){.x = 136, .y = 350, .a = 0.0f},
        (pos2_t){.x = 144, .y = 352, .a = 0.0f},
        (pos2_t){.x = 152, .y = 354, .a = 0.0f},
        (pos2_t){.x = 158, .y = 354, .a = 0.0f},
        (pos2_t){.x = 162, .y = 352, .a = 0.0f},
        (pos2_t){.x = 168, .y = 350, .a = 0.0f},
        (pos2_t){.x = 170, .y = 344, .a = 0.0f},
        (pos2_t){.x = 174, .y = 336, .a = 0.0f},
        (pos2_t){.x = 176, .y = 324, .a = 0.0f},
        (pos2_t){.x = 178, .y = 314, .a = 0.0f},
        (pos2_t){.x = 178, .y = 304, .a = 0.0f},
        (pos2_t){.x = 178, .y = 302, .a = 0.0f},
        (pos2_t){.x = 180, .y = 304, .a = 0.0f},
        (pos2_t){.x = 182, .y = 306, .a = 0.0f},
        (pos2_t){.x = 186, .y = 320, .a = 0.0f},
        (pos2_t){.x = 194, .y = 334, .a = 0.0f},
        (pos2_t){.x = 204, .y = 352, .a = 0.0f},
        (pos2_t){.x = 222, .y = 376, .a = 0.0f},
        (pos2_t){.x = 232, .y = 384, .a = 0.0f},
        (pos2_t){.x = 246, .y = 394, .a = 0.0f},
        (pos2_t){.x = 252, .y = 398, .a = 0.0f},
        (pos2_t){.x = 252, .y = 398, .a = 0.0f},
        (pos2_t){.x = 252, .y = 398, .a = 0.0f},
        (pos2_t){.x = 252, .y = 394, .a = 0.0f},
        (pos2_t){.x = 252, .y = 384, .a = 0.0f},
        (pos2_t){.x = 252, .y = 376, .a = 0.0f},
        (pos2_t){.x = 254, .y = 366, .a = 0.0f},
        (pos2_t){.x = 256, .y = 358, .a = 0.0f},
        (pos2_t){.x = 260, .y = 352, .a = 0.0f},
        (pos2_t){.x = 270, .y = 338, .a = 0.0f},
        (pos2_t){.x = 278, .y = 334, .a = 0.0f},
        (pos2_t){.x = 284, .y = 330, .a = 0.0f},
        (pos2_t){.x = 290, .y = 328, .a = 0.0f},
        (pos2_t){.x = 296, .y = 326, .a = 0.0f},
        (pos2_t){.x = 296, .y = 322, .a = 0.0f},
        (pos2_t){.x = 294, .y = 318, .a = 0.0f},
        (pos2_t){.x = 290, .y = 310, .a = 0.0f},
        (pos2_t){.x = 284, .y = 304, .a = 0.0f},
        (pos2_t){.x = 276, .y = 300, .a = 0.0f},
        (pos2_t){.x = 268, .y = 294, .a = 0.0f},
        (pos2_t){.x = 258, .y = 292, .a = 0.0f},
        (pos2_t){.x = 258, .y = 292, .a = 0.0f},
        (pos2_t){.x = 258, .y = 292, .a = 0.0f},
        (pos2_t){.x = 256, .y = 288, .a = 0.0f},
        (pos2_t){.x = 250, .y = 272, .a = 0.0f},
        (pos2_t){.x = 240, .y = 254, .a = 0.0f},
        (pos2_t){.x = 226, .y = 240, .a = 0.0f},
        (pos2_t){.x = 224, .y = 238, .a = 0.0f},
        (pos2_t){.x = 226, .y = 234, .a = 0.0f},
        (pos2_t){.x = 232, .y = 222, .a = 0.0f},
        (pos2_t){.x = 236, .y = 210, .a = 0.0f},
        (pos2_t){.x = 238, .y = 196, .a = 0.0f},
        (pos2_t){.x = 238, .y = 182, .a = 0.0f},
        (pos2_t){.x = 236, .y = 168, .a = 0.0f},
        (pos2_t){.x = 232, .y = 154, .a = 0.0f},
        (pos2_t){.x = 230, .y = 150, .a = 0.0f},
        (pos2_t){.x = 232, .y = 150, .a = 0.0f},
        (pos2_t){.x = 236, .y = 144, .a = 0.0f},
        (pos2_t){.x = 238, .y = 140, .a = 0.0f},
        (pos2_t){.x = 236, .y = 136, .a = 0.0f},
        (pos2_t){.x = 230, .y = 126, .a = 0.0f},
        (pos2_t){.x = 228, .y = 126, .a = 0.0f},
        (pos2_t){.x = 230, .y = 124, .a = 0.0f},
        (pos2_t){.x = 236, .y = 118, .a = 0.0f},
        (pos2_t){.x = 238, .y = 116, .a = 0.0f},
        (pos2_t){.x = 236, .y = 112, .a = 0.0f},
        (pos2_t){.x = 236, .y = 108, .a = 0.0f},
        (pos2_t){.x = 236, .y = 104, .a = 0.0f},
        (pos2_t){.x = 234, .y = 100, .a = 0.0f},
        (pos2_t){.x = 234, .y = 98, .a = 0.0f},
        (pos2_t){.x = 238, .y = 94, .a = 0.0f},
        (pos2_t){.x = 242, .y = 90, .a = 0.0f},
        (pos2_t){.x = 246, .y = 88, .a = 0.0f},
        (pos2_t){.x = 256, .y = 86, .a = 0.0f},
        (pos2_t){.x = 264, .y = 84, .a = 0.0f},
        (pos2_t){.x = 268, .y = 82, .a = 0.0f},
        (pos2_t){.x = 270, .y = 80, .a = 0.0f},
        (pos2_t){.x = 270, .y = 78, .a = 0.0f},
        (pos2_t){.x = 270, .y = 74, .a = 0.0f},
        (pos2_t){.x = 268, .y = 74, .a = 0.0f},
        (pos2_t){.x = 266, .y = 72, .a = 0.0f},
        (pos2_t){.x = 268, .y = 72, .a = 0.0f},
        (pos2_t){.x = 272, .y = 70, .a = 0.0f},
        (pos2_t){.x = 278, .y = 68, .a = 0.0f},
        (pos2_t){.x = 286, .y = 66, .a = 0.0f},
        (pos2_t){.x = 292, .y = 64, .a = 0.0f},
        (pos2_t){.x = 298, .y = 60, .a = 0.0f},
        (pos2_t){.x = 300, .y = 54, .a = 0.0f},
        (pos2_t){.x = 298, .y = 50, .a = 0.0f},
        (pos2_t){.x = 296, .y = 48, .a = 0.0f},
        (pos2_t){.x = 292, .y = 46, .a = 0.0f},
        (pos2_t){.x = 288, .y = 46, .a = 0.0f},
        (pos2_t){.x = 268, .y = 52, .a = 0.0f},
        (pos2_t){.x = 264, .y = 54, .a = 0.0f},
        (pos2_t){.x = 258, .y = 54, .a = 0.0f},
        (pos2_t){.x = 256, .y = 54, .a = 0.0f},
        (pos2_t){.x = 254, .y = 52, .a = 0.0f},
        (pos2_t){.x = 256, .y = 48, .a = 0.0f},
        (pos2_t){.x = 256, .y = 44, .a = 0.0f},
        (pos2_t){.x = 260, .y = 40, .a = 0.0f},
        (pos2_t){.x = 258, .y = 34, .a = 0.0f},
        (pos2_t){.x = 256, .y = 32, .a = 0.0f},
        (pos2_t){.x = 252, .y = 32, .a = 0.0f},
        (pos2_t){.x = 244, .y = 34, .a = 0.0f},
        (pos2_t){.x = 244, .y = 34, .a = 0.0f},
        (pos2_t){.x = 242, .y = 36, .a = 0.0f},
        (pos2_t){.x = 236, .y = 36, .a = 0.0f},
        (pos2_t){.x = 230, .y = 38, .a = 0.0f},
        (pos2_t){.x = 224, .y = 40, .a = 0.0f},
        (pos2_t){.x = 220, .y = 44, .a = 0.0f},
        (pos2_t){.x = 214, .y = 46, .a = 0.0f},
        (pos2_t){.x = 204, .y = 50, .a = 0.0f},
        (pos2_t){.x = 202, .y = 50, .a = 0.0f},
        (pos2_t){.x = 200, .y = 50, .a = 0.0f},
        (pos2_t){.x = 200, .y = 50, .a = 0.0f},
        (pos2_t){.x = 200, .y = 50, .a = 0.0f},
        (pos2_t){.x = 200, .y = 48, .a = 0.0f},
        (pos2_t){.x = 202, .y = 44, .a = 0.0f},
        (pos2_t){.x = 206, .y = 40, .a = 0.0f},
        (pos2_t){.x = 206, .y = 38, .a = 0.0f},
        (pos2_t){.x = 204, .y = 34, .a = 0.0f},
        (pos2_t){.x = 200, .y = 32, .a = 0.0f},
        (pos2_t){.x = 194, .y = 30, .a = 0.0f},
        (pos2_t){.x = 188, .y = 30, .a = 0.0f},
        (pos2_t){.x = 184, .y = 32, .a = 0.0f},
        (pos2_t){.x = 174, .y = 38, .a = 0.0f},
        (pos2_t){.x = 166, .y = 42, .a = 0.0f},
        (pos2_t){.x = 162, .y = 42, .a = 0.0f},
        (pos2_t){.x = 158, .y = 38, .a = 0.0f},
        (pos2_t){.x = 148, .y = 34, .a = 0.0f},
        (pos2_t){.x = 136, .y = 32, .a = 0.0f},
        (pos2_t){.x = 128, .y = 32, .a = 0.0f},
        (pos2_t){.x = 114, .y = 34, .a = 0.0f},
        (pos2_t){.x = 100, .y = 36, .a = 0.0f},
        (pos2_t){.x = 96, .y = 38, .a = 0.0f},
        (pos2_t){.x = 90, .y = 36, .a = 0.0f},
        (pos2_t){.x = 88, .y = 34, .a = 0.0f},
        (pos2_t){.x = 86, .y = 32, .a = 0.0f},
        (pos2_t){.x = 82, .y = 26, .a = 0.0f},
        (pos2_t){.x = 76, .y = 22, .a = 0.0f},
        (pos2_t){.x = 70, .y = 22, .a = 0.0f},
        (pos2_t){.x = 70, .y = 22, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps13);
    control_set_waypoints(&shared_ctrl, wps13, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps14[] = {
        (pos2_t){.x = 68, .y = 26, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps14);
    control_set_waypoints(&shared_ctrl, wps14, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps15[] = {
        (pos2_t){.x = 64, .y = 26, .a = 0.0f},
        (pos2_t){.x = 64, .y = 28, .a = 0.0f},
        (pos2_t){.x = 62, .y = 32, .a = 0.0f},
        (pos2_t){.x = 66, .y = 38, .a = 0.0f},
        (pos2_t){.x = 66, .y = 44, .a = 0.0f},
        (pos2_t){.x = 66, .y = 46, .a = 0.0f},
        (pos2_t){.x = 66, .y = 48, .a = 0.0f},
        (pos2_t){.x = 62, .y = 48, .a = 0.0f},
        (pos2_t){.x = 58, .y = 46, .a = 0.0f},
        (pos2_t){.x = 54, .y = 44, .a = 0.0f},
        (pos2_t){.x = 50, .y = 44, .a = 0.0f},
        (pos2_t){.x = 44, .y = 46, .a = 0.0f},
        (pos2_t){.x = 40, .y = 50, .a = 0.0f},
        (pos2_t){.x = 34, .y = 58, .a = 0.0f},
        (pos2_t){.x = 28, .y = 60, .a = 0.0f},
        (pos2_t){.x = 20, .y = 58, .a = 0.0f},
        (pos2_t){.x = 12, .y = 58, .a = 0.0f},
        (pos2_t){.x = 6, .y = 60, .a = 0.0f},
        (pos2_t){.x = 2, .y = 62, .a = 0.0f},
        (pos2_t){.x = 2, .y = 66, .a = 0.0f},
        (pos2_t){.x = 2, .y = 72, .a = 0.0f},
        (pos2_t){.x = 6, .y = 76, .a = 0.0f},
        (pos2_t){.x = 12, .y = 80, .a = 0.0f},
        (pos2_t){.x = 20, .y = 84, .a = 0.0f},
        (pos2_t){.x = 30, .y = 84, .a = 0.0f},
        (pos2_t){.x = 38, .y = 84, .a = 0.0f},
        (pos2_t){.x = 42, .y = 82, .a = 0.0f},
        (pos2_t){.x = 42, .y = 80, .a = 0.0f},
        (pos2_t){.x = 42, .y = 74, .a = 0.0f},
        (pos2_t){.x = 40, .y = 70, .a = 0.0f},
        (pos2_t){.x = 42, .y = 66, .a = 0.0f},
        (pos2_t){.x = 46, .y = 62, .a = 0.0f},
        (pos2_t){.x = 52, .y = 62, .a = 0.0f},
        (pos2_t){.x = 60, .y = 62, .a = 0.0f},
        (pos2_t){.x = 72, .y = 64, .a = 0.0f},
        (pos2_t){.x = 78, .y = 66, .a = 0.0f},
        (pos2_t){.x = 82, .y = 66, .a = 0.0f},
        (pos2_t){.x = 88, .y = 62, .a = 0.0f},
        (pos2_t){.x = 94, .y = 58, .a = 0.0f},
        (pos2_t){.x = 98, .y = 56, .a = 0.0f},
        (pos2_t){.x = 104, .y = 54, .a = 0.0f},
        (pos2_t){.x = 114, .y = 54, .a = 0.0f},
        (pos2_t){.x = 124, .y = 58, .a = 0.0f},
        (pos2_t){.x = 130, .y = 60, .a = 0.0f},
        (pos2_t){.x = 136, .y = 62, .a = 0.0f},
        (pos2_t){.x = 142, .y = 62, .a = 0.0f},
        (pos2_t){.x = 144, .y = 60, .a = 0.0f},
        (pos2_t){.x = 146, .y = 58, .a = 0.0f},
        (pos2_t){.x = 150, .y = 54, .a = 0.0f},
        (pos2_t){.x = 156, .y = 54, .a = 0.0f},
        (pos2_t){.x = 166, .y = 54, .a = 0.0f},
        (pos2_t){.x = 170, .y = 56, .a = 0.0f},
        (pos2_t){.x = 172, .y = 58, .a = 0.0f},
        (pos2_t){.x = 176, .y = 60, .a = 0.0f},
        (pos2_t){.x = 180, .y = 62, .a = 0.0f},
        (pos2_t){.x = 188, .y = 62, .a = 0.0f},
        (pos2_t){.x = 194, .y = 64, .a = 0.0f},
        (pos2_t){.x = 196, .y = 64, .a = 0.0f},
        (pos2_t){.x = 200, .y = 60, .a = 0.0f},
        (pos2_t){.x = 206, .y = 56, .a = 0.0f},
        (pos2_t){.x = 212, .y = 54, .a = 0.0f},
        (pos2_t){.x = 214, .y = 54, .a = 0.0f},
        (pos2_t){.x = 218, .y = 52, .a = 0.0f},
        (pos2_t){.x = 220, .y = 48, .a = 0.0f},
        (pos2_t){.x = 222, .y = 46, .a = 0.0f},
        (pos2_t){.x = 216, .y = 48, .a = 0.0f},
        (pos2_t){.x = 208, .y = 52, .a = 0.0f},
        (pos2_t){.x = 202, .y = 52, .a = 0.0f},
        (pos2_t){.x = 198, .y = 52, .a = 0.0f},
        (pos2_t){.x = 196, .y = 48, .a = 0.0f},
        (pos2_t){.x = 198, .y = 44, .a = 0.0f},
        (pos2_t){.x = 202, .y = 38, .a = 0.0f},
        (pos2_t){.x = 202, .y = 36, .a = 0.0f},
        (pos2_t){.x = 200, .y = 34, .a = 0.0f},
        (pos2_t){.x = 194, .y = 34, .a = 0.0f},
        (pos2_t){.x = 188, .y = 34, .a = 0.0f},
        (pos2_t){.x = 186, .y = 36, .a = 0.0f},
        (pos2_t){.x = 180, .y = 38, .a = 0.0f},
        (pos2_t){.x = 170, .y = 44, .a = 0.0f},
        (pos2_t){.x = 164, .y = 44, .a = 0.0f},
        (pos2_t){.x = 160, .y = 44, .a = 0.0f},
        (pos2_t){.x = 156, .y = 42, .a = 0.0f},
        (pos2_t){.x = 146, .y = 36, .a = 0.0f},
        (pos2_t){.x = 140, .y = 36, .a = 0.0f},
        (pos2_t){.x = 132, .y = 34, .a = 0.0f},
        (pos2_t){.x = 124, .y = 36, .a = 0.0f},
        (pos2_t){.x = 114, .y = 38, .a = 0.0f},
        (pos2_t){.x = 102, .y = 40, .a = 0.0f},
        (pos2_t){.x = 96, .y = 40, .a = 0.0f},
        (pos2_t){.x = 90, .y = 40, .a = 0.0f},
        (pos2_t){.x = 88, .y = 38, .a = 0.0f},
        (pos2_t){.x = 84, .y = 36, .a = 0.0f},
        (pos2_t){.x = 82, .y = 34, .a = 0.0f},
        (pos2_t){.x = 80, .y = 28, .a = 0.0f},
        (pos2_t){.x = 74, .y = 26, .a = 0.0f},
        (pos2_t){.x = 70, .y = 24, .a = 0.0f},
        (pos2_t){.x = 68, .y = 26, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps15);
    control_set_waypoints(&shared_ctrl, wps15, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps16[] = {
        (pos2_t){.x = 200, .y = 36, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps16);
    control_set_waypoints(&shared_ctrl, wps16, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps17[] = {
        (pos2_t){.x = 200, .y = 38, .a = 0.0f},
        (pos2_t){.x = 198, .y = 42, .a = 0.0f},
        (pos2_t){.x = 198, .y = 44, .a = 0.0f},
        (pos2_t){.x = 196, .y = 44, .a = 0.0f},
        (pos2_t){.x = 196, .y = 44, .a = 0.0f},
        (pos2_t){.x = 196, .y = 42, .a = 0.0f},
        (pos2_t){.x = 196, .y = 42, .a = 0.0f},
        (pos2_t){.x = 196, .y = 40, .a = 0.0f},
        (pos2_t){.x = 198, .y = 38, .a = 0.0f},
        (pos2_t){.x = 198, .y = 36, .a = 0.0f},
        (pos2_t){.x = 200, .y = 36, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps17);
    control_set_waypoints(&shared_ctrl, wps17, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps18[] = {
        (pos2_t){.x = 28, .y = 62, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps18);
    control_set_waypoints(&shared_ctrl, wps18, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps19[] = {
        (pos2_t){.x = 32, .y = 62, .a = 0.0f},
        (pos2_t){.x = 32, .y = 64, .a = 0.0f},
        (pos2_t){.x = 32, .y = 64, .a = 0.0f},
        (pos2_t){.x = 28, .y = 66, .a = 0.0f},
        (pos2_t){.x = 24, .y = 64, .a = 0.0f},
        (pos2_t){.x = 24, .y = 64, .a = 0.0f},
        (pos2_t){.x = 24, .y = 64, .a = 0.0f},
        (pos2_t){.x = 24, .y = 64, .a = 0.0f},
        (pos2_t){.x = 24, .y = 62, .a = 0.0f},
        (pos2_t){.x = 24, .y = 62, .a = 0.0f},
        (pos2_t){.x = 24, .y = 62, .a = 0.0f},
        (pos2_t){.x = 26, .y = 62, .a = 0.0f},
        (pos2_t){.x = 28, .y = 62, .a = 0.0f},
        (pos2_t){.x = 28, .y = 62, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps19);
    control_set_waypoints(&shared_ctrl, wps19, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps20[] = {
        (pos2_t){.x = 248, .y = 36, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps20);
    control_set_waypoints(&shared_ctrl, wps20, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps21[] = {
        (pos2_t){.x = 246, .y = 36, .a = 0.0f},
        (pos2_t){.x = 246, .y = 36, .a = 0.0f},
        (pos2_t){.x = 246, .y = 36, .a = 0.0f},
        (pos2_t){.x = 244, .y = 38, .a = 0.0f},
        (pos2_t){.x = 244, .y = 38, .a = 0.0f},
        (pos2_t){.x = 246, .y = 38, .a = 0.0f},
        (pos2_t){.x = 248, .y = 42, .a = 0.0f},
        (pos2_t){.x = 248, .y = 46, .a = 0.0f},
        (pos2_t){.x = 246, .y = 50, .a = 0.0f},
        (pos2_t){.x = 236, .y = 56, .a = 0.0f},
        (pos2_t){.x = 230, .y = 62, .a = 0.0f},
        (pos2_t){.x = 230, .y = 66, .a = 0.0f},
        (pos2_t){.x = 230, .y = 68, .a = 0.0f},
        (pos2_t){.x = 232, .y = 70, .a = 0.0f},
        (pos2_t){.x = 234, .y = 72, .a = 0.0f},
        (pos2_t){.x = 240, .y = 74, .a = 0.0f},
        (pos2_t){.x = 242, .y = 76, .a = 0.0f},
        (pos2_t){.x = 244, .y = 76, .a = 0.0f},
        (pos2_t){.x = 250, .y = 76, .a = 0.0f},
        (pos2_t){.x = 262, .y = 74, .a = 0.0f},
        (pos2_t){.x = 264, .y = 72, .a = 0.0f},
        (pos2_t){.x = 266, .y = 70, .a = 0.0f},
        (pos2_t){.x = 270, .y = 68, .a = 0.0f},
        (pos2_t){.x = 278, .y = 66, .a = 0.0f},
        (pos2_t){.x = 292, .y = 62, .a = 0.0f},
        (pos2_t){.x = 296, .y = 58, .a = 0.0f},
        (pos2_t){.x = 298, .y = 54, .a = 0.0f},
        (pos2_t){.x = 296, .y = 52, .a = 0.0f},
        (pos2_t){.x = 296, .y = 50, .a = 0.0f},
        (pos2_t){.x = 292, .y = 48, .a = 0.0f},
        (pos2_t){.x = 288, .y = 48, .a = 0.0f},
        (pos2_t){.x = 284, .y = 50, .a = 0.0f},
        (pos2_t){.x = 278, .y = 52, .a = 0.0f},
        (pos2_t){.x = 270, .y = 56, .a = 0.0f},
        (pos2_t){.x = 260, .y = 58, .a = 0.0f},
        (pos2_t){.x = 256, .y = 58, .a = 0.0f},
        (pos2_t){.x = 254, .y = 56, .a = 0.0f},
        (pos2_t){.x = 252, .y = 54, .a = 0.0f},
        (pos2_t){.x = 252, .y = 52, .a = 0.0f},
        (pos2_t){.x = 252, .y = 48, .a = 0.0f},
        (pos2_t){.x = 254, .y = 44, .a = 0.0f},
        (pos2_t){.x = 256, .y = 40, .a = 0.0f},
        (pos2_t){.x = 256, .y = 38, .a = 0.0f},
        (pos2_t){.x = 256, .y = 36, .a = 0.0f},
        (pos2_t){.x = 252, .y = 34, .a = 0.0f},
        (pos2_t){.x = 248, .y = 36, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps21);
    control_set_waypoints(&shared_ctrl, wps21, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps22[] = {
        (pos2_t){.x = 282, .y = 56, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps22);
    control_set_waypoints(&shared_ctrl, wps22, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps23[] = {
        (pos2_t){.x = 280, .y = 58, .a = 0.0f},
        (pos2_t){.x = 282, .y = 60, .a = 0.0f},
        (pos2_t){.x = 284, .y = 60, .a = 0.0f},
        (pos2_t){.x = 286, .y = 58, .a = 0.0f},
        (pos2_t){.x = 286, .y = 58, .a = 0.0f},
        (pos2_t){.x = 286, .y = 56, .a = 0.0f},
        (pos2_t){.x = 286, .y = 56, .a = 0.0f},
        (pos2_t){.x = 286, .y = 56, .a = 0.0f},
        (pos2_t){.x = 284, .y = 56, .a = 0.0f},
        (pos2_t){.x = 284, .y = 56, .a = 0.0f},
        (pos2_t){.x = 284, .y = 56, .a = 0.0f},
        (pos2_t){.x = 282, .y = 56, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps23);
    control_set_waypoints(&shared_ctrl, wps23, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps24[] = {
        (pos2_t){.x = 294, .y = 54, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps24);
    control_set_waypoints(&shared_ctrl, wps24, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps25[] = {
        (pos2_t){.x = 294, .y = 56, .a = 0.0f},
        (pos2_t){.x = 294, .y = 60, .a = 0.0f},
        (pos2_t){.x = 292, .y = 62, .a = 0.0f},
        (pos2_t){.x = 288, .y = 62, .a = 0.0f},
        (pos2_t){.x = 286, .y = 60, .a = 0.0f},
        (pos2_t){.x = 288, .y = 58, .a = 0.0f},
        (pos2_t){.x = 292, .y = 56, .a = 0.0f},
        (pos2_t){.x = 294, .y = 54, .a = 0.0f},
        (pos2_t){.x = 294, .y = 54, .a = 0.0f},
        (pos2_t){.x = 294, .y = 54, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps25);
    control_set_waypoints(&shared_ctrl, wps25, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps26[] = {
        (pos2_t){.x = 236, .y = 38, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps26);
    control_set_waypoints(&shared_ctrl, wps26, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps27[] = {
        (pos2_t){.x = 232, .y = 40, .a = 0.0f},
        (pos2_t){.x = 228, .y = 44, .a = 0.0f},
        (pos2_t){.x = 224, .y = 48, .a = 0.0f},
        (pos2_t){.x = 218, .y = 54, .a = 0.0f},
        (pos2_t){.x = 212, .y = 56, .a = 0.0f},
        (pos2_t){.x = 208, .y = 58, .a = 0.0f},
        (pos2_t){.x = 198, .y = 66, .a = 0.0f},
        (pos2_t){.x = 202, .y = 68, .a = 0.0f},
        (pos2_t){.x = 208, .y = 72, .a = 0.0f},
        (pos2_t){.x = 214, .y = 76, .a = 0.0f},
        (pos2_t){.x = 216, .y = 76, .a = 0.0f},
        (pos2_t){.x = 218, .y = 76, .a = 0.0f},
        (pos2_t){.x = 222, .y = 76, .a = 0.0f},
        (pos2_t){.x = 226, .y = 74, .a = 0.0f},
        (pos2_t){.x = 228, .y = 72, .a = 0.0f},
        (pos2_t){.x = 230, .y = 72, .a = 0.0f},
        (pos2_t){.x = 228, .y = 70, .a = 0.0f},
        (pos2_t){.x = 228, .y = 68, .a = 0.0f},
        (pos2_t){.x = 228, .y = 64, .a = 0.0f},
        (pos2_t){.x = 230, .y = 60, .a = 0.0f},
        (pos2_t){.x = 236, .y = 54, .a = 0.0f},
        (pos2_t){.x = 246, .y = 46, .a = 0.0f},
        (pos2_t){.x = 246, .y = 44, .a = 0.0f},
        (pos2_t){.x = 246, .y = 42, .a = 0.0f},
        (pos2_t){.x = 242, .y = 38, .a = 0.0f},
        (pos2_t){.x = 236, .y = 38, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps27);
    control_set_waypoints(&shared_ctrl, wps27, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps28[] = {
        (pos2_t){.x = 244, .y = 42, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps28);
    control_set_waypoints(&shared_ctrl, wps28, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps29[] = {
        (pos2_t){.x = 244, .y = 42, .a = 0.0f},
        (pos2_t){.x = 244, .y = 44, .a = 0.0f},
        (pos2_t){.x = 244, .y = 44, .a = 0.0f},
        (pos2_t){.x = 244, .y = 44, .a = 0.0f},
        (pos2_t){.x = 244, .y = 44, .a = 0.0f},
        (pos2_t){.x = 244, .y = 44, .a = 0.0f},
        (pos2_t){.x = 244, .y = 46, .a = 0.0f},
        (pos2_t){.x = 242, .y = 48, .a = 0.0f},
        (pos2_t){.x = 242, .y = 48, .a = 0.0f},
        (pos2_t){.x = 240, .y = 48, .a = 0.0f},
        (pos2_t){.x = 240, .y = 46, .a = 0.0f},
        (pos2_t){.x = 240, .y = 46, .a = 0.0f},
        (pos2_t){.x = 240, .y = 44, .a = 0.0f},
        (pos2_t){.x = 242, .y = 42, .a = 0.0f},
        (pos2_t){.x = 242, .y = 42, .a = 0.0f},
        (pos2_t){.x = 242, .y = 42, .a = 0.0f},
        (pos2_t){.x = 244, .y = 42, .a = 0.0f},
        (pos2_t){.x = 244, .y = 42, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps29);
    control_set_waypoints(&shared_ctrl, wps29, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps30[] = {
        (pos2_t){.x = 166, .y = 58, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps30);
    control_set_waypoints(&shared_ctrl, wps30, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps31[] = {
        (pos2_t){.x = 170, .y = 60, .a = 0.0f},
        (pos2_t){.x = 170, .y = 60, .a = 0.0f},
        (pos2_t){.x = 170, .y = 60, .a = 0.0f},
        (pos2_t){.x = 160, .y = 64, .a = 0.0f},
        (pos2_t){.x = 152, .y = 68, .a = 0.0f},
        (pos2_t){.x = 148, .y = 72, .a = 0.0f},
        (pos2_t){.x = 142, .y = 76, .a = 0.0f},
        (pos2_t){.x = 138, .y = 78, .a = 0.0f},
        (pos2_t){.x = 136, .y = 76, .a = 0.0f},
        (pos2_t){.x = 134, .y = 70, .a = 0.0f},
        (pos2_t){.x = 132, .y = 70, .a = 0.0f},
        (pos2_t){.x = 130, .y = 70, .a = 0.0f},
        (pos2_t){.x = 130, .y = 72, .a = 0.0f},
        (pos2_t){.x = 130, .y = 76, .a = 0.0f},
        (pos2_t){.x = 130, .y = 78, .a = 0.0f},
        (pos2_t){.x = 130, .y = 78, .a = 0.0f},
        (pos2_t){.x = 130, .y = 78, .a = 0.0f},
        (pos2_t){.x = 130, .y = 78, .a = 0.0f},
        (pos2_t){.x = 130, .y = 78, .a = 0.0f},
        (pos2_t){.x = 128, .y = 78, .a = 0.0f},
        (pos2_t){.x = 128, .y = 76, .a = 0.0f},
        (pos2_t){.x = 124, .y = 76, .a = 0.0f},
        (pos2_t){.x = 120, .y = 76, .a = 0.0f},
        (pos2_t){.x = 114, .y = 80, .a = 0.0f},
        (pos2_t){.x = 112, .y = 80, .a = 0.0f},
        (pos2_t){.x = 108, .y = 80, .a = 0.0f},
        (pos2_t){.x = 100, .y = 82, .a = 0.0f},
        (pos2_t){.x = 92, .y = 84, .a = 0.0f},
        (pos2_t){.x = 90, .y = 84, .a = 0.0f},
        (pos2_t){.x = 84, .y = 80, .a = 0.0f},
        (pos2_t){.x = 76, .y = 78, .a = 0.0f},
        (pos2_t){.x = 70, .y = 78, .a = 0.0f},
        (pos2_t){.x = 64, .y = 78, .a = 0.0f},
        (pos2_t){.x = 60, .y = 82, .a = 0.0f},
        (pos2_t){.x = 58, .y = 84, .a = 0.0f},
        (pos2_t){.x = 58, .y = 88, .a = 0.0f},
        (pos2_t){.x = 60, .y = 88, .a = 0.0f},
        (pos2_t){.x = 60, .y = 90, .a = 0.0f},
        (pos2_t){.x = 58, .y = 90, .a = 0.0f},
        (pos2_t){.x = 56, .y = 88, .a = 0.0f},
        (pos2_t){.x = 54, .y = 86, .a = 0.0f},
        (pos2_t){.x = 52, .y = 88, .a = 0.0f},
        (pos2_t){.x = 48, .y = 90, .a = 0.0f},
        (pos2_t){.x = 46, .y = 92, .a = 0.0f},
        (pos2_t){.x = 42, .y = 92, .a = 0.0f},
        (pos2_t){.x = 40, .y = 92, .a = 0.0f},
        (pos2_t){.x = 38, .y = 92, .a = 0.0f},
        (pos2_t){.x = 38, .y = 90, .a = 0.0f},
        (pos2_t){.x = 40, .y = 88, .a = 0.0f},
        (pos2_t){.x = 44, .y = 86, .a = 0.0f},
        (pos2_t){.x = 44, .y = 82, .a = 0.0f},
        (pos2_t){.x = 46, .y = 78, .a = 0.0f},
        (pos2_t){.x = 44, .y = 74, .a = 0.0f},
        (pos2_t){.x = 44, .y = 70, .a = 0.0f},
        (pos2_t){.x = 46, .y = 68, .a = 0.0f},
        (pos2_t){.x = 46, .y = 66, .a = 0.0f},
        (pos2_t){.x = 46, .y = 66, .a = 0.0f},
        (pos2_t){.x = 48, .y = 68, .a = 0.0f},
        (pos2_t){.x = 54, .y = 68, .a = 0.0f},
        (pos2_t){.x = 58, .y = 68, .a = 0.0f},
        (pos2_t){.x = 60, .y = 66, .a = 0.0f},
        (pos2_t){.x = 60, .y = 66, .a = 0.0f},
        (pos2_t){.x = 60, .y = 66, .a = 0.0f},
        (pos2_t){.x = 60, .y = 66, .a = 0.0f},
        (pos2_t){.x = 64, .y = 66, .a = 0.0f},
        (pos2_t){.x = 70, .y = 68, .a = 0.0f},
        (pos2_t){.x = 80, .y = 70, .a = 0.0f},
        (pos2_t){.x = 84, .y = 70, .a = 0.0f},
        (pos2_t){.x = 90, .y = 66, .a = 0.0f},
        (pos2_t){.x = 92, .y = 64, .a = 0.0f},
        (pos2_t){.x = 94, .y = 64, .a = 0.0f},
        (pos2_t){.x = 92, .y = 64, .a = 0.0f},
        (pos2_t){.x = 94, .y = 62, .a = 0.0f},
        (pos2_t){.x = 98, .y = 60, .a = 0.0f},
        (pos2_t){.x = 102, .y = 58, .a = 0.0f},
        (pos2_t){.x = 108, .y = 58, .a = 0.0f},
        (pos2_t){.x = 114, .y = 58, .a = 0.0f},
        (pos2_t){.x = 122, .y = 62, .a = 0.0f},
        (pos2_t){.x = 128, .y = 64, .a = 0.0f},
        (pos2_t){.x = 136, .y = 66, .a = 0.0f},
        (pos2_t){.x = 144, .y = 64, .a = 0.0f},
        (pos2_t){.x = 148, .y = 62, .a = 0.0f},
        (pos2_t){.x = 150, .y = 58, .a = 0.0f},
        (pos2_t){.x = 152, .y = 58, .a = 0.0f},
        (pos2_t){.x = 152, .y = 58, .a = 0.0f},
        (pos2_t){.x = 152, .y = 58, .a = 0.0f},
        (pos2_t){.x = 152, .y = 58, .a = 0.0f},
        (pos2_t){.x = 152, .y = 58, .a = 0.0f},
        (pos2_t){.x = 154, .y = 58, .a = 0.0f},
        (pos2_t){.x = 160, .y = 58, .a = 0.0f},
        (pos2_t){.x = 166, .y = 58, .a = 0.0f},
        (pos2_t){.x = 166, .y = 58, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps31);
    control_set_waypoints(&shared_ctrl, wps31, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps32[] = {
        (pos2_t){.x = 66, .y = 68, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps32);
    control_set_waypoints(&shared_ctrl, wps32, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps33[] = {
        (pos2_t){.x = 66, .y = 70, .a = 0.0f},
        (pos2_t){.x = 66, .y = 70, .a = 0.0f},
        (pos2_t){.x = 66, .y = 70, .a = 0.0f},
        (pos2_t){.x = 64, .y = 70, .a = 0.0f},
        (pos2_t){.x = 62, .y = 68, .a = 0.0f},
        (pos2_t){.x = 64, .y = 66, .a = 0.0f},
        (pos2_t){.x = 66, .y = 68, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps33);
    control_set_waypoints(&shared_ctrl, wps33, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps34[] = {
        (pos2_t){.x = 186, .y = 66, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps34);
    control_set_waypoints(&shared_ctrl, wps34, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps35[] = {
        (pos2_t){.x = 202, .y = 72, .a = 0.0f},
        (pos2_t){.x = 206, .y = 76, .a = 0.0f},
        (pos2_t){.x = 210, .y = 78, .a = 0.0f},
        (pos2_t){.x = 214, .y = 78, .a = 0.0f},
        (pos2_t){.x = 216, .y = 80, .a = 0.0f},
        (pos2_t){.x = 216, .y = 80, .a = 0.0f},
        (pos2_t){.x = 214, .y = 82, .a = 0.0f},
        (pos2_t){.x = 214, .y = 82, .a = 0.0f},
        (pos2_t){.x = 212, .y = 82, .a = 0.0f},
        (pos2_t){.x = 212, .y = 82, .a = 0.0f},
        (pos2_t){.x = 204, .y = 82, .a = 0.0f},
        (pos2_t){.x = 194, .y = 82, .a = 0.0f},
        (pos2_t){.x = 188, .y = 80, .a = 0.0f},
        (pos2_t){.x = 180, .y = 78, .a = 0.0f},
        (pos2_t){.x = 172, .y = 78, .a = 0.0f},
        (pos2_t){.x = 164, .y = 80, .a = 0.0f},
        (pos2_t){.x = 158, .y = 80, .a = 0.0f},
        (pos2_t){.x = 148, .y = 86, .a = 0.0f},
        (pos2_t){.x = 140, .y = 88, .a = 0.0f},
        (pos2_t){.x = 138, .y = 90, .a = 0.0f},
        (pos2_t){.x = 138, .y = 86, .a = 0.0f},
        (pos2_t){.x = 138, .y = 82, .a = 0.0f},
        (pos2_t){.x = 138, .y = 80, .a = 0.0f},
        (pos2_t){.x = 138, .y = 80, .a = 0.0f},
        (pos2_t){.x = 142, .y = 78, .a = 0.0f},
        (pos2_t){.x = 146, .y = 76, .a = 0.0f},
        (pos2_t){.x = 150, .y = 74, .a = 0.0f},
        (pos2_t){.x = 160, .y = 68, .a = 0.0f},
        (pos2_t){.x = 166, .y = 66, .a = 0.0f},
        (pos2_t){.x = 172, .y = 66, .a = 0.0f},
        (pos2_t){.x = 186, .y = 66, .a = 0.0f},
        (pos2_t){.x = 186, .y = 66, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps35);
    control_set_waypoints(&shared_ctrl, wps35, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps36[] = {
        (pos2_t){.x = 134, .y = 74, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps36);
    control_set_waypoints(&shared_ctrl, wps36, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps37[] = {
        (pos2_t){.x = 136, .y = 86, .a = 0.0f},
        (pos2_t){.x = 136, .y = 90, .a = 0.0f},
        (pos2_t){.x = 136, .y = 88, .a = 0.0f},
        (pos2_t){.x = 134, .y = 76, .a = 0.0f},
        (pos2_t){.x = 132, .y = 72, .a = 0.0f},
        (pos2_t){.x = 134, .y = 74, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps37);
    control_set_waypoints(&shared_ctrl, wps37, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps38[] = {
        (pos2_t){.x = 128, .y = 78, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps38);
    control_set_waypoints(&shared_ctrl, wps38, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps39[] = {
        (pos2_t){.x = 128, .y = 78, .a = 0.0f},
        (pos2_t){.x = 130, .y = 78, .a = 0.0f},
        (pos2_t){.x = 130, .y = 78, .a = 0.0f},
        (pos2_t){.x = 130, .y = 80, .a = 0.0f},
        (pos2_t){.x = 132, .y = 90, .a = 0.0f},
        (pos2_t){.x = 130, .y = 90, .a = 0.0f},
        (pos2_t){.x = 128, .y = 90, .a = 0.0f},
        (pos2_t){.x = 126, .y = 88, .a = 0.0f},
        (pos2_t){.x = 124, .y = 82, .a = 0.0f},
        (pos2_t){.x = 122, .y = 82, .a = 0.0f},
        (pos2_t){.x = 120, .y = 82, .a = 0.0f},
        (pos2_t){.x = 120, .y = 90, .a = 0.0f},
        (pos2_t){.x = 120, .y = 92, .a = 0.0f},
        (pos2_t){.x = 118, .y = 90, .a = 0.0f},
        (pos2_t){.x = 114, .y = 90, .a = 0.0f},
        (pos2_t){.x = 108, .y = 88, .a = 0.0f},
        (pos2_t){.x = 104, .y = 86, .a = 0.0f},
        (pos2_t){.x = 100, .y = 88, .a = 0.0f},
        (pos2_t){.x = 96, .y = 90, .a = 0.0f},
        (pos2_t){.x = 94, .y = 94, .a = 0.0f},
        (pos2_t){.x = 92, .y = 96, .a = 0.0f},
        (pos2_t){.x = 88, .y = 96, .a = 0.0f},
        (pos2_t){.x = 82, .y = 92, .a = 0.0f},
        (pos2_t){.x = 78, .y = 90, .a = 0.0f},
        (pos2_t){.x = 74, .y = 90, .a = 0.0f},
        (pos2_t){.x = 68, .y = 90, .a = 0.0f},
        (pos2_t){.x = 66, .y = 92, .a = 0.0f},
        (pos2_t){.x = 64, .y = 92, .a = 0.0f},
        (pos2_t){.x = 64, .y = 90, .a = 0.0f},
        (pos2_t){.x = 62, .y = 86, .a = 0.0f},
        (pos2_t){.x = 62, .y = 84, .a = 0.0f},
        (pos2_t){.x = 66, .y = 82, .a = 0.0f},
        (pos2_t){.x = 70, .y = 80, .a = 0.0f},
        (pos2_t){.x = 74, .y = 80, .a = 0.0f},
        (pos2_t){.x = 82, .y = 84, .a = 0.0f},
        (pos2_t){.x = 90, .y = 86, .a = 0.0f},
        (pos2_t){.x = 96, .y = 86, .a = 0.0f},
        (pos2_t){.x = 100, .y = 86, .a = 0.0f},
        (pos2_t){.x = 108, .y = 84, .a = 0.0f},
        (pos2_t){.x = 112, .y = 84, .a = 0.0f},
        (pos2_t){.x = 118, .y = 82, .a = 0.0f},
        (pos2_t){.x = 122, .y = 80, .a = 0.0f},
        (pos2_t){.x = 126, .y = 78, .a = 0.0f},
        (pos2_t){.x = 126, .y = 78, .a = 0.0f},
        (pos2_t){.x = 126, .y = 78, .a = 0.0f},
        (pos2_t){.x = 128, .y = 78, .a = 0.0f},
        (pos2_t){.x = 128, .y = 78, .a = 0.0f},
        (pos2_t){.x = 128, .y = 78, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps39);
    control_set_waypoints(&shared_ctrl, wps39, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps40[] = {
        (pos2_t){.x = 124, .y = 86, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps40);
    control_set_waypoints(&shared_ctrl, wps40, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps41[] = {
        (pos2_t){.x = 126, .y = 96, .a = 0.0f},
        (pos2_t){.x = 128, .y = 104, .a = 0.0f},
        (pos2_t){.x = 130, .y = 104, .a = 0.0f},
        (pos2_t){.x = 128, .y = 104, .a = 0.0f},
        (pos2_t){.x = 126, .y = 100, .a = 0.0f},
        (pos2_t){.x = 122, .y = 92, .a = 0.0f},
        (pos2_t){.x = 122, .y = 84, .a = 0.0f},
        (pos2_t){.x = 122, .y = 84, .a = 0.0f},
        (pos2_t){.x = 122, .y = 84, .a = 0.0f},
        (pos2_t){.x = 122, .y = 84, .a = 0.0f},
        (pos2_t){.x = 124, .y = 86, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps41);
    control_set_waypoints(&shared_ctrl, wps41, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps42[] = {
        (pos2_t){.x = 132, .y = 98, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps42);
    control_set_waypoints(&shared_ctrl, wps42, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps43[] = {
        (pos2_t){.x = 132, .y = 104, .a = 0.0f},
        (pos2_t){.x = 132, .y = 104, .a = 0.0f},
        (pos2_t){.x = 130, .y = 104, .a = 0.0f},
        (pos2_t){.x = 128, .y = 94, .a = 0.0f},
        (pos2_t){.x = 128, .y = 94, .a = 0.0f},
        (pos2_t){.x = 128, .y = 94, .a = 0.0f},
        (pos2_t){.x = 130, .y = 94, .a = 0.0f},
        (pos2_t){.x = 130, .y = 94, .a = 0.0f},
        (pos2_t){.x = 130, .y = 94, .a = 0.0f},
        (pos2_t){.x = 132, .y = 94, .a = 0.0f},
        (pos2_t){.x = 132, .y = 94, .a = 0.0f},
        (pos2_t){.x = 132, .y = 98, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps43);
    control_set_waypoints(&shared_ctrl, wps43, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps44[] = {
        (pos2_t){.x = 118, .y = 96, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps44);
    control_set_waypoints(&shared_ctrl, wps44, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps45[] = {
        (pos2_t){.x = 120, .y = 98, .a = 0.0f},
        (pos2_t){.x = 120, .y = 98, .a = 0.0f},
        (pos2_t){.x = 120, .y = 98, .a = 0.0f},
        (pos2_t){.x = 118, .y = 98, .a = 0.0f},
        (pos2_t){.x = 116, .y = 98, .a = 0.0f},
        (pos2_t){.x = 114, .y = 98, .a = 0.0f},
        (pos2_t){.x = 112, .y = 100, .a = 0.0f},
        (pos2_t){.x = 114, .y = 104, .a = 0.0f},
        (pos2_t){.x = 116, .y = 108, .a = 0.0f},
        (pos2_t){.x = 120, .y = 112, .a = 0.0f},
        (pos2_t){.x = 124, .y = 114, .a = 0.0f},
        (pos2_t){.x = 126, .y = 112, .a = 0.0f},
        (pos2_t){.x = 128, .y = 112, .a = 0.0f},
        (pos2_t){.x = 128, .y = 112, .a = 0.0f},
        (pos2_t){.x = 128, .y = 112, .a = 0.0f},
        (pos2_t){.x = 126, .y = 114, .a = 0.0f},
        (pos2_t){.x = 124, .y = 114, .a = 0.0f},
        (pos2_t){.x = 120, .y = 114, .a = 0.0f},
        (pos2_t){.x = 116, .y = 110, .a = 0.0f},
        (pos2_t){.x = 112, .y = 106, .a = 0.0f},
        (pos2_t){.x = 110, .y = 100, .a = 0.0f},
        (pos2_t){.x = 112, .y = 98, .a = 0.0f},
        (pos2_t){.x = 114, .y = 96, .a = 0.0f},
        (pos2_t){.x = 118, .y = 96, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps45);
    control_set_waypoints(&shared_ctrl, wps45, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps46[] = {
        (pos2_t){.x = 226, .y = 76, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps46);
    control_set_waypoints(&shared_ctrl, wps46, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps47[] = {
        (pos2_t){.x = 222, .y = 78, .a = 0.0f},
        (pos2_t){.x = 218, .y = 82, .a = 0.0f},
        (pos2_t){.x = 214, .y = 86, .a = 0.0f},
        (pos2_t){.x = 210, .y = 86, .a = 0.0f},
        (pos2_t){.x = 204, .y = 86, .a = 0.0f},
        (pos2_t){.x = 194, .y = 86, .a = 0.0f},
        (pos2_t){.x = 178, .y = 82, .a = 0.0f},
        (pos2_t){.x = 168, .y = 82, .a = 0.0f},
        (pos2_t){.x = 158, .y = 84, .a = 0.0f},
        (pos2_t){.x = 150, .y = 88, .a = 0.0f},
        (pos2_t){.x = 144, .y = 90, .a = 0.0f},
        (pos2_t){.x = 140, .y = 92, .a = 0.0f},
        (pos2_t){.x = 136, .y = 92, .a = 0.0f},
        (pos2_t){.x = 136, .y = 96, .a = 0.0f},
        (pos2_t){.x = 138, .y = 96, .a = 0.0f},
        (pos2_t){.x = 142, .y = 96, .a = 0.0f},
        (pos2_t){.x = 144, .y = 96, .a = 0.0f},
        (pos2_t){.x = 148, .y = 98, .a = 0.0f},
        (pos2_t){.x = 148, .y = 102, .a = 0.0f},
        (pos2_t){.x = 148, .y = 102, .a = 0.0f},
        (pos2_t){.x = 148, .y = 104, .a = 0.0f},
        (pos2_t){.x = 148, .y = 102, .a = 0.0f},
        (pos2_t){.x = 146, .y = 98, .a = 0.0f},
        (pos2_t){.x = 142, .y = 96, .a = 0.0f},
        (pos2_t){.x = 140, .y = 96, .a = 0.0f},
        (pos2_t){.x = 136, .y = 98, .a = 0.0f},
        (pos2_t){.x = 136, .y = 100, .a = 0.0f},
        (pos2_t){.x = 136, .y = 100, .a = 0.0f},
        (pos2_t){.x = 136, .y = 100, .a = 0.0f},
        (pos2_t){.x = 136, .y = 100, .a = 0.0f},
        (pos2_t){.x = 138, .y = 100, .a = 0.0f},
        (pos2_t){.x = 142, .y = 98, .a = 0.0f},
        (pos2_t){.x = 144, .y = 98, .a = 0.0f},
        (pos2_t){.x = 146, .y = 98, .a = 0.0f},
        (pos2_t){.x = 146, .y = 102, .a = 0.0f},
        (pos2_t){.x = 144, .y = 106, .a = 0.0f},
        (pos2_t){.x = 140, .y = 110, .a = 0.0f},
        (pos2_t){.x = 136, .y = 110, .a = 0.0f},
        (pos2_t){.x = 134, .y = 110, .a = 0.0f},
        (pos2_t){.x = 134, .y = 110, .a = 0.0f},
        (pos2_t){.x = 134, .y = 112, .a = 0.0f},
        (pos2_t){.x = 134, .y = 112, .a = 0.0f},
        (pos2_t){.x = 138, .y = 114, .a = 0.0f},
        (pos2_t){.x = 140, .y = 112, .a = 0.0f},
        (pos2_t){.x = 144, .y = 112, .a = 0.0f},
        (pos2_t){.x = 148, .y = 106, .a = 0.0f},
        (pos2_t){.x = 148, .y = 104, .a = 0.0f},
        (pos2_t){.x = 148, .y = 104, .a = 0.0f},
        (pos2_t){.x = 148, .y = 104, .a = 0.0f},
        (pos2_t){.x = 148, .y = 104, .a = 0.0f},
        (pos2_t){.x = 148, .y = 106, .a = 0.0f},
        (pos2_t){.x = 158, .y = 106, .a = 0.0f},
        (pos2_t){.x = 166, .y = 106, .a = 0.0f},
        (pos2_t){.x = 172, .y = 108, .a = 0.0f},
        (pos2_t){.x = 176, .y = 108, .a = 0.0f},
        (pos2_t){.x = 176, .y = 106, .a = 0.0f},
        (pos2_t){.x = 182, .y = 100, .a = 0.0f},
        (pos2_t){.x = 186, .y = 94, .a = 0.0f},
        (pos2_t){.x = 190, .y = 92, .a = 0.0f},
        (pos2_t){.x = 194, .y = 92, .a = 0.0f},
        (pos2_t){.x = 200, .y = 94, .a = 0.0f},
        (pos2_t){.x = 204, .y = 96, .a = 0.0f},
        (pos2_t){.x = 208, .y = 100, .a = 0.0f},
        (pos2_t){.x = 210, .y = 104, .a = 0.0f},
        (pos2_t){.x = 210, .y = 108, .a = 0.0f},
        (pos2_t){.x = 210, .y = 116, .a = 0.0f},
        (pos2_t){.x = 208, .y = 122, .a = 0.0f},
        (pos2_t){.x = 206, .y = 124, .a = 0.0f},
        (pos2_t){.x = 210, .y = 126, .a = 0.0f},
        (pos2_t){.x = 214, .y = 126, .a = 0.0f},
        (pos2_t){.x = 218, .y = 124, .a = 0.0f},
        (pos2_t){.x = 222, .y = 124, .a = 0.0f},
        (pos2_t){.x = 224, .y = 124, .a = 0.0f},
        (pos2_t){.x = 224, .y = 124, .a = 0.0f},
        (pos2_t){.x = 224, .y = 124, .a = 0.0f},
        (pos2_t){.x = 228, .y = 122, .a = 0.0f},
        (pos2_t){.x = 232, .y = 118, .a = 0.0f},
        (pos2_t){.x = 234, .y = 116, .a = 0.0f},
        (pos2_t){.x = 234, .y = 112, .a = 0.0f},
        (pos2_t){.x = 232, .y = 110, .a = 0.0f},
        (pos2_t){.x = 232, .y = 106, .a = 0.0f},
        (pos2_t){.x = 232, .y = 104, .a = 0.0f},
        (pos2_t){.x = 232, .y = 100, .a = 0.0f},
        (pos2_t){.x = 230, .y = 98, .a = 0.0f},
        (pos2_t){.x = 232, .y = 96, .a = 0.0f},
        (pos2_t){.x = 232, .y = 92, .a = 0.0f},
        (pos2_t){.x = 234, .y = 88, .a = 0.0f},
        (pos2_t){.x = 238, .y = 84, .a = 0.0f},
        (pos2_t){.x = 240, .y = 80, .a = 0.0f},
        (pos2_t){.x = 238, .y = 76, .a = 0.0f},
        (pos2_t){.x = 236, .y = 76, .a = 0.0f},
        (pos2_t){.x = 232, .y = 74, .a = 0.0f},
        (pos2_t){.x = 226, .y = 76, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps47);
    control_set_waypoints(&shared_ctrl, wps47, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps48[] = {
        (pos2_t){.x = 220, .y = 86, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps48);
    control_set_waypoints(&shared_ctrl, wps48, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps49[] = {
        (pos2_t){.x = 220, .y = 88, .a = 0.0f},
        (pos2_t){.x = 216, .y = 90, .a = 0.0f},
        (pos2_t){.x = 214, .y = 90, .a = 0.0f},
        (pos2_t){.x = 212, .y = 90, .a = 0.0f},
        (pos2_t){.x = 212, .y = 92, .a = 0.0f},
        (pos2_t){.x = 212, .y = 92, .a = 0.0f},
        (pos2_t){.x = 216, .y = 92, .a = 0.0f},
        (pos2_t){.x = 222, .y = 94, .a = 0.0f},
        (pos2_t){.x = 224, .y = 98, .a = 0.0f},
        (pos2_t){.x = 226, .y = 100, .a = 0.0f},
        (pos2_t){.x = 226, .y = 100, .a = 0.0f},
        (pos2_t){.x = 228, .y = 100, .a = 0.0f},
        (pos2_t){.x = 228, .y = 100, .a = 0.0f},
        (pos2_t){.x = 228, .y = 100, .a = 0.0f},
        (pos2_t){.x = 228, .y = 98, .a = 0.0f},
        (pos2_t){.x = 228, .y = 94, .a = 0.0f},
        (pos2_t){.x = 226, .y = 92, .a = 0.0f},
        (pos2_t){.x = 226, .y = 90, .a = 0.0f},
        (pos2_t){.x = 226, .y = 90, .a = 0.0f},
        (pos2_t){.x = 228, .y = 88, .a = 0.0f},
        (pos2_t){.x = 228, .y = 88, .a = 0.0f},
        (pos2_t){.x = 228, .y = 88, .a = 0.0f},
        (pos2_t){.x = 228, .y = 88, .a = 0.0f},
        (pos2_t){.x = 226, .y = 88, .a = 0.0f},
        (pos2_t){.x = 226, .y = 86, .a = 0.0f},
        (pos2_t){.x = 226, .y = 86, .a = 0.0f},
        (pos2_t){.x = 224, .y = 86, .a = 0.0f},
        (pos2_t){.x = 220, .y = 86, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps49);
    control_set_waypoints(&shared_ctrl, wps49, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps50[] = {
        (pos2_t){.x = 210, .y = 96, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps50);
    control_set_waypoints(&shared_ctrl, wps50, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps51[] = {
        (pos2_t){.x = 216, .y = 96, .a = 0.0f},
        (pos2_t){.x = 220, .y = 98, .a = 0.0f},
        (pos2_t){.x = 226, .y = 102, .a = 0.0f},
        (pos2_t){.x = 226, .y = 108, .a = 0.0f},
        (pos2_t){.x = 228, .y = 112, .a = 0.0f},
        (pos2_t){.x = 230, .y = 116, .a = 0.0f},
        (pos2_t){.x = 228, .y = 118, .a = 0.0f},
        (pos2_t){.x = 228, .y = 120, .a = 0.0f},
        (pos2_t){.x = 228, .y = 116, .a = 0.0f},
        (pos2_t){.x = 228, .y = 114, .a = 0.0f},
        (pos2_t){.x = 226, .y = 112, .a = 0.0f},
        (pos2_t){.x = 224, .y = 108, .a = 0.0f},
        (pos2_t){.x = 224, .y = 104, .a = 0.0f},
        (pos2_t){.x = 222, .y = 102, .a = 0.0f},
        (pos2_t){.x = 218, .y = 98, .a = 0.0f},
        (pos2_t){.x = 214, .y = 98, .a = 0.0f},
        (pos2_t){.x = 208, .y = 96, .a = 0.0f},
        (pos2_t){.x = 206, .y = 94, .a = 0.0f},
        (pos2_t){.x = 210, .y = 96, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps51);
    control_set_waypoints(&shared_ctrl, wps51, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps52[] = {
        (pos2_t){.x = 260, .y = 76, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps52);
    control_set_waypoints(&shared_ctrl, wps52, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps53[] = {
        (pos2_t){.x = 250, .y = 78, .a = 0.0f},
        (pos2_t){.x = 246, .y = 78, .a = 0.0f},
        (pos2_t){.x = 242, .y = 78, .a = 0.0f},
        (pos2_t){.x = 242, .y = 80, .a = 0.0f},
        (pos2_t){.x = 240, .y = 84, .a = 0.0f},
        (pos2_t){.x = 240, .y = 86, .a = 0.0f},
        (pos2_t){.x = 242, .y = 86, .a = 0.0f},
        (pos2_t){.x = 250, .y = 82, .a = 0.0f},
        (pos2_t){.x = 262, .y = 82, .a = 0.0f},
        (pos2_t){.x = 264, .y = 82, .a = 0.0f},
        (pos2_t){.x = 266, .y = 80, .a = 0.0f},
        (pos2_t){.x = 268, .y = 78, .a = 0.0f},
        (pos2_t){.x = 266, .y = 76, .a = 0.0f},
        (pos2_t){.x = 260, .y = 76, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps53);
    control_set_waypoints(&shared_ctrl, wps53, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps54[] = {
        (pos2_t){.x = 100, .y = 90, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps54);
    control_set_waypoints(&shared_ctrl, wps54, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps55[] = {
        (pos2_t){.x = 98, .y = 92, .a = 0.0f},
        (pos2_t){.x = 96, .y = 96, .a = 0.0f},
        (pos2_t){.x = 94, .y = 98, .a = 0.0f},
        (pos2_t){.x = 92, .y = 100, .a = 0.0f},
        (pos2_t){.x = 86, .y = 100, .a = 0.0f},
        (pos2_t){.x = 80, .y = 96, .a = 0.0f},
        (pos2_t){.x = 78, .y = 92, .a = 0.0f},
        (pos2_t){.x = 74, .y = 92, .a = 0.0f},
        (pos2_t){.x = 70, .y = 92, .a = 0.0f},
        (pos2_t){.x = 68, .y = 94, .a = 0.0f},
        (pos2_t){.x = 66, .y = 96, .a = 0.0f},
        (pos2_t){.x = 66, .y = 100, .a = 0.0f},
        (pos2_t){.x = 64, .y = 102, .a = 0.0f},
        (pos2_t){.x = 62, .y = 104, .a = 0.0f},
        (pos2_t){.x = 62, .y = 104, .a = 0.0f},
        (pos2_t){.x = 60, .y = 106, .a = 0.0f},
        (pos2_t){.x = 60, .y = 106, .a = 0.0f},
        (pos2_t){.x = 62, .y = 110, .a = 0.0f},
        (pos2_t){.x = 64, .y = 114, .a = 0.0f},
        (pos2_t){.x = 72, .y = 122, .a = 0.0f},
        (pos2_t){.x = 74, .y = 120, .a = 0.0f},
        (pos2_t){.x = 78, .y = 114, .a = 0.0f},
        (pos2_t){.x = 84, .y = 110, .a = 0.0f},
        (pos2_t){.x = 88, .y = 108, .a = 0.0f},
        (pos2_t){.x = 92, .y = 108, .a = 0.0f},
        (pos2_t){.x = 98, .y = 110, .a = 0.0f},
        (pos2_t){.x = 100, .y = 110, .a = 0.0f},
        (pos2_t){.x = 100, .y = 110, .a = 0.0f},
        (pos2_t){.x = 104, .y = 108, .a = 0.0f},
        (pos2_t){.x = 108, .y = 108, .a = 0.0f},
        (pos2_t){.x = 108, .y = 108, .a = 0.0f},
        (pos2_t){.x = 108, .y = 110, .a = 0.0f},
        (pos2_t){.x = 104, .y = 110, .a = 0.0f},
        (pos2_t){.x = 102, .y = 110, .a = 0.0f},
        (pos2_t){.x = 104, .y = 114, .a = 0.0f},
        (pos2_t){.x = 108, .y = 118, .a = 0.0f},
        (pos2_t){.x = 110, .y = 120, .a = 0.0f},
        (pos2_t){.x = 110, .y = 120, .a = 0.0f},
        (pos2_t){.x = 114, .y = 116, .a = 0.0f},
        (pos2_t){.x = 118, .y = 116, .a = 0.0f},
        (pos2_t){.x = 118, .y = 114, .a = 0.0f},
        (pos2_t){.x = 118, .y = 114, .a = 0.0f},
        (pos2_t){.x = 116, .y = 114, .a = 0.0f},
        (pos2_t){.x = 112, .y = 108, .a = 0.0f},
        (pos2_t){.x = 110, .y = 104, .a = 0.0f},
        (pos2_t){.x = 108, .y = 100, .a = 0.0f},
        (pos2_t){.x = 110, .y = 96, .a = 0.0f},
        (pos2_t){.x = 114, .y = 94, .a = 0.0f},
        (pos2_t){.x = 114, .y = 92, .a = 0.0f},
        (pos2_t){.x = 104, .y = 90, .a = 0.0f},
        (pos2_t){.x = 100, .y = 90, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps55);
    control_set_waypoints(&shared_ctrl, wps55, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps56[] = {
        (pos2_t){.x = 70, .y = 106, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps56);
    control_set_waypoints(&shared_ctrl, wps56, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps57[] = {
        (pos2_t){.x = 68, .y = 108, .a = 0.0f},
        (pos2_t){.x = 66, .y = 110, .a = 0.0f},
        (pos2_t){.x = 68, .y = 110, .a = 0.0f},
        (pos2_t){.x = 68, .y = 110, .a = 0.0f},
        (pos2_t){.x = 68, .y = 112, .a = 0.0f},
        (pos2_t){.x = 68, .y = 112, .a = 0.0f},
        (pos2_t){.x = 68, .y = 114, .a = 0.0f},
        (pos2_t){.x = 70, .y = 116, .a = 0.0f},
        (pos2_t){.x = 70, .y = 116, .a = 0.0f},
        (pos2_t){.x = 70, .y = 116, .a = 0.0f},
        (pos2_t){.x = 72, .y = 116, .a = 0.0f},
        (pos2_t){.x = 72, .y = 116, .a = 0.0f},
        (pos2_t){.x = 72, .y = 116, .a = 0.0f},
        (pos2_t){.x = 72, .y = 112, .a = 0.0f},
        (pos2_t){.x = 74, .y = 112, .a = 0.0f},
        (pos2_t){.x = 76, .y = 108, .a = 0.0f},
        (pos2_t){.x = 76, .y = 108, .a = 0.0f},
        (pos2_t){.x = 74, .y = 108, .a = 0.0f},
        (pos2_t){.x = 74, .y = 108, .a = 0.0f},
        (pos2_t){.x = 74, .y = 108, .a = 0.0f},
        (pos2_t){.x = 72, .y = 106, .a = 0.0f},
        (pos2_t){.x = 70, .y = 106, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps57);
    control_set_waypoints(&shared_ctrl, wps57, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps58[] = {
        (pos2_t){.x = 106, .y = 104, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps58);
    control_set_waypoints(&shared_ctrl, wps58, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps59[] = {
        (pos2_t){.x = 106, .y = 104, .a = 0.0f},
        (pos2_t){.x = 106, .y = 104, .a = 0.0f},
        (pos2_t){.x = 100, .y = 106, .a = 0.0f},
        (pos2_t){.x = 98, .y = 108, .a = 0.0f},
        (pos2_t){.x = 102, .y = 104, .a = 0.0f},
        (pos2_t){.x = 106, .y = 104, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps59);
    control_set_waypoints(&shared_ctrl, wps59, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps60[] = {
        (pos2_t){.x = 192, .y = 94, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps60);
    control_set_waypoints(&shared_ctrl, wps60, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps61[] = {
        (pos2_t){.x = 190, .y = 94, .a = 0.0f},
        (pos2_t){.x = 184, .y = 100, .a = 0.0f},
        (pos2_t){.x = 180, .y = 108, .a = 0.0f},
        (pos2_t){.x = 180, .y = 108, .a = 0.0f},
        (pos2_t){.x = 180, .y = 110, .a = 0.0f},
        (pos2_t){.x = 180, .y = 110, .a = 0.0f},
        (pos2_t){.x = 186, .y = 112, .a = 0.0f},
        (pos2_t){.x = 200, .y = 120, .a = 0.0f},
        (pos2_t){.x = 204, .y = 122, .a = 0.0f},
        (pos2_t){.x = 208, .y = 114, .a = 0.0f},
        (pos2_t){.x = 210, .y = 106, .a = 0.0f},
        (pos2_t){.x = 208, .y = 102, .a = 0.0f},
        (pos2_t){.x = 206, .y = 98, .a = 0.0f},
        (pos2_t){.x = 202, .y = 96, .a = 0.0f},
        (pos2_t){.x = 196, .y = 94, .a = 0.0f},
        (pos2_t){.x = 194, .y = 92, .a = 0.0f},
        (pos2_t){.x = 192, .y = 94, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps61);
    control_set_waypoints(&shared_ctrl, wps61, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps62[] = {
        (pos2_t){.x = 120, .y = 94, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps62);
    control_set_waypoints(&shared_ctrl, wps62, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps63[] = {
        (pos2_t){.x = 118, .y = 94, .a = 0.0f},
        (pos2_t){.x = 118, .y = 94, .a = 0.0f},
        (pos2_t){.x = 118, .y = 94, .a = 0.0f},
        (pos2_t){.x = 120, .y = 96, .a = 0.0f},
        (pos2_t){.x = 122, .y = 96, .a = 0.0f},
        (pos2_t){.x = 122, .y = 94, .a = 0.0f},
        (pos2_t){.x = 122, .y = 94, .a = 0.0f},
        (pos2_t){.x = 120, .y = 94, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps63);
    control_set_waypoints(&shared_ctrl, wps63, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps64[] = {
        (pos2_t){.x = 140, .y = 100, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps64);
    control_set_waypoints(&shared_ctrl, wps64, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps65[] = {
        (pos2_t){.x = 136, .y = 102, .a = 0.0f},
        (pos2_t){.x = 134, .y = 108, .a = 0.0f},
        (pos2_t){.x = 134, .y = 108, .a = 0.0f},
        (pos2_t){.x = 134, .y = 108, .a = 0.0f},
        (pos2_t){.x = 134, .y = 108, .a = 0.0f},
        (pos2_t){.x = 140, .y = 108, .a = 0.0f},
        (pos2_t){.x = 144, .y = 106, .a = 0.0f},
        (pos2_t){.x = 146, .y = 104, .a = 0.0f},
        (pos2_t){.x = 146, .y = 102, .a = 0.0f},
        (pos2_t){.x = 146, .y = 100, .a = 0.0f},
        (pos2_t){.x = 142, .y = 98, .a = 0.0f},
        (pos2_t){.x = 140, .y = 100, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps65);
    control_set_waypoints(&shared_ctrl, wps65, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps66[] = {
        (pos2_t){.x = 130, .y = 108, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps66);
    control_set_waypoints(&shared_ctrl, wps66, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps67[] = {
        (pos2_t){.x = 128, .y = 108, .a = 0.0f},
        (pos2_t){.x = 128, .y = 110, .a = 0.0f},
        (pos2_t){.x = 130, .y = 112, .a = 0.0f},
        (pos2_t){.x = 130, .y = 112, .a = 0.0f},
        (pos2_t){.x = 132, .y = 112, .a = 0.0f},
        (pos2_t){.x = 132, .y = 112, .a = 0.0f},
        (pos2_t){.x = 132, .y = 110, .a = 0.0f},
        (pos2_t){.x = 134, .y = 110, .a = 0.0f},
        (pos2_t){.x = 134, .y = 110, .a = 0.0f},
        (pos2_t){.x = 134, .y = 110, .a = 0.0f},
        (pos2_t){.x = 132, .y = 108, .a = 0.0f},
        (pos2_t){.x = 130, .y = 106, .a = 0.0f},
        (pos2_t){.x = 130, .y = 108, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps67);
    control_set_waypoints(&shared_ctrl, wps67, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps68[] = {
        (pos2_t){.x = 150, .y = 108, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps68);
    control_set_waypoints(&shared_ctrl, wps68, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps69[] = {
        (pos2_t){.x = 146, .y = 108, .a = 0.0f},
        (pos2_t){.x = 146, .y = 110, .a = 0.0f},
        (pos2_t){.x = 146, .y = 110, .a = 0.0f},
        (pos2_t){.x = 146, .y = 110, .a = 0.0f},
        (pos2_t){.x = 146, .y = 110, .a = 0.0f},
        (pos2_t){.x = 150, .y = 110, .a = 0.0f},
        (pos2_t){.x = 166, .y = 112, .a = 0.0f},
        (pos2_t){.x = 176, .y = 116, .a = 0.0f},
        (pos2_t){.x = 184, .y = 120, .a = 0.0f},
        (pos2_t){.x = 188, .y = 124, .a = 0.0f},
        (pos2_t){.x = 198, .y = 132, .a = 0.0f},
        (pos2_t){.x = 206, .y = 144, .a = 0.0f},
        (pos2_t){.x = 212, .y = 156, .a = 0.0f},
        (pos2_t){.x = 216, .y = 170, .a = 0.0f},
        (pos2_t){.x = 216, .y = 182, .a = 0.0f},
        (pos2_t){.x = 216, .y = 192, .a = 0.0f},
        (pos2_t){.x = 212, .y = 204, .a = 0.0f},
        (pos2_t){.x = 208, .y = 212, .a = 0.0f},
        (pos2_t){.x = 202, .y = 222, .a = 0.0f},
        (pos2_t){.x = 194, .y = 230, .a = 0.0f},
        (pos2_t){.x = 186, .y = 236, .a = 0.0f},
        (pos2_t){.x = 178, .y = 242, .a = 0.0f},
        (pos2_t){.x = 168, .y = 244, .a = 0.0f},
        (pos2_t){.x = 156, .y = 248, .a = 0.0f},
        (pos2_t){.x = 140, .y = 248, .a = 0.0f},
        (pos2_t){.x = 130, .y = 244, .a = 0.0f},
        (pos2_t){.x = 120, .y = 240, .a = 0.0f},
        (pos2_t){.x = 110, .y = 236, .a = 0.0f},
        (pos2_t){.x = 102, .y = 228, .a = 0.0f},
        (pos2_t){.x = 94, .y = 220, .a = 0.0f},
        (pos2_t){.x = 88, .y = 210, .a = 0.0f},
        (pos2_t){.x = 82, .y = 194, .a = 0.0f},
        (pos2_t){.x = 80, .y = 186, .a = 0.0f},
        (pos2_t){.x = 80, .y = 178, .a = 0.0f},
        (pos2_t){.x = 82, .y = 164, .a = 0.0f},
        (pos2_t){.x = 86, .y = 150, .a = 0.0f},
        (pos2_t){.x = 86, .y = 148, .a = 0.0f},
        (pos2_t){.x = 86, .y = 148, .a = 0.0f},
        (pos2_t){.x = 86, .y = 148, .a = 0.0f},
        (pos2_t){.x = 84, .y = 152, .a = 0.0f},
        (pos2_t){.x = 80, .y = 162, .a = 0.0f},
        (pos2_t){.x = 76, .y = 172, .a = 0.0f},
        (pos2_t){.x = 76, .y = 182, .a = 0.0f},
        (pos2_t){.x = 76, .y = 194, .a = 0.0f},
        (pos2_t){.x = 78, .y = 208, .a = 0.0f},
        (pos2_t){.x = 80, .y = 216, .a = 0.0f},
        (pos2_t){.x = 84, .y = 226, .a = 0.0f},
        (pos2_t){.x = 90, .y = 234, .a = 0.0f},
        (pos2_t){.x = 94, .y = 240, .a = 0.0f},
        (pos2_t){.x = 106, .y = 252, .a = 0.0f},
        (pos2_t){.x = 120, .y = 260, .a = 0.0f},
        (pos2_t){.x = 124, .y = 262, .a = 0.0f},
        (pos2_t){.x = 126, .y = 260, .a = 0.0f},
        (pos2_t){.x = 128, .y = 260, .a = 0.0f},
        (pos2_t){.x = 128, .y = 260, .a = 0.0f},
        (pos2_t){.x = 128, .y = 260, .a = 0.0f},
        (pos2_t){.x = 132, .y = 260, .a = 0.0f},
        (pos2_t){.x = 142, .y = 262, .a = 0.0f},
        (pos2_t){.x = 152, .y = 262, .a = 0.0f},
        (pos2_t){.x = 164, .y = 260, .a = 0.0f},
        (pos2_t){.x = 168, .y = 262, .a = 0.0f},
        (pos2_t){.x = 170, .y = 262, .a = 0.0f},
        (pos2_t){.x = 170, .y = 260, .a = 0.0f},
        (pos2_t){.x = 172, .y = 254, .a = 0.0f},
        (pos2_t){.x = 172, .y = 252, .a = 0.0f},
        (pos2_t){.x = 174, .y = 252, .a = 0.0f},
        (pos2_t){.x = 174, .y = 252, .a = 0.0f},
        (pos2_t){.x = 174, .y = 252, .a = 0.0f},
        (pos2_t){.x = 174, .y = 252, .a = 0.0f},
        (pos2_t){.x = 176, .y = 254, .a = 0.0f},
        (pos2_t){.x = 178, .y = 258, .a = 0.0f},
        (pos2_t){.x = 180, .y = 260, .a = 0.0f},
        (pos2_t){.x = 182, .y = 258, .a = 0.0f},
        (pos2_t){.x = 192, .y = 252, .a = 0.0f},
        (pos2_t){.x = 202, .y = 246, .a = 0.0f},
        (pos2_t){.x = 210, .y = 238, .a = 0.0f},
        (pos2_t){.x = 214, .y = 234, .a = 0.0f},
        (pos2_t){.x = 216, .y = 236, .a = 0.0f},
        (pos2_t){.x = 218, .y = 236, .a = 0.0f},
        (pos2_t){.x = 222, .y = 232, .a = 0.0f},
        (pos2_t){.x = 228, .y = 222, .a = 0.0f},
        (pos2_t){.x = 232, .y = 210, .a = 0.0f},
        (pos2_t){.x = 234, .y = 200, .a = 0.0f},
        (pos2_t){.x = 234, .y = 188, .a = 0.0f},
        (pos2_t){.x = 234, .y = 176, .a = 0.0f},
        (pos2_t){.x = 232, .y = 166, .a = 0.0f},
        (pos2_t){.x = 228, .y = 154, .a = 0.0f},
        (pos2_t){.x = 222, .y = 144, .a = 0.0f},
        (pos2_t){.x = 212, .y = 132, .a = 0.0f},
        (pos2_t){.x = 200, .y = 122, .a = 0.0f},
        (pos2_t){.x = 186, .y = 114, .a = 0.0f},
        (pos2_t){.x = 172, .y = 110, .a = 0.0f},
        (pos2_t){.x = 160, .y = 108, .a = 0.0f},
        (pos2_t){.x = 150, .y = 108, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps69);
    control_set_waypoints(&shared_ctrl, wps69, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps70[] = {
        (pos2_t){.x = 228, .y = 186, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps70);
    control_set_waypoints(&shared_ctrl, wps70, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps71[] = {
        (pos2_t){.x = 226, .y = 188, .a = 0.0f},
        (pos2_t){.x = 224, .y = 196, .a = 0.0f},
        (pos2_t){.x = 222, .y = 204, .a = 0.0f},
        (pos2_t){.x = 220, .y = 210, .a = 0.0f},
        (pos2_t){.x = 222, .y = 212, .a = 0.0f},
        (pos2_t){.x = 224, .y = 214, .a = 0.0f},
        (pos2_t){.x = 226, .y = 214, .a = 0.0f},
        (pos2_t){.x = 228, .y = 210, .a = 0.0f},
        (pos2_t){.x = 230, .y = 206, .a = 0.0f},
        (pos2_t){.x = 232, .y = 198, .a = 0.0f},
        (pos2_t){.x = 232, .y = 194, .a = 0.0f},
        (pos2_t){.x = 232, .y = 192, .a = 0.0f},
        (pos2_t){.x = 232, .y = 190, .a = 0.0f},
        (pos2_t){.x = 232, .y = 190, .a = 0.0f},
        (pos2_t){.x = 234, .y = 188, .a = 0.0f},
        (pos2_t){.x = 234, .y = 186, .a = 0.0f},
        (pos2_t){.x = 232, .y = 186, .a = 0.0f},
        (pos2_t){.x = 232, .y = 186, .a = 0.0f},
        (pos2_t){.x = 232, .y = 186, .a = 0.0f},
        (pos2_t){.x = 230, .y = 186, .a = 0.0f},
        (pos2_t){.x = 230, .y = 186, .a = 0.0f},
        (pos2_t){.x = 230, .y = 186, .a = 0.0f},
        (pos2_t){.x = 230, .y = 186, .a = 0.0f},
        (pos2_t){.x = 230, .y = 186, .a = 0.0f},
        (pos2_t){.x = 228, .y = 186, .a = 0.0f},
        (pos2_t){.x = 228, .y = 186, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps71);
    control_set_waypoints(&shared_ctrl, wps71, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps72[] = {
        (pos2_t){.x = 214, .y = 208, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps72);
    control_set_waypoints(&shared_ctrl, wps72, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps73[] = {
        (pos2_t){.x = 214, .y = 212, .a = 0.0f},
        (pos2_t){.x = 212, .y = 214, .a = 0.0f},
        (pos2_t){.x = 214, .y = 216, .a = 0.0f},
        (pos2_t){.x = 214, .y = 218, .a = 0.0f},
        (pos2_t){.x = 216, .y = 216, .a = 0.0f},
        (pos2_t){.x = 218, .y = 214, .a = 0.0f},
        (pos2_t){.x = 218, .y = 210, .a = 0.0f},
        (pos2_t){.x = 218, .y = 208, .a = 0.0f},
        (pos2_t){.x = 216, .y = 208, .a = 0.0f},
        (pos2_t){.x = 214, .y = 208, .a = 0.0f},
        (pos2_t){.x = 214, .y = 208, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps73);
    control_set_waypoints(&shared_ctrl, wps73, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps74[] = {
        (pos2_t){.x = 210, .y = 220, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps74);
    control_set_waypoints(&shared_ctrl, wps74, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps75[] = {
        (pos2_t){.x = 208, .y = 222, .a = 0.0f},
        (pos2_t){.x = 208, .y = 222, .a = 0.0f},
        (pos2_t){.x = 206, .y = 222, .a = 0.0f},
        (pos2_t){.x = 206, .y = 222, .a = 0.0f},
        (pos2_t){.x = 204, .y = 224, .a = 0.0f},
        (pos2_t){.x = 204, .y = 226, .a = 0.0f},
        (pos2_t){.x = 204, .y = 228, .a = 0.0f},
        (pos2_t){.x = 202, .y = 228, .a = 0.0f},
        (pos2_t){.x = 202, .y = 230, .a = 0.0f},
        (pos2_t){.x = 204, .y = 230, .a = 0.0f},
        (pos2_t){.x = 204, .y = 230, .a = 0.0f},
        (pos2_t){.x = 206, .y = 230, .a = 0.0f},
        (pos2_t){.x = 210, .y = 232, .a = 0.0f},
        (pos2_t){.x = 212, .y = 232, .a = 0.0f},
        (pos2_t){.x = 212, .y = 230, .a = 0.0f},
        (pos2_t){.x = 212, .y = 228, .a = 0.0f},
        (pos2_t){.x = 214, .y = 228, .a = 0.0f},
        (pos2_t){.x = 216, .y = 226, .a = 0.0f},
        (pos2_t){.x = 216, .y = 226, .a = 0.0f},
        (pos2_t){.x = 216, .y = 226, .a = 0.0f},
        (pos2_t){.x = 216, .y = 224, .a = 0.0f},
        (pos2_t){.x = 214, .y = 224, .a = 0.0f},
        (pos2_t){.x = 212, .y = 222, .a = 0.0f},
        (pos2_t){.x = 212, .y = 220, .a = 0.0f},
        (pos2_t){.x = 210, .y = 220, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps75);
    control_set_waypoints(&shared_ctrl, wps75, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps76[] = {
        (pos2_t){.x = 86, .y = 112, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps76);
    control_set_waypoints(&shared_ctrl, wps76, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps77[] = {
        (pos2_t){.x = 80, .y = 116, .a = 0.0f},
        (pos2_t){.x = 74, .y = 124, .a = 0.0f},
        (pos2_t){.x = 76, .y = 130, .a = 0.0f},
        (pos2_t){.x = 80, .y = 138, .a = 0.0f},
        (pos2_t){.x = 86, .y = 142, .a = 0.0f},
        (pos2_t){.x = 88, .y = 140, .a = 0.0f},
        (pos2_t){.x = 96, .y = 132, .a = 0.0f},
        (pos2_t){.x = 104, .y = 124, .a = 0.0f},
        (pos2_t){.x = 106, .y = 122, .a = 0.0f},
        (pos2_t){.x = 98, .y = 112, .a = 0.0f},
        (pos2_t){.x = 96, .y = 110, .a = 0.0f},
        (pos2_t){.x = 92, .y = 110, .a = 0.0f},
        (pos2_t){.x = 86, .y = 112, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps77);
    control_set_waypoints(&shared_ctrl, wps77, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps78[] = {
        (pos2_t){.x = 142, .y = 112, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps78);
    control_set_waypoints(&shared_ctrl, wps78, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps79[] = {
        (pos2_t){.x = 136, .y = 114, .a = 0.0f},
        (pos2_t){.x = 132, .y = 114, .a = 0.0f},
        (pos2_t){.x = 128, .y = 116, .a = 0.0f},
        (pos2_t){.x = 124, .y = 116, .a = 0.0f},
        (pos2_t){.x = 118, .y = 120, .a = 0.0f},
        (pos2_t){.x = 108, .y = 126, .a = 0.0f},
        (pos2_t){.x = 96, .y = 136, .a = 0.0f},
        (pos2_t){.x = 88, .y = 150, .a = 0.0f},
        (pos2_t){.x = 84, .y = 162, .a = 0.0f},
        (pos2_t){.x = 82, .y = 170, .a = 0.0f},
        (pos2_t){.x = 82, .y = 178, .a = 0.0f},
        (pos2_t){.x = 82, .y = 186, .a = 0.0f},
        (pos2_t){.x = 84, .y = 196, .a = 0.0f},
        (pos2_t){.x = 88, .y = 204, .a = 0.0f},
        (pos2_t){.x = 90, .y = 212, .a = 0.0f},
        (pos2_t){.x = 96, .y = 218, .a = 0.0f},
        (pos2_t){.x = 102, .y = 224, .a = 0.0f},
        (pos2_t){.x = 108, .y = 230, .a = 0.0f},
        (pos2_t){.x = 114, .y = 236, .a = 0.0f},
        (pos2_t){.x = 122, .y = 238, .a = 0.0f},
        (pos2_t){.x = 132, .y = 242, .a = 0.0f},
        (pos2_t){.x = 142, .y = 244, .a = 0.0f},
        (pos2_t){.x = 154, .y = 244, .a = 0.0f},
        (pos2_t){.x = 166, .y = 242, .a = 0.0f},
        (pos2_t){.x = 176, .y = 238, .a = 0.0f},
        (pos2_t){.x = 184, .y = 232, .a = 0.0f},
        (pos2_t){.x = 192, .y = 228, .a = 0.0f},
        (pos2_t){.x = 198, .y = 222, .a = 0.0f},
        (pos2_t){.x = 202, .y = 216, .a = 0.0f},
        (pos2_t){.x = 206, .y = 208, .a = 0.0f},
        (pos2_t){.x = 210, .y = 200, .a = 0.0f},
        (pos2_t){.x = 212, .y = 192, .a = 0.0f},
        (pos2_t){.x = 214, .y = 184, .a = 0.0f},
        (pos2_t){.x = 214, .y = 172, .a = 0.0f},
        (pos2_t){.x = 212, .y = 160, .a = 0.0f},
        (pos2_t){.x = 208, .y = 148, .a = 0.0f},
        (pos2_t){.x = 200, .y = 138, .a = 0.0f},
        (pos2_t){.x = 188, .y = 126, .a = 0.0f},
        (pos2_t){.x = 176, .y = 118, .a = 0.0f},
        (pos2_t){.x = 162, .y = 114, .a = 0.0f},
        (pos2_t){.x = 150, .y = 112, .a = 0.0f},
        (pos2_t){.x = 142, .y = 112, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps79);
    control_set_waypoints(&shared_ctrl, wps79, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps80[] = {
        (pos2_t){.x = 124, .y = 132, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps80);
    control_set_waypoints(&shared_ctrl, wps80, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps81[] = {
        (pos2_t){.x = 120, .y = 134, .a = 0.0f},
        (pos2_t){.x = 116, .y = 136, .a = 0.0f},
        (pos2_t){.x = 114, .y = 140, .a = 0.0f},
        (pos2_t){.x = 114, .y = 142, .a = 0.0f},
        (pos2_t){.x = 116, .y = 148, .a = 0.0f},
        (pos2_t){.x = 120, .y = 154, .a = 0.0f},
        (pos2_t){.x = 120, .y = 156, .a = 0.0f},
        (pos2_t){.x = 120, .y = 158, .a = 0.0f},
        (pos2_t){.x = 118, .y = 162, .a = 0.0f},
        (pos2_t){.x = 118, .y = 166, .a = 0.0f},
        (pos2_t){.x = 118, .y = 172, .a = 0.0f},
        (pos2_t){.x = 120, .y = 176, .a = 0.0f},
        (pos2_t){.x = 124, .y = 180, .a = 0.0f},
        (pos2_t){.x = 128, .y = 184, .a = 0.0f},
        (pos2_t){.x = 134, .y = 186, .a = 0.0f},
        (pos2_t){.x = 140, .y = 186, .a = 0.0f},
        (pos2_t){.x = 142, .y = 186, .a = 0.0f},
        (pos2_t){.x = 142, .y = 186, .a = 0.0f},
        (pos2_t){.x = 142, .y = 184, .a = 0.0f},
        (pos2_t){.x = 140, .y = 184, .a = 0.0f},
        (pos2_t){.x = 130, .y = 182, .a = 0.0f},
        (pos2_t){.x = 124, .y = 176, .a = 0.0f},
        (pos2_t){.x = 122, .y = 174, .a = 0.0f},
        (pos2_t){.x = 120, .y = 170, .a = 0.0f},
        (pos2_t){.x = 120, .y = 162, .a = 0.0f},
        (pos2_t){.x = 122, .y = 164, .a = 0.0f},
        (pos2_t){.x = 124, .y = 164, .a = 0.0f},
        (pos2_t){.x = 128, .y = 162, .a = 0.0f},
        (pos2_t){.x = 130, .y = 158, .a = 0.0f},
        (pos2_t){.x = 132, .y = 152, .a = 0.0f},
        (pos2_t){.x = 136, .y = 156, .a = 0.0f},
        (pos2_t){.x = 136, .y = 160, .a = 0.0f},
        (pos2_t){.x = 136, .y = 162, .a = 0.0f},
        (pos2_t){.x = 132, .y = 164, .a = 0.0f},
        (pos2_t){.x = 130, .y = 166, .a = 0.0f},
        (pos2_t){.x = 130, .y = 170, .a = 0.0f},
        (pos2_t){.x = 132, .y = 174, .a = 0.0f},
        (pos2_t){.x = 136, .y = 176, .a = 0.0f},
        (pos2_t){.x = 142, .y = 176, .a = 0.0f},
        (pos2_t){.x = 148, .y = 174, .a = 0.0f},
        (pos2_t){.x = 150, .y = 172, .a = 0.0f},
        (pos2_t){.x = 152, .y = 170, .a = 0.0f},
        (pos2_t){.x = 152, .y = 166, .a = 0.0f},
        (pos2_t){.x = 150, .y = 162, .a = 0.0f},
        (pos2_t){.x = 148, .y = 160, .a = 0.0f},
        (pos2_t){.x = 144, .y = 160, .a = 0.0f},
        (pos2_t){.x = 140, .y = 160, .a = 0.0f},
        (pos2_t){.x = 140, .y = 160, .a = 0.0f},
        (pos2_t){.x = 138, .y = 158, .a = 0.0f},
        (pos2_t){.x = 136, .y = 152, .a = 0.0f},
        (pos2_t){.x = 138, .y = 152, .a = 0.0f},
        (pos2_t){.x = 144, .y = 152, .a = 0.0f},
        (pos2_t){.x = 150, .y = 154, .a = 0.0f},
        (pos2_t){.x = 154, .y = 152, .a = 0.0f},
        (pos2_t){.x = 154, .y = 148, .a = 0.0f},
        (pos2_t){.x = 152, .y = 144, .a = 0.0f},
        (pos2_t){.x = 150, .y = 144, .a = 0.0f},
        (pos2_t){.x = 152, .y = 144, .a = 0.0f},
        (pos2_t){.x = 158, .y = 148, .a = 0.0f},
        (pos2_t){.x = 162, .y = 152, .a = 0.0f},
        (pos2_t){.x = 164, .y = 156, .a = 0.0f},
        (pos2_t){.x = 164, .y = 162, .a = 0.0f},
        (pos2_t){.x = 164, .y = 166, .a = 0.0f},
        (pos2_t){.x = 162, .y = 172, .a = 0.0f},
        (pos2_t){.x = 158, .y = 176, .a = 0.0f},
        (pos2_t){.x = 158, .y = 178, .a = 0.0f},
        (pos2_t){.x = 158, .y = 180, .a = 0.0f},
        (pos2_t){.x = 158, .y = 180, .a = 0.0f},
        (pos2_t){.x = 162, .y = 176, .a = 0.0f},
        (pos2_t){.x = 166, .y = 170, .a = 0.0f},
        (pos2_t){.x = 166, .y = 164, .a = 0.0f},
        (pos2_t){.x = 166, .y = 158, .a = 0.0f},
        (pos2_t){.x = 164, .y = 150, .a = 0.0f},
        (pos2_t){.x = 162, .y = 146, .a = 0.0f},
        (pos2_t){.x = 158, .y = 144, .a = 0.0f},
        (pos2_t){.x = 154, .y = 142, .a = 0.0f},
        (pos2_t){.x = 150, .y = 142, .a = 0.0f},
        (pos2_t){.x = 146, .y = 140, .a = 0.0f},
        (pos2_t){.x = 140, .y = 134, .a = 0.0f},
        (pos2_t){.x = 132, .y = 132, .a = 0.0f},
        (pos2_t){.x = 128, .y = 132, .a = 0.0f},
        (pos2_t){.x = 124, .y = 132, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps81);
    control_set_waypoints(&shared_ctrl, wps81, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps82[] = {
        (pos2_t){.x = 136, .y = 136, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps82);
    control_set_waypoints(&shared_ctrl, wps82, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps83[] = {
        (pos2_t){.x = 144, .y = 140, .a = 0.0f},
        (pos2_t){.x = 148, .y = 146, .a = 0.0f},
        (pos2_t){.x = 150, .y = 148, .a = 0.0f},
        (pos2_t){.x = 148, .y = 148, .a = 0.0f},
        (pos2_t){.x = 148, .y = 148, .a = 0.0f},
        (pos2_t){.x = 146, .y = 148, .a = 0.0f},
        (pos2_t){.x = 142, .y = 144, .a = 0.0f},
        (pos2_t){.x = 136, .y = 138, .a = 0.0f},
        (pos2_t){.x = 130, .y = 134, .a = 0.0f},
        (pos2_t){.x = 130, .y = 134, .a = 0.0f},
        (pos2_t){.x = 130, .y = 134, .a = 0.0f},
        (pos2_t){.x = 130, .y = 134, .a = 0.0f},
        (pos2_t){.x = 136, .y = 136, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps83);
    control_set_waypoints(&shared_ctrl, wps83, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps84[] = {
        (pos2_t){.x = 126, .y = 136, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps84);
    control_set_waypoints(&shared_ctrl, wps84, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps85[] = {
        (pos2_t){.x = 132, .y = 138, .a = 0.0f},
        (pos2_t){.x = 138, .y = 142, .a = 0.0f},
        (pos2_t){.x = 140, .y = 146, .a = 0.0f},
        (pos2_t){.x = 140, .y = 148, .a = 0.0f},
        (pos2_t){.x = 138, .y = 148, .a = 0.0f},
        (pos2_t){.x = 132, .y = 150, .a = 0.0f},
        (pos2_t){.x = 132, .y = 150, .a = 0.0f},
        (pos2_t){.x = 132, .y = 152, .a = 0.0f},
        (pos2_t){.x = 132, .y = 152, .a = 0.0f},
        (pos2_t){.x = 128, .y = 154, .a = 0.0f},
        (pos2_t){.x = 128, .y = 154, .a = 0.0f},
        (pos2_t){.x = 124, .y = 150, .a = 0.0f},
        (pos2_t){.x = 120, .y = 146, .a = 0.0f},
        (pos2_t){.x = 120, .y = 142, .a = 0.0f},
        (pos2_t){.x = 120, .y = 138, .a = 0.0f},
        (pos2_t){.x = 124, .y = 138, .a = 0.0f},
        (pos2_t){.x = 126, .y = 136, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps85);
    control_set_waypoints(&shared_ctrl, wps85, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps86[] = {
        (pos2_t){.x = 130, .y = 144, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps86);
    control_set_waypoints(&shared_ctrl, wps86, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps87[] = {
        (pos2_t){.x = 130, .y = 144, .a = 0.0f},
        (pos2_t){.x = 130, .y = 146, .a = 0.0f},
        (pos2_t){.x = 130, .y = 146, .a = 0.0f},
        (pos2_t){.x = 130, .y = 146, .a = 0.0f},
        (pos2_t){.x = 132, .y = 146, .a = 0.0f},
        (pos2_t){.x = 134, .y = 144, .a = 0.0f},
        (pos2_t){.x = 132, .y = 144, .a = 0.0f},
        (pos2_t){.x = 132, .y = 144, .a = 0.0f},
        (pos2_t){.x = 130, .y = 144, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps87);
    control_set_waypoints(&shared_ctrl, wps87, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps88[] = {
        (pos2_t){.x = 144, .y = 166, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps88);
    control_set_waypoints(&shared_ctrl, wps88, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps89[] = {
        (pos2_t){.x = 142, .y = 168, .a = 0.0f},
        (pos2_t){.x = 144, .y = 170, .a = 0.0f},
        (pos2_t){.x = 144, .y = 170, .a = 0.0f},
        (pos2_t){.x = 144, .y = 170, .a = 0.0f},
        (pos2_t){.x = 144, .y = 170, .a = 0.0f},
        (pos2_t){.x = 146, .y = 168, .a = 0.0f},
        (pos2_t){.x = 146, .y = 166, .a = 0.0f},
        (pos2_t){.x = 146, .y = 166, .a = 0.0f},
        (pos2_t){.x = 144, .y = 166, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps89);
    control_set_waypoints(&shared_ctrl, wps89, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps90[] = {
        (pos2_t){.x = 148, .y = 168, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps90);
    control_set_waypoints(&shared_ctrl, wps90, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps91[] = {
        (pos2_t){.x = 144, .y = 170, .a = 0.0f},
        (pos2_t){.x = 144, .y = 172, .a = 0.0f},
        (pos2_t){.x = 144, .y = 172, .a = 0.0f},
        (pos2_t){.x = 144, .y = 172, .a = 0.0f},
        (pos2_t){.x = 144, .y = 172, .a = 0.0f},
        (pos2_t){.x = 146, .y = 174, .a = 0.0f},
        (pos2_t){.x = 146, .y = 172, .a = 0.0f},
        (pos2_t){.x = 150, .y = 170, .a = 0.0f},
        (pos2_t){.x = 150, .y = 168, .a = 0.0f},
        (pos2_t){.x = 150, .y = 168, .a = 0.0f},
        (pos2_t){.x = 150, .y = 168, .a = 0.0f},
        (pos2_t){.x = 150, .y = 166, .a = 0.0f},
        (pos2_t){.x = 150, .y = 166, .a = 0.0f},
        (pos2_t){.x = 148, .y = 166, .a = 0.0f},
        (pos2_t){.x = 148, .y = 168, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps91);
    control_set_waypoints(&shared_ctrl, wps91, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps92[] = {
        (pos2_t){.x = 100, .y = 142, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps92);
    control_set_waypoints(&shared_ctrl, wps92, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps93[] = {
        (pos2_t){.x = 100, .y = 148, .a = 0.0f},
        (pos2_t){.x = 98, .y = 150, .a = 0.0f},
        (pos2_t){.x = 96, .y = 152, .a = 0.0f},
        (pos2_t){.x = 94, .y = 152, .a = 0.0f},
        (pos2_t){.x = 94, .y = 154, .a = 0.0f},
        (pos2_t){.x = 94, .y = 158, .a = 0.0f},
        (pos2_t){.x = 94, .y = 160, .a = 0.0f},
        (pos2_t){.x = 94, .y = 160, .a = 0.0f},
        (pos2_t){.x = 88, .y = 162, .a = 0.0f},
        (pos2_t){.x = 88, .y = 166, .a = 0.0f},
        (pos2_t){.x = 90, .y = 170, .a = 0.0f},
        (pos2_t){.x = 92, .y = 172, .a = 0.0f},
        (pos2_t){.x = 94, .y = 174, .a = 0.0f},
        (pos2_t){.x = 96, .y = 174, .a = 0.0f},
        (pos2_t){.x = 96, .y = 182, .a = 0.0f},
        (pos2_t){.x = 98, .y = 194, .a = 0.0f},
        (pos2_t){.x = 100, .y = 206, .a = 0.0f},
        (pos2_t){.x = 104, .y = 214, .a = 0.0f},
        (pos2_t){.x = 110, .y = 222, .a = 0.0f},
        (pos2_t){.x = 118, .y = 232, .a = 0.0f},
        (pos2_t){.x = 126, .y = 236, .a = 0.0f},
        (pos2_t){.x = 126, .y = 236, .a = 0.0f},
        (pos2_t){.x = 126, .y = 236, .a = 0.0f},
        (pos2_t){.x = 114, .y = 226, .a = 0.0f},
        (pos2_t){.x = 106, .y = 218, .a = 0.0f},
        (pos2_t){.x = 102, .y = 210, .a = 0.0f},
        (pos2_t){.x = 102, .y = 208, .a = 0.0f},
        (pos2_t){.x = 98, .y = 196, .a = 0.0f},
        (pos2_t){.x = 96, .y = 180, .a = 0.0f},
        (pos2_t){.x = 96, .y = 174, .a = 0.0f},
        (pos2_t){.x = 100, .y = 174, .a = 0.0f},
        (pos2_t){.x = 102, .y = 174, .a = 0.0f},
        (pos2_t){.x = 106, .y = 172, .a = 0.0f},
        (pos2_t){.x = 110, .y = 168, .a = 0.0f},
        (pos2_t){.x = 112, .y = 164, .a = 0.0f},
        (pos2_t){.x = 112, .y = 160, .a = 0.0f},
        (pos2_t){.x = 110, .y = 158, .a = 0.0f},
        (pos2_t){.x = 108, .y = 156, .a = 0.0f},
        (pos2_t){.x = 106, .y = 156, .a = 0.0f},
        (pos2_t){.x = 104, .y = 156, .a = 0.0f},
        (pos2_t){.x = 104, .y = 156, .a = 0.0f},
        (pos2_t){.x = 104, .y = 156, .a = 0.0f},
        (pos2_t){.x = 104, .y = 154, .a = 0.0f},
        (pos2_t){.x = 104, .y = 154, .a = 0.0f},
        (pos2_t){.x = 100, .y = 152, .a = 0.0f},
        (pos2_t){.x = 100, .y = 150, .a = 0.0f},
        (pos2_t){.x = 100, .y = 146, .a = 0.0f},
        (pos2_t){.x = 100, .y = 142, .a = 0.0f},
        (pos2_t){.x = 100, .y = 142, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps93);
    control_set_waypoints(&shared_ctrl, wps93, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps94[] = {
        (pos2_t){.x = 96, .y = 154, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps94);
    control_set_waypoints(&shared_ctrl, wps94, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps95[] = {
        (pos2_t){.x = 94, .y = 154, .a = 0.0f},
        (pos2_t){.x = 96, .y = 158, .a = 0.0f},
        (pos2_t){.x = 96, .y = 158, .a = 0.0f},
        (pos2_t){.x = 98, .y = 160, .a = 0.0f},
        (pos2_t){.x = 96, .y = 160, .a = 0.0f},
        (pos2_t){.x = 96, .y = 160, .a = 0.0f},
        (pos2_t){.x = 94, .y = 162, .a = 0.0f},
        (pos2_t){.x = 90, .y = 164, .a = 0.0f},
        (pos2_t){.x = 90, .y = 166, .a = 0.0f},
        (pos2_t){.x = 92, .y = 170, .a = 0.0f},
        (pos2_t){.x = 98, .y = 172, .a = 0.0f},
        (pos2_t){.x = 102, .y = 172, .a = 0.0f},
        (pos2_t){.x = 108, .y = 168, .a = 0.0f},
        (pos2_t){.x = 108, .y = 166, .a = 0.0f},
        (pos2_t){.x = 110, .y = 162, .a = 0.0f},
        (pos2_t){.x = 108, .y = 158, .a = 0.0f},
        (pos2_t){.x = 104, .y = 158, .a = 0.0f},
        (pos2_t){.x = 102, .y = 158, .a = 0.0f},
        (pos2_t){.x = 102, .y = 156, .a = 0.0f},
        (pos2_t){.x = 100, .y = 154, .a = 0.0f},
        (pos2_t){.x = 98, .y = 152, .a = 0.0f},
        (pos2_t){.x = 96, .y = 154, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps95);
    control_set_waypoints(&shared_ctrl, wps95, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps96[] = {
        (pos2_t){.x = 102, .y = 166, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps96);
    control_set_waypoints(&shared_ctrl, wps96, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps97[] = {
        (pos2_t){.x = 102, .y = 166, .a = 0.0f},
        (pos2_t){.x = 102, .y = 166, .a = 0.0f},
        (pos2_t){.x = 100, .y = 166, .a = 0.0f},
        (pos2_t){.x = 100, .y = 168, .a = 0.0f},
        (pos2_t){.x = 100, .y = 168, .a = 0.0f},
        (pos2_t){.x = 100, .y = 168, .a = 0.0f},
        (pos2_t){.x = 102, .y = 170, .a = 0.0f},
        (pos2_t){.x = 104, .y = 168, .a = 0.0f},
        (pos2_t){.x = 104, .y = 166, .a = 0.0f},
        (pos2_t){.x = 102, .y = 166, .a = 0.0f},
        (pos2_t){.x = 102, .y = 166, .a = 0.0f},
        (pos2_t){.x = 102, .y = 166, .a = 0.0f},
        (pos2_t){.x = 102, .y = 166, .a = 0.0f},
        (pos2_t){.x = 102, .y = 164, .a = 0.0f},
        (pos2_t){.x = 102, .y = 164, .a = 0.0f},
        (pos2_t){.x = 102, .y = 166, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps97);
    control_set_waypoints(&shared_ctrl, wps97, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps98[] = {
        (pos2_t){.x = 108, .y = 160, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps98);
    control_set_waypoints(&shared_ctrl, wps98, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps99[] = {
        (pos2_t){.x = 108, .y = 162, .a = 0.0f},
        (pos2_t){.x = 108, .y = 166, .a = 0.0f},
        (pos2_t){.x = 106, .y = 168, .a = 0.0f},
        (pos2_t){.x = 106, .y = 164, .a = 0.0f},
        (pos2_t){.x = 108, .y = 160, .a = 0.0f},
        (pos2_t){.x = 108, .y = 160, .a = 0.0f},
        (pos2_t){.x = 108, .y = 160, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps99);
    control_set_waypoints(&shared_ctrl, wps99, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps100[] = {
        (pos2_t){.x = 178, .y = 166, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps100);
    control_set_waypoints(&shared_ctrl, wps100, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps101[] = {
        (pos2_t){.x = 176, .y = 168, .a = 0.0f},
        (pos2_t){.x = 174, .y = 172, .a = 0.0f},
        (pos2_t){.x = 172, .y = 182, .a = 0.0f},
        (pos2_t){.x = 172, .y = 192, .a = 0.0f},
        (pos2_t){.x = 176, .y = 208, .a = 0.0f},
        (pos2_t){.x = 178, .y = 210, .a = 0.0f},
        (pos2_t){.x = 180, .y = 212, .a = 0.0f},
        (pos2_t){.x = 184, .y = 212, .a = 0.0f},
        (pos2_t){.x = 190, .y = 208, .a = 0.0f},
        (pos2_t){.x = 192, .y = 200, .a = 0.0f},
        (pos2_t){.x = 194, .y = 190, .a = 0.0f},
        (pos2_t){.x = 194, .y = 182, .a = 0.0f},
        (pos2_t){.x = 190, .y = 172, .a = 0.0f},
        (pos2_t){.x = 186, .y = 166, .a = 0.0f},
        (pos2_t){.x = 182, .y = 164, .a = 0.0f},
        (pos2_t){.x = 178, .y = 166, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps101);
    control_set_waypoints(&shared_ctrl, wps101, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps102[] = {
        (pos2_t){.x = 184, .y = 172, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps102);
    control_set_waypoints(&shared_ctrl, wps102, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps103[] = {
        (pos2_t){.x = 188, .y = 176, .a = 0.0f},
        (pos2_t){.x = 190, .y = 180, .a = 0.0f},
        (pos2_t){.x = 190, .y = 188, .a = 0.0f},
        (pos2_t){.x = 190, .y = 196, .a = 0.0f},
        (pos2_t){.x = 190, .y = 198, .a = 0.0f},
        (pos2_t){.x = 190, .y = 198, .a = 0.0f},
        (pos2_t){.x = 190, .y = 198, .a = 0.0f},
        (pos2_t){.x = 188, .y = 200, .a = 0.0f},
        (pos2_t){.x = 186, .y = 202, .a = 0.0f},
        (pos2_t){.x = 184, .y = 204, .a = 0.0f},
        (pos2_t){.x = 186, .y = 206, .a = 0.0f},
        (pos2_t){.x = 186, .y = 206, .a = 0.0f},
        (pos2_t){.x = 186, .y = 206, .a = 0.0f},
        (pos2_t){.x = 186, .y = 208, .a = 0.0f},
        (pos2_t){.x = 184, .y = 208, .a = 0.0f},
        (pos2_t){.x = 182, .y = 206, .a = 0.0f},
        (pos2_t){.x = 178, .y = 202, .a = 0.0f},
        (pos2_t){.x = 178, .y = 186, .a = 0.0f},
        (pos2_t){.x = 178, .y = 176, .a = 0.0f},
        (pos2_t){.x = 180, .y = 172, .a = 0.0f},
        (pos2_t){.x = 182, .y = 170, .a = 0.0f},
        (pos2_t){.x = 184, .y = 172, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps103);
    control_set_waypoints(&shared_ctrl, wps103, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps104[] = {
        (pos2_t){.x = 186, .y = 194, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps104);
    control_set_waypoints(&shared_ctrl, wps104, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps105[] = {
        (pos2_t){.x = 186, .y = 196, .a = 0.0f},
        (pos2_t){.x = 188, .y = 196, .a = 0.0f},
        (pos2_t){.x = 188, .y = 196, .a = 0.0f},
        (pos2_t){.x = 188, .y = 194, .a = 0.0f},
        (pos2_t){.x = 188, .y = 194, .a = 0.0f},
        (pos2_t){.x = 186, .y = 194, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps105);
    control_set_waypoints(&shared_ctrl, wps105, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps106[] = {
        (pos2_t){.x = 190, .y = 200, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps106);
    control_set_waypoints(&shared_ctrl, wps106, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps107[] = {
        (pos2_t){.x = 190, .y = 204, .a = 0.0f},
        (pos2_t){.x = 188, .y = 206, .a = 0.0f},
        (pos2_t){.x = 186, .y = 204, .a = 0.0f},
        (pos2_t){.x = 188, .y = 202, .a = 0.0f},
        (pos2_t){.x = 188, .y = 200, .a = 0.0f},
        (pos2_t){.x = 190, .y = 200, .a = 0.0f},
        (pos2_t){.x = 190, .y = 200, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps107);
    control_set_waypoints(&shared_ctrl, wps107, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps108[] = {
        (pos2_t){.x = 178, .y = 210, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps108);
    control_set_waypoints(&shared_ctrl, wps108, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps109[] = {
        (pos2_t){.x = 180, .y = 210, .a = 0.0f},
        (pos2_t){.x = 182, .y = 210, .a = 0.0f},
        (pos2_t){.x = 186, .y = 210, .a = 0.0f},
        (pos2_t){.x = 186, .y = 208, .a = 0.0f},
        (pos2_t){.x = 186, .y = 208, .a = 0.0f},
        (pos2_t){.x = 188, .y = 208, .a = 0.0f},
        (pos2_t){.x = 186, .y = 210, .a = 0.0f},
        (pos2_t){.x = 184, .y = 212, .a = 0.0f},
        (pos2_t){.x = 180, .y = 212, .a = 0.0f},
        (pos2_t){.x = 176, .y = 206, .a = 0.0f},
        (pos2_t){.x = 176, .y = 206, .a = 0.0f},
        (pos2_t){.x = 176, .y = 206, .a = 0.0f},
        (pos2_t){.x = 176, .y = 206, .a = 0.0f},
        (pos2_t){.x = 176, .y = 208, .a = 0.0f},
        (pos2_t){.x = 178, .y = 210, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps109);
    control_set_waypoints(&shared_ctrl, wps109, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps110[] = {
        (pos2_t){.x = 108, .y = 182, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps110);
    control_set_waypoints(&shared_ctrl, wps110, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps111[] = {
        (pos2_t){.x = 106, .y = 182, .a = 0.0f},
        (pos2_t){.x = 104, .y = 186, .a = 0.0f},
        (pos2_t){.x = 102, .y = 194, .a = 0.0f},
        (pos2_t){.x = 104, .y = 204, .a = 0.0f},
        (pos2_t){.x = 110, .y = 214, .a = 0.0f},
        (pos2_t){.x = 116, .y = 220, .a = 0.0f},
        (pos2_t){.x = 122, .y = 224, .a = 0.0f},
        (pos2_t){.x = 126, .y = 224, .a = 0.0f},
        (pos2_t){.x = 130, .y = 224, .a = 0.0f},
        (pos2_t){.x = 132, .y = 222, .a = 0.0f},
        (pos2_t){.x = 132, .y = 218, .a = 0.0f},
        (pos2_t){.x = 130, .y = 210, .a = 0.0f},
        (pos2_t){.x = 126, .y = 196, .a = 0.0f},
        (pos2_t){.x = 118, .y = 186, .a = 0.0f},
        (pos2_t){.x = 114, .y = 182, .a = 0.0f},
        (pos2_t){.x = 110, .y = 180, .a = 0.0f},
        (pos2_t){.x = 108, .y = 182, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps111);
    control_set_waypoints(&shared_ctrl, wps111, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps112[] = {
        (pos2_t){.x = 118, .y = 208, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps112);
    control_set_waypoints(&shared_ctrl, wps112, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps113[] = {
        (pos2_t){.x = 118, .y = 208, .a = 0.0f},
        (pos2_t){.x = 118, .y = 210, .a = 0.0f},
        (pos2_t){.x = 120, .y = 210, .a = 0.0f},
        (pos2_t){.x = 120, .y = 210, .a = 0.0f},
        (pos2_t){.x = 120, .y = 208, .a = 0.0f},
        (pos2_t){.x = 120, .y = 208, .a = 0.0f},
        (pos2_t){.x = 120, .y = 208, .a = 0.0f},
        (pos2_t){.x = 120, .y = 208, .a = 0.0f},
        (pos2_t){.x = 118, .y = 208, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps113);
    control_set_waypoints(&shared_ctrl, wps113, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps114[] = {
        (pos2_t){.x = 118, .y = 190, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps114);
    control_set_waypoints(&shared_ctrl, wps114, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps115[] = {
        (pos2_t){.x = 122, .y = 194, .a = 0.0f},
        (pos2_t){.x = 126, .y = 202, .a = 0.0f},
        (pos2_t){.x = 130, .y = 216, .a = 0.0f},
        (pos2_t){.x = 130, .y = 220, .a = 0.0f},
        (pos2_t){.x = 128, .y = 222, .a = 0.0f},
        (pos2_t){.x = 124, .y = 222, .a = 0.0f},
        (pos2_t){.x = 120, .y = 220, .a = 0.0f},
        (pos2_t){.x = 120, .y = 220, .a = 0.0f},
        (pos2_t){.x = 120, .y = 220, .a = 0.0f},
        (pos2_t){.x = 124, .y = 220, .a = 0.0f},
        (pos2_t){.x = 126, .y = 220, .a = 0.0f},
        (pos2_t){.x = 126, .y = 220, .a = 0.0f},
        (pos2_t){.x = 124, .y = 218, .a = 0.0f},
        (pos2_t){.x = 124, .y = 216, .a = 0.0f},
        (pos2_t){.x = 122, .y = 212, .a = 0.0f},
        (pos2_t){.x = 126, .y = 210, .a = 0.0f},
        (pos2_t){.x = 126, .y = 210, .a = 0.0f},
        (pos2_t){.x = 124, .y = 200, .a = 0.0f},
        (pos2_t){.x = 118, .y = 190, .a = 0.0f},
        (pos2_t){.x = 118, .y = 188, .a = 0.0f},
        (pos2_t){.x = 118, .y = 190, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps115);
    control_set_waypoints(&shared_ctrl, wps115, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps116[] = {
        (pos2_t){.x = 142, .y = 226, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps116);
    control_set_waypoints(&shared_ctrl, wps116, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps117[] = {
        (pos2_t){.x = 140, .y = 228, .a = 0.0f},
        (pos2_t){.x = 142, .y = 230, .a = 0.0f},
        (pos2_t){.x = 144, .y = 230, .a = 0.0f},
        (pos2_t){.x = 146, .y = 230, .a = 0.0f},
        (pos2_t){.x = 146, .y = 228, .a = 0.0f},
        (pos2_t){.x = 144, .y = 226, .a = 0.0f},
        (pos2_t){.x = 144, .y = 228, .a = 0.0f},
        (pos2_t){.x = 146, .y = 228, .a = 0.0f},
        (pos2_t){.x = 146, .y = 230, .a = 0.0f},
        (pos2_t){.x = 144, .y = 230, .a = 0.0f},
        (pos2_t){.x = 142, .y = 230, .a = 0.0f},
        (pos2_t){.x = 140, .y = 228, .a = 0.0f},
        (pos2_t){.x = 142, .y = 226, .a = 0.0f},
        (pos2_t){.x = 142, .y = 226, .a = 0.0f},
        (pos2_t){.x = 144, .y = 226, .a = 0.0f},
        (pos2_t){.x = 142, .y = 226, .a = 0.0f},
        (pos2_t){.x = 142, .y = 226, .a = 0.0f},
        (pos2_t){.x = 142, .y = 226, .a = 0.0f},
        (pos2_t){.x = 142, .y = 226, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps117);
    control_set_waypoints(&shared_ctrl, wps117, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps118[] = {
        (pos2_t){.x = 134, .y = 232, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps118);
    control_set_waypoints(&shared_ctrl, wps118, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps119[] = {
        (pos2_t){.x = 132, .y = 232, .a = 0.0f},
        (pos2_t){.x = 130, .y = 234, .a = 0.0f},
        (pos2_t){.x = 130, .y = 236, .a = 0.0f},
        (pos2_t){.x = 130, .y = 238, .a = 0.0f},
        (pos2_t){.x = 130, .y = 238, .a = 0.0f},
        (pos2_t){.x = 130, .y = 238, .a = 0.0f},
        (pos2_t){.x = 130, .y = 236, .a = 0.0f},
        (pos2_t){.x = 130, .y = 236, .a = 0.0f},
        (pos2_t){.x = 130, .y = 236, .a = 0.0f},
        (pos2_t){.x = 132, .y = 234, .a = 0.0f},
        (pos2_t){.x = 132, .y = 234, .a = 0.0f},
        (pos2_t){.x = 132, .y = 234, .a = 0.0f},
        (pos2_t){.x = 136, .y = 232, .a = 0.0f},
        (pos2_t){.x = 140, .y = 234, .a = 0.0f},
        (pos2_t){.x = 144, .y = 234, .a = 0.0f},
        (pos2_t){.x = 146, .y = 236, .a = 0.0f},
        (pos2_t){.x = 148, .y = 240, .a = 0.0f},
        (pos2_t){.x = 148, .y = 242, .a = 0.0f},
        (pos2_t){.x = 148, .y = 242, .a = 0.0f},
        (pos2_t){.x = 148, .y = 242, .a = 0.0f},
        (pos2_t){.x = 148, .y = 242, .a = 0.0f},
        (pos2_t){.x = 148, .y = 242, .a = 0.0f},
        (pos2_t){.x = 148, .y = 240, .a = 0.0f},
        (pos2_t){.x = 148, .y = 238, .a = 0.0f},
        (pos2_t){.x = 146, .y = 236, .a = 0.0f},
        (pos2_t){.x = 144, .y = 234, .a = 0.0f},
        (pos2_t){.x = 138, .y = 232, .a = 0.0f},
        (pos2_t){.x = 134, .y = 232, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps119);
    control_set_waypoints(&shared_ctrl, wps119, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps120[] = {
        (pos2_t){.x = 218, .y = 126, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps120);
    control_set_waypoints(&shared_ctrl, wps120, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps121[] = {
        (pos2_t){.x = 212, .y = 128, .a = 0.0f},
        (pos2_t){.x = 216, .y = 132, .a = 0.0f},
        (pos2_t){.x = 224, .y = 144, .a = 0.0f},
        (pos2_t){.x = 228, .y = 146, .a = 0.0f},
        (pos2_t){.x = 230, .y = 146, .a = 0.0f},
        (pos2_t){.x = 234, .y = 144, .a = 0.0f},
        (pos2_t){.x = 234, .y = 140, .a = 0.0f},
        (pos2_t){.x = 234, .y = 138, .a = 0.0f},
        (pos2_t){.x = 232, .y = 134, .a = 0.0f},
        (pos2_t){.x = 228, .y = 128, .a = 0.0f},
        (pos2_t){.x = 224, .y = 126, .a = 0.0f},
        (pos2_t){.x = 220, .y = 124, .a = 0.0f},
        (pos2_t){.x = 218, .y = 126, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps121);
    control_set_waypoints(&shared_ctrl, wps121, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps122[] = {
        (pos2_t){.x = 222, .y = 128, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps122);
    control_set_waypoints(&shared_ctrl, wps122, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps123[] = {
        (pos2_t){.x = 224, .y = 132, .a = 0.0f},
        (pos2_t){.x = 224, .y = 132, .a = 0.0f},
        (pos2_t){.x = 224, .y = 132, .a = 0.0f},
        (pos2_t){.x = 224, .y = 134, .a = 0.0f},
        (pos2_t){.x = 224, .y = 134, .a = 0.0f},
        (pos2_t){.x = 222, .y = 134, .a = 0.0f},
        (pos2_t){.x = 222, .y = 134, .a = 0.0f},
        (pos2_t){.x = 220, .y = 134, .a = 0.0f},
        (pos2_t){.x = 218, .y = 134, .a = 0.0f},
        (pos2_t){.x = 218, .y = 132, .a = 0.0f},
        (pos2_t){.x = 216, .y = 130, .a = 0.0f},
        (pos2_t){.x = 214, .y = 128, .a = 0.0f},
        (pos2_t){.x = 214, .y = 128, .a = 0.0f},
        (pos2_t){.x = 218, .y = 126, .a = 0.0f},
        (pos2_t){.x = 220, .y = 126, .a = 0.0f},
        (pos2_t){.x = 222, .y = 126, .a = 0.0f},
        (pos2_t){.x = 222, .y = 128, .a = 0.0f},
        (pos2_t){.x = 222, .y = 128, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps123);
    control_set_waypoints(&shared_ctrl, wps123, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps124[] = {
        (pos2_t){.x = 74, .y = 146, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps124);
    control_set_waypoints(&shared_ctrl, wps124, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps125[] = {
        (pos2_t){.x = 72, .y = 150, .a = 0.0f},
        (pos2_t){.x = 70, .y = 158, .a = 0.0f},
        (pos2_t){.x = 70, .y = 160, .a = 0.0f},
        (pos2_t){.x = 74, .y = 162, .a = 0.0f},
        (pos2_t){.x = 76, .y = 162, .a = 0.0f},
        (pos2_t){.x = 78, .y = 160, .a = 0.0f},
        (pos2_t){.x = 80, .y = 152, .a = 0.0f},
        (pos2_t){.x = 82, .y = 148, .a = 0.0f},
        (pos2_t){.x = 80, .y = 148, .a = 0.0f},
        (pos2_t){.x = 76, .y = 146, .a = 0.0f},
        (pos2_t){.x = 74, .y = 146, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps125);
    control_set_waypoints(&shared_ctrl, wps125, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps126[] = {
        (pos2_t){.x = 214, .y = 236, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps126);
    control_set_waypoints(&shared_ctrl, wps126, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps127[] = {
        (pos2_t){.x = 212, .y = 236, .a = 0.0f},
        (pos2_t){.x = 212, .y = 236, .a = 0.0f},
        (pos2_t){.x = 214, .y = 238, .a = 0.0f},
        (pos2_t){.x = 214, .y = 238, .a = 0.0f},
        (pos2_t){.x = 220, .y = 242, .a = 0.0f},
        (pos2_t){.x = 230, .y = 250, .a = 0.0f},
        (pos2_t){.x = 242, .y = 266, .a = 0.0f},
        (pos2_t){.x = 250, .y = 282, .a = 0.0f},
        (pos2_t){.x = 250, .y = 288, .a = 0.0f},
        (pos2_t){.x = 252, .y = 296, .a = 0.0f},
        (pos2_t){.x = 250, .y = 302, .a = 0.0f},
        (pos2_t){.x = 248, .y = 308, .a = 0.0f},
        (pos2_t){.x = 246, .y = 312, .a = 0.0f},
        (pos2_t){.x = 242, .y = 314, .a = 0.0f},
        (pos2_t){.x = 238, .y = 316, .a = 0.0f},
        (pos2_t){.x = 234, .y = 316, .a = 0.0f},
        (pos2_t){.x = 228, .y = 314, .a = 0.0f},
        (pos2_t){.x = 222, .y = 310, .a = 0.0f},
        (pos2_t){.x = 218, .y = 306, .a = 0.0f},
        (pos2_t){.x = 212, .y = 300, .a = 0.0f},
        (pos2_t){.x = 202, .y = 286, .a = 0.0f},
        (pos2_t){.x = 194, .y = 272, .a = 0.0f},
        (pos2_t){.x = 188, .y = 262, .a = 0.0f},
        (pos2_t){.x = 186, .y = 258, .a = 0.0f},
        (pos2_t){.x = 186, .y = 258, .a = 0.0f},
        (pos2_t){.x = 186, .y = 258, .a = 0.0f},
        (pos2_t){.x = 186, .y = 258, .a = 0.0f},
        (pos2_t){.x = 184, .y = 258, .a = 0.0f},
        (pos2_t){.x = 182, .y = 262, .a = 0.0f},
        (pos2_t){.x = 182, .y = 268, .a = 0.0f},
        (pos2_t){.x = 184, .y = 274, .a = 0.0f},
        (pos2_t){.x = 188, .y = 280, .a = 0.0f},
        (pos2_t){.x = 204, .y = 304, .a = 0.0f},
        (pos2_t){.x = 214, .y = 316, .a = 0.0f},
        (pos2_t){.x = 224, .y = 324, .a = 0.0f},
        (pos2_t){.x = 230, .y = 326, .a = 0.0f},
        (pos2_t){.x = 234, .y = 328, .a = 0.0f},
        (pos2_t){.x = 238, .y = 328, .a = 0.0f},
        (pos2_t){.x = 244, .y = 326, .a = 0.0f},
        (pos2_t){.x = 248, .y = 322, .a = 0.0f},
        (pos2_t){.x = 252, .y = 318, .a = 0.0f},
        (pos2_t){.x = 254, .y = 310, .a = 0.0f},
        (pos2_t){.x = 256, .y = 302, .a = 0.0f},
        (pos2_t){.x = 254, .y = 292, .a = 0.0f},
        (pos2_t){.x = 252, .y = 280, .a = 0.0f},
        (pos2_t){.x = 246, .y = 268, .a = 0.0f},
        (pos2_t){.x = 238, .y = 254, .a = 0.0f},
        (pos2_t){.x = 226, .y = 244, .a = 0.0f},
        (pos2_t){.x = 218, .y = 236, .a = 0.0f},
        (pos2_t){.x = 214, .y = 236, .a = 0.0f},
        (pos2_t){.x = 214, .y = 236, .a = 0.0f},
        (pos2_t){.x = 214, .y = 236, .a = 0.0f},
        (pos2_t){.x = 214, .y = 236, .a = 0.0f},
        (pos2_t){.x = 214, .y = 236, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps127);
    control_set_waypoints(&shared_ctrl, wps127, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps128[] = {
        (pos2_t){.x = 210, .y = 240, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps128);
    control_set_waypoints(&shared_ctrl, wps128, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps129[] = {
        (pos2_t){.x = 196, .y = 252, .a = 0.0f},
        (pos2_t){.x = 190, .y = 254, .a = 0.0f},
        (pos2_t){.x = 188, .y = 258, .a = 0.0f},
        (pos2_t){.x = 188, .y = 260, .a = 0.0f},
        (pos2_t){.x = 194, .y = 268, .a = 0.0f},
        (pos2_t){.x = 204, .y = 286, .a = 0.0f},
        (pos2_t){.x = 212, .y = 298, .a = 0.0f},
        (pos2_t){.x = 220, .y = 308, .a = 0.0f},
        (pos2_t){.x = 228, .y = 312, .a = 0.0f},
        (pos2_t){.x = 232, .y = 314, .a = 0.0f},
        (pos2_t){.x = 236, .y = 314, .a = 0.0f},
        (pos2_t){.x = 242, .y = 312, .a = 0.0f},
        (pos2_t){.x = 246, .y = 310, .a = 0.0f},
        (pos2_t){.x = 248, .y = 306, .a = 0.0f},
        (pos2_t){.x = 250, .y = 302, .a = 0.0f},
        (pos2_t){.x = 250, .y = 296, .a = 0.0f},
        (pos2_t){.x = 250, .y = 288, .a = 0.0f},
        (pos2_t){.x = 248, .y = 282, .a = 0.0f},
        (pos2_t){.x = 242, .y = 268, .a = 0.0f},
        (pos2_t){.x = 232, .y = 254, .a = 0.0f},
        (pos2_t){.x = 224, .y = 246, .a = 0.0f},
        (pos2_t){.x = 216, .y = 240, .a = 0.0f},
        (pos2_t){.x = 214, .y = 238, .a = 0.0f},
        (pos2_t){.x = 212, .y = 238, .a = 0.0f},
        (pos2_t){.x = 210, .y = 240, .a = 0.0f},
        (pos2_t){.x = 210, .y = 240, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps129);
    control_set_waypoints(&shared_ctrl, wps129, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps130[] = {
        (pos2_t){.x = 126, .y = 262, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps130);
    control_set_waypoints(&shared_ctrl, wps130, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps131[] = {
        (pos2_t){.x = 122, .y = 270, .a = 0.0f},
        (pos2_t){.x = 120, .y = 286, .a = 0.0f},
        (pos2_t){.x = 120, .y = 296, .a = 0.0f},
        (pos2_t){.x = 120, .y = 306, .a = 0.0f},
        (pos2_t){.x = 122, .y = 318, .a = 0.0f},
        (pos2_t){.x = 126, .y = 328, .a = 0.0f},
        (pos2_t){.x = 130, .y = 338, .a = 0.0f},
        (pos2_t){.x = 136, .y = 344, .a = 0.0f},
        (pos2_t){.x = 144, .y = 350, .a = 0.0f},
        (pos2_t){.x = 152, .y = 352, .a = 0.0f},
        (pos2_t){.x = 160, .y = 350, .a = 0.0f},
        (pos2_t){.x = 164, .y = 346, .a = 0.0f},
        (pos2_t){.x = 168, .y = 342, .a = 0.0f},
        (pos2_t){.x = 170, .y = 336, .a = 0.0f},
        (pos2_t){.x = 174, .y = 322, .a = 0.0f},
        (pos2_t){.x = 176, .y = 312, .a = 0.0f},
        (pos2_t){.x = 176, .y = 300, .a = 0.0f},
        (pos2_t){.x = 176, .y = 286, .a = 0.0f},
        (pos2_t){.x = 174, .y = 276, .a = 0.0f},
        (pos2_t){.x = 172, .y = 268, .a = 0.0f},
        (pos2_t){.x = 170, .y = 264, .a = 0.0f},
        (pos2_t){.x = 168, .y = 264, .a = 0.0f},
        (pos2_t){.x = 166, .y = 262, .a = 0.0f},
        (pos2_t){.x = 164, .y = 264, .a = 0.0f},
        (pos2_t){.x = 166, .y = 280, .a = 0.0f},
        (pos2_t){.x = 166, .y = 302, .a = 0.0f},
        (pos2_t){.x = 164, .y = 322, .a = 0.0f},
        (pos2_t){.x = 162, .y = 328, .a = 0.0f},
        (pos2_t){.x = 160, .y = 334, .a = 0.0f},
        (pos2_t){.x = 152, .y = 340, .a = 0.0f},
        (pos2_t){.x = 148, .y = 340, .a = 0.0f},
        (pos2_t){.x = 144, .y = 340, .a = 0.0f},
        (pos2_t){.x = 138, .y = 338, .a = 0.0f},
        (pos2_t){.x = 134, .y = 334, .a = 0.0f},
        (pos2_t){.x = 128, .y = 324, .a = 0.0f},
        (pos2_t){.x = 124, .y = 312, .a = 0.0f},
        (pos2_t){.x = 122, .y = 296, .a = 0.0f},
        (pos2_t){.x = 122, .y = 286, .a = 0.0f},
        (pos2_t){.x = 124, .y = 278, .a = 0.0f},
        (pos2_t){.x = 128, .y = 262, .a = 0.0f},
        (pos2_t){.x = 128, .y = 260, .a = 0.0f},
        (pos2_t){.x = 126, .y = 262, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps131);
    control_set_waypoints(&shared_ctrl, wps131, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps132[] = {
        (pos2_t){.x = 130, .y = 264, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps132);
    control_set_waypoints(&shared_ctrl, wps132, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps133[] = {
        (pos2_t){.x = 126, .y = 270, .a = 0.0f},
        (pos2_t){.x = 124, .y = 282, .a = 0.0f},
        (pos2_t){.x = 124, .y = 294, .a = 0.0f},
        (pos2_t){.x = 124, .y = 306, .a = 0.0f},
        (pos2_t){.x = 126, .y = 314, .a = 0.0f},
        (pos2_t){.x = 128, .y = 322, .a = 0.0f},
        (pos2_t){.x = 132, .y = 328, .a = 0.0f},
        (pos2_t){.x = 134, .y = 334, .a = 0.0f},
        (pos2_t){.x = 138, .y = 336, .a = 0.0f},
        (pos2_t){.x = 142, .y = 338, .a = 0.0f},
        (pos2_t){.x = 148, .y = 338, .a = 0.0f},
        (pos2_t){.x = 152, .y = 338, .a = 0.0f},
        (pos2_t){.x = 156, .y = 334, .a = 0.0f},
        (pos2_t){.x = 160, .y = 330, .a = 0.0f},
        (pos2_t){.x = 162, .y = 326, .a = 0.0f},
        (pos2_t){.x = 164, .y = 320, .a = 0.0f},
        (pos2_t){.x = 164, .y = 300, .a = 0.0f},
        (pos2_t){.x = 164, .y = 280, .a = 0.0f},
        (pos2_t){.x = 162, .y = 266, .a = 0.0f},
        (pos2_t){.x = 162, .y = 264, .a = 0.0f},
        (pos2_t){.x = 162, .y = 264, .a = 0.0f},
        (pos2_t){.x = 154, .y = 264, .a = 0.0f},
        (pos2_t){.x = 144, .y = 264, .a = 0.0f},
        (pos2_t){.x = 132, .y = 262, .a = 0.0f},
        (pos2_t){.x = 130, .y = 260, .a = 0.0f},
        (pos2_t){.x = 130, .y = 264, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps133);
    control_set_waypoints(&shared_ctrl, wps133, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps134[] = {
        (pos2_t){.x = 134, .y = 286, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps134);
    control_set_waypoints(&shared_ctrl, wps134, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps135[] = {
        (pos2_t){.x = 136, .y = 308, .a = 0.0f},
        (pos2_t){.x = 136, .y = 308, .a = 0.0f},
        (pos2_t){.x = 136, .y = 308, .a = 0.0f},
        (pos2_t){.x = 136, .y = 308, .a = 0.0f},
        (pos2_t){.x = 136, .y = 306, .a = 0.0f},
        (pos2_t){.x = 136, .y = 292, .a = 0.0f},
        (pos2_t){.x = 136, .y = 282, .a = 0.0f},
        (pos2_t){.x = 134, .y = 286, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps135);
    control_set_waypoints(&shared_ctrl, wps135, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps136[] = {
        (pos2_t){.x = 186, .y = 284, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps136);
    control_set_waypoints(&shared_ctrl, wps136, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps137[] = {
        (pos2_t){.x = 186, .y = 292, .a = 0.0f},
        (pos2_t){.x = 192, .y = 310, .a = 0.0f},
        (pos2_t){.x = 196, .y = 322, .a = 0.0f},
        (pos2_t){.x = 200, .y = 332, .a = 0.0f},
        (pos2_t){.x = 204, .y = 340, .a = 0.0f},
        (pos2_t){.x = 210, .y = 348, .a = 0.0f},
        (pos2_t){.x = 214, .y = 356, .a = 0.0f},
        (pos2_t){.x = 226, .y = 372, .a = 0.0f},
        (pos2_t){.x = 238, .y = 386, .a = 0.0f},
        (pos2_t){.x = 246, .y = 392, .a = 0.0f},
        (pos2_t){.x = 246, .y = 392, .a = 0.0f},
        (pos2_t){.x = 244, .y = 388, .a = 0.0f},
        (pos2_t){.x = 240, .y = 380, .a = 0.0f},
        (pos2_t){.x = 236, .y = 376, .a = 0.0f},
        (pos2_t){.x = 228, .y = 360, .a = 0.0f},
        (pos2_t){.x = 226, .y = 352, .a = 0.0f},
        (pos2_t){.x = 224, .y = 344, .a = 0.0f},
        (pos2_t){.x = 224, .y = 334, .a = 0.0f},
        (pos2_t){.x = 224, .y = 326, .a = 0.0f},
        (pos2_t){.x = 222, .y = 326, .a = 0.0f},
        (pos2_t){.x = 202, .y = 306, .a = 0.0f},
        (pos2_t){.x = 188, .y = 286, .a = 0.0f},
        (pos2_t){.x = 186, .y = 282, .a = 0.0f},
        (pos2_t){.x = 186, .y = 282, .a = 0.0f},
        (pos2_t){.x = 186, .y = 284, .a = 0.0f},
        (pos2_t){.x = 186, .y = 284, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps137);
    control_set_waypoints(&shared_ctrl, wps137, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps138[] = {
        (pos2_t){.x = 256, .y = 294, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps138);
    control_set_waypoints(&shared_ctrl, wps138, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps139[] = {
        (pos2_t){.x = 256, .y = 298, .a = 0.0f},
        (pos2_t){.x = 258, .y = 304, .a = 0.0f},
        (pos2_t){.x = 256, .y = 310, .a = 0.0f},
        (pos2_t){.x = 256, .y = 314, .a = 0.0f},
        (pos2_t){.x = 252, .y = 320, .a = 0.0f},
        (pos2_t){.x = 248, .y = 324, .a = 0.0f},
        (pos2_t){.x = 244, .y = 328, .a = 0.0f},
        (pos2_t){.x = 236, .y = 330, .a = 0.0f},
        (pos2_t){.x = 232, .y = 330, .a = 0.0f},
        (pos2_t){.x = 228, .y = 330, .a = 0.0f},
        (pos2_t){.x = 226, .y = 328, .a = 0.0f},
        (pos2_t){.x = 224, .y = 328, .a = 0.0f},
        (pos2_t){.x = 224, .y = 334, .a = 0.0f},
        (pos2_t){.x = 226, .y = 342, .a = 0.0f},
        (pos2_t){.x = 226, .y = 348, .a = 0.0f},
        (pos2_t){.x = 232, .y = 364, .a = 0.0f},
        (pos2_t){.x = 244, .y = 386, .a = 0.0f},
        (pos2_t){.x = 250, .y = 394, .a = 0.0f},
        (pos2_t){.x = 250, .y = 394, .a = 0.0f},
        (pos2_t){.x = 250, .y = 392, .a = 0.0f},
        (pos2_t){.x = 250, .y = 384, .a = 0.0f},
        (pos2_t){.x = 250, .y = 376, .a = 0.0f},
        (pos2_t){.x = 252, .y = 366, .a = 0.0f},
        (pos2_t){.x = 254, .y = 356, .a = 0.0f},
        (pos2_t){.x = 258, .y = 348, .a = 0.0f},
        (pos2_t){.x = 266, .y = 340, .a = 0.0f},
        (pos2_t){.x = 274, .y = 334, .a = 0.0f},
        (pos2_t){.x = 282, .y = 328, .a = 0.0f},
        (pos2_t){.x = 292, .y = 326, .a = 0.0f},
        (pos2_t){.x = 294, .y = 324, .a = 0.0f},
        (pos2_t){.x = 292, .y = 322, .a = 0.0f},
        (pos2_t){.x = 290, .y = 314, .a = 0.0f},
        (pos2_t){.x = 284, .y = 308, .a = 0.0f},
        (pos2_t){.x = 278, .y = 302, .a = 0.0f},
        (pos2_t){.x = 270, .y = 298, .a = 0.0f},
        (pos2_t){.x = 262, .y = 296, .a = 0.0f},
        (pos2_t){.x = 256, .y = 294, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps139);
    control_set_waypoints(&shared_ctrl, wps139, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps140[] = {
        (pos2_t){.x = 18, .y = 92, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps140);
    control_set_waypoints(&shared_ctrl, wps140, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps141[] = {
        (pos2_t){.x = 14, .y = 94, .a = 0.0f},
        (pos2_t){.x = 12, .y = 98, .a = 0.0f},
        (pos2_t){.x = 12, .y = 100, .a = 0.0f},
        (pos2_t){.x = 14, .y = 104, .a = 0.0f},
        (pos2_t){.x = 18, .y = 106, .a = 0.0f},
        (pos2_t){.x = 24, .y = 106, .a = 0.0f},
        (pos2_t){.x = 28, .y = 104, .a = 0.0f},
        (pos2_t){.x = 32, .y = 100, .a = 0.0f},
        (pos2_t){.x = 32, .y = 98, .a = 0.0f},
        (pos2_t){.x = 30, .y = 94, .a = 0.0f},
        (pos2_t){.x = 24, .y = 92, .a = 0.0f},
        (pos2_t){.x = 22, .y = 92, .a = 0.0f},
        (pos2_t){.x = 18, .y = 92, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps141);
    control_set_waypoints(&shared_ctrl, wps141, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps142[] = {
        (pos2_t){.x = 18, .y = 96, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps142);
    control_set_waypoints(&shared_ctrl, wps142, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps143[] = {
        (pos2_t){.x = 16, .y = 98, .a = 0.0f},
        (pos2_t){.x = 16, .y = 102, .a = 0.0f},
        (pos2_t){.x = 18, .y = 104, .a = 0.0f},
        (pos2_t){.x = 22, .y = 104, .a = 0.0f},
        (pos2_t){.x = 26, .y = 104, .a = 0.0f},
        (pos2_t){.x = 28, .y = 100, .a = 0.0f},
        (pos2_t){.x = 28, .y = 98, .a = 0.0f},
        (pos2_t){.x = 28, .y = 96, .a = 0.0f},
        (pos2_t){.x = 24, .y = 94, .a = 0.0f},
        (pos2_t){.x = 18, .y = 96, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps143);
    control_set_waypoints(&shared_ctrl, wps143, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps144[] = {
        (pos2_t){.x = 26, .y = 98, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps144);
    control_set_waypoints(&shared_ctrl, wps144, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps145[] = {
        (pos2_t){.x = 28, .y = 98, .a = 0.0f},
        (pos2_t){.x = 28, .y = 100, .a = 0.0f},
        (pos2_t){.x = 26, .y = 100, .a = 0.0f},
        (pos2_t){.x = 22, .y = 98, .a = 0.0f},
        (pos2_t){.x = 22, .y = 98, .a = 0.0f},
        (pos2_t){.x = 22, .y = 98, .a = 0.0f},
        (pos2_t){.x = 24, .y = 98, .a = 0.0f},
        (pos2_t){.x = 24, .y = 96, .a = 0.0f},
        (pos2_t){.x = 24, .y = 96, .a = 0.0f},
        (pos2_t){.x = 24, .y = 96, .a = 0.0f},
        (pos2_t){.x = 26, .y = 98, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps145);
    control_set_waypoints(&shared_ctrl, wps145, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps146[] = {
        (pos2_t){.x = 252, .y = 108, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps146);
    control_set_waypoints(&shared_ctrl, wps146, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps147[] = {
        (pos2_t){.x = 248, .y = 110, .a = 0.0f},
        (pos2_t){.x = 246, .y = 114, .a = 0.0f},
        (pos2_t){.x = 244, .y = 116, .a = 0.0f},
        (pos2_t){.x = 246, .y = 120, .a = 0.0f},
        (pos2_t){.x = 250, .y = 124, .a = 0.0f},
        (pos2_t){.x = 254, .y = 126, .a = 0.0f},
        (pos2_t){.x = 260, .y = 126, .a = 0.0f},
        (pos2_t){.x = 264, .y = 122, .a = 0.0f},
        (pos2_t){.x = 266, .y = 118, .a = 0.0f},
        (pos2_t){.x = 266, .y = 112, .a = 0.0f},
        (pos2_t){.x = 262, .y = 108, .a = 0.0f},
        (pos2_t){.x = 256, .y = 108, .a = 0.0f},
        (pos2_t){.x = 252, .y = 108, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps147);
    control_set_waypoints(&shared_ctrl, wps147, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps148[] = {
        (pos2_t){.x = 256, .y = 110, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps148);
    control_set_waypoints(&shared_ctrl, wps148, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps149[] = {
        (pos2_t){.x = 250, .y = 112, .a = 0.0f},
        (pos2_t){.x = 248, .y = 114, .a = 0.0f},
        (pos2_t){.x = 248, .y = 116, .a = 0.0f},
        (pos2_t){.x = 248, .y = 120, .a = 0.0f},
        (pos2_t){.x = 250, .y = 122, .a = 0.0f},
        (pos2_t){.x = 252, .y = 122, .a = 0.0f},
        (pos2_t){.x = 256, .y = 122, .a = 0.0f},
        (pos2_t){.x = 262, .y = 122, .a = 0.0f},
        (pos2_t){.x = 264, .y = 120, .a = 0.0f},
        (pos2_t){.x = 264, .y = 118, .a = 0.0f},
        (pos2_t){.x = 262, .y = 112, .a = 0.0f},
        (pos2_t){.x = 258, .y = 110, .a = 0.0f},
        (pos2_t){.x = 256, .y = 110, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps149);
    control_set_waypoints(&shared_ctrl, wps149, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps150[] = {
        (pos2_t){.x = 262, .y = 116, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps150);
    control_set_waypoints(&shared_ctrl, wps150, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps151[] = {
        (pos2_t){.x = 262, .y = 120, .a = 0.0f},
        (pos2_t){.x = 260, .y = 120, .a = 0.0f},
        (pos2_t){.x = 260, .y = 120, .a = 0.0f},
        (pos2_t){.x = 260, .y = 120, .a = 0.0f},
        (pos2_t){.x = 260, .y = 118, .a = 0.0f},
        (pos2_t){.x = 260, .y = 118, .a = 0.0f},
        (pos2_t){.x = 262, .y = 116, .a = 0.0f},
        (pos2_t){.x = 262, .y = 116, .a = 0.0f},
        (pos2_t){.x = 262, .y = 116, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps151);
    control_set_waypoints(&shared_ctrl, wps151, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps152[] = {
        (pos2_t){.x = 254, .y = 134, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps152);
    control_set_waypoints(&shared_ctrl, wps152, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps153[] = {
        (pos2_t){.x = 252, .y = 134, .a = 0.0f},
        (pos2_t){.x = 250, .y = 136, .a = 0.0f},
        (pos2_t){.x = 250, .y = 138, .a = 0.0f},
        (pos2_t){.x = 250, .y = 140, .a = 0.0f},
        (pos2_t){.x = 252, .y = 146, .a = 0.0f},
        (pos2_t){.x = 256, .y = 146, .a = 0.0f},
        (pos2_t){.x = 258, .y = 146, .a = 0.0f},
        (pos2_t){.x = 260, .y = 144, .a = 0.0f},
        (pos2_t){.x = 262, .y = 140, .a = 0.0f},
        (pos2_t){.x = 262, .y = 138, .a = 0.0f},
        (pos2_t){.x = 260, .y = 134, .a = 0.0f},
        (pos2_t){.x = 258, .y = 134, .a = 0.0f},
        (pos2_t){.x = 254, .y = 134, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps153);
    control_set_waypoints(&shared_ctrl, wps153, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps154[] = {
        (pos2_t){.x = 254, .y = 136, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps154);
    control_set_waypoints(&shared_ctrl, wps154, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps155[] = {
        (pos2_t){.x = 252, .y = 138, .a = 0.0f},
        (pos2_t){.x = 252, .y = 140, .a = 0.0f},
        (pos2_t){.x = 254, .y = 142, .a = 0.0f},
        (pos2_t){.x = 256, .y = 144, .a = 0.0f},
        (pos2_t){.x = 258, .y = 142, .a = 0.0f},
        (pos2_t){.x = 258, .y = 140, .a = 0.0f},
        (pos2_t){.x = 258, .y = 136, .a = 0.0f},
        (pos2_t){.x = 256, .y = 136, .a = 0.0f},
        (pos2_t){.x = 254, .y = 136, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps155);
    control_set_waypoints(&shared_ctrl, wps155, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps156[] = {
        (pos2_t){.x = 56, .y = 262, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps156);
    control_set_waypoints(&shared_ctrl, wps156, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps157[] = {
        (pos2_t){.x = 52, .y = 264, .a = 0.0f},
        (pos2_t){.x = 42, .y = 272, .a = 0.0f},
        (pos2_t){.x = 24, .y = 292, .a = 0.0f},
        (pos2_t){.x = 20, .y = 294, .a = 0.0f},
        (pos2_t){.x = 16, .y = 296, .a = 0.0f},
        (pos2_t){.x = 14, .y = 298, .a = 0.0f},
        (pos2_t){.x = 12, .y = 300, .a = 0.0f},
        (pos2_t){.x = 12, .y = 304, .a = 0.0f},
        (pos2_t){.x = 16, .y = 316, .a = 0.0f},
        (pos2_t){.x = 18, .y = 322, .a = 0.0f},
        (pos2_t){.x = 20, .y = 326, .a = 0.0f},
        (pos2_t){.x = 24, .y = 328, .a = 0.0f},
        (pos2_t){.x = 26, .y = 328, .a = 0.0f},
        (pos2_t){.x = 30, .y = 326, .a = 0.0f},
        (pos2_t){.x = 34, .y = 322, .a = 0.0f},
        (pos2_t){.x = 38, .y = 316, .a = 0.0f},
        (pos2_t){.x = 40, .y = 308, .a = 0.0f},
        (pos2_t){.x = 44, .y = 294, .a = 0.0f},
        (pos2_t){.x = 46, .y = 284, .a = 0.0f},
        (pos2_t){.x = 46, .y = 278, .a = 0.0f},
        (pos2_t){.x = 50, .y = 276, .a = 0.0f},
        (pos2_t){.x = 56, .y = 270, .a = 0.0f},
        (pos2_t){.x = 58, .y = 266, .a = 0.0f},
        (pos2_t){.x = 58, .y = 262, .a = 0.0f},
        (pos2_t){.x = 56, .y = 262, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps157);
    control_set_waypoints(&shared_ctrl, wps157, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps158[] = {
        (pos2_t){.x = 36, .y = 292, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps158);
    control_set_waypoints(&shared_ctrl, wps158, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps159[] = {
        (pos2_t){.x = 34, .y = 298, .a = 0.0f},
        (pos2_t){.x = 32, .y = 312, .a = 0.0f},
        (pos2_t){.x = 30, .y = 316, .a = 0.0f},
        (pos2_t){.x = 28, .y = 314, .a = 0.0f},
        (pos2_t){.x = 26, .y = 306, .a = 0.0f},
        (pos2_t){.x = 26, .y = 302, .a = 0.0f},
        (pos2_t){.x = 26, .y = 298, .a = 0.0f},
        (pos2_t){.x = 36, .y = 290, .a = 0.0f},
        (pos2_t){.x = 36, .y = 292, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps159);
    control_set_waypoints(&shared_ctrl, wps159, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps160[] = {
        (pos2_t){.x = 56, .y = 298, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps160);
    control_set_waypoints(&shared_ctrl, wps160, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps161[] = {
        (pos2_t){.x = 52, .y = 302, .a = 0.0f},
        (pos2_t){.x = 46, .y = 308, .a = 0.0f},
        (pos2_t){.x = 42, .y = 314, .a = 0.0f},
        (pos2_t){.x = 40, .y = 324, .a = 0.0f},
        (pos2_t){.x = 38, .y = 336, .a = 0.0f},
        (pos2_t){.x = 36, .y = 344, .a = 0.0f},
        (pos2_t){.x = 38, .y = 348, .a = 0.0f},
        (pos2_t){.x = 40, .y = 350, .a = 0.0f},
        (pos2_t){.x = 42, .y = 350, .a = 0.0f},
        (pos2_t){.x = 48, .y = 346, .a = 0.0f},
        (pos2_t){.x = 56, .y = 338, .a = 0.0f},
        (pos2_t){.x = 66, .y = 324, .a = 0.0f},
        (pos2_t){.x = 72, .y = 314, .a = 0.0f},
        (pos2_t){.x = 72, .y = 308, .a = 0.0f},
        (pos2_t){.x = 72, .y = 302, .a = 0.0f},
        (pos2_t){.x = 68, .y = 298, .a = 0.0f},
        (pos2_t){.x = 62, .y = 298, .a = 0.0f},
        (pos2_t){.x = 56, .y = 298, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps161);
    control_set_waypoints(&shared_ctrl, wps161, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps162[] = {
        (pos2_t){.x = 58, .y = 306, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps162);
    control_set_waypoints(&shared_ctrl, wps162, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps163[] = {
        (pos2_t){.x = 54, .y = 310, .a = 0.0f},
        (pos2_t){.x = 50, .y = 316, .a = 0.0f},
        (pos2_t){.x = 46, .y = 326, .a = 0.0f},
        (pos2_t){.x = 44, .y = 332, .a = 0.0f},
        (pos2_t){.x = 46, .y = 336, .a = 0.0f},
        (pos2_t){.x = 48, .y = 336, .a = 0.0f},
        (pos2_t){.x = 50, .y = 334, .a = 0.0f},
        (pos2_t){.x = 54, .y = 332, .a = 0.0f},
        (pos2_t){.x = 60, .y = 324, .a = 0.0f},
        (pos2_t){.x = 64, .y = 316, .a = 0.0f},
        (pos2_t){.x = 66, .y = 312, .a = 0.0f},
        (pos2_t){.x = 66, .y = 308, .a = 0.0f},
        (pos2_t){.x = 62, .y = 306, .a = 0.0f},
        (pos2_t){.x = 58, .y = 306, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps163);
    control_set_waypoints(&shared_ctrl, wps163, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps164[] = {
        (pos2_t){.x = 78, .y = 316, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps164);
    control_set_waypoints(&shared_ctrl, wps164, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps165[] = {
        (pos2_t){.x = 74, .y = 322, .a = 0.0f},
        (pos2_t){.x = 66, .y = 330, .a = 0.0f},
        (pos2_t){.x = 52, .y = 352, .a = 0.0f},
        (pos2_t){.x = 50, .y = 354, .a = 0.0f},
        (pos2_t){.x = 50, .y = 358, .a = 0.0f},
        (pos2_t){.x = 52, .y = 362, .a = 0.0f},
        (pos2_t){.x = 52, .y = 362, .a = 0.0f},
        (pos2_t){.x = 54, .y = 360, .a = 0.0f},
        (pos2_t){.x = 60, .y = 352, .a = 0.0f},
        (pos2_t){.x = 66, .y = 344, .a = 0.0f},
        (pos2_t){.x = 66, .y = 350, .a = 0.0f},
        (pos2_t){.x = 62, .y = 366, .a = 0.0f},
        (pos2_t){.x = 62, .y = 376, .a = 0.0f},
        (pos2_t){.x = 64, .y = 378, .a = 0.0f},
        (pos2_t){.x = 66, .y = 378, .a = 0.0f},
        (pos2_t){.x = 68, .y = 376, .a = 0.0f},
        (pos2_t){.x = 70, .y = 370, .a = 0.0f},
        (pos2_t){.x = 72, .y = 358, .a = 0.0f},
        (pos2_t){.x = 76, .y = 338, .a = 0.0f},
        (pos2_t){.x = 80, .y = 338, .a = 0.0f},
        (pos2_t){.x = 86, .y = 342, .a = 0.0f},
        (pos2_t){.x = 90, .y = 342, .a = 0.0f},
        (pos2_t){.x = 92, .y = 340, .a = 0.0f},
        (pos2_t){.x = 94, .y = 338, .a = 0.0f},
        (pos2_t){.x = 94, .y = 336, .a = 0.0f},
        (pos2_t){.x = 90, .y = 332, .a = 0.0f},
        (pos2_t){.x = 84, .y = 330, .a = 0.0f},
        (pos2_t){.x = 78, .y = 328, .a = 0.0f},
        (pos2_t){.x = 80, .y = 326, .a = 0.0f},
        (pos2_t){.x = 84, .y = 320, .a = 0.0f},
        (pos2_t){.x = 84, .y = 320, .a = 0.0f},
        (pos2_t){.x = 84, .y = 320, .a = 0.0f},
        (pos2_t){.x = 84, .y = 318, .a = 0.0f},
        (pos2_t){.x = 82, .y = 316, .a = 0.0f},
        (pos2_t){.x = 80, .y = 316, .a = 0.0f},
        (pos2_t){.x = 78, .y = 316, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps165);
    control_set_waypoints(&shared_ctrl, wps165, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps166[] = {
        (pos2_t){.x = 98, .y = 340, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps166);
    control_set_waypoints(&shared_ctrl, wps166, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps167[] = {
        (pos2_t){.x = 96, .y = 342, .a = 0.0f},
        (pos2_t){.x = 94, .y = 344, .a = 0.0f},
        (pos2_t){.x = 96, .y = 348, .a = 0.0f},
        (pos2_t){.x = 98, .y = 352, .a = 0.0f},
        (pos2_t){.x = 92, .y = 360, .a = 0.0f},
        (pos2_t){.x = 86, .y = 370, .a = 0.0f},
        (pos2_t){.x = 84, .y = 374, .a = 0.0f},
        (pos2_t){.x = 84, .y = 372, .a = 0.0f},
        (pos2_t){.x = 82, .y = 370, .a = 0.0f},
        (pos2_t){.x = 82, .y = 370, .a = 0.0f},
        (pos2_t){.x = 80, .y = 370, .a = 0.0f},
        (pos2_t){.x = 78, .y = 370, .a = 0.0f},
        (pos2_t){.x = 76, .y = 372, .a = 0.0f},
        (pos2_t){.x = 74, .y = 374, .a = 0.0f},
        (pos2_t){.x = 74, .y = 376, .a = 0.0f},
        (pos2_t){.x = 78, .y = 388, .a = 0.0f},
        (pos2_t){.x = 82, .y = 394, .a = 0.0f},
        (pos2_t){.x = 86, .y = 396, .a = 0.0f},
        (pos2_t){.x = 90, .y = 394, .a = 0.0f},
        (pos2_t){.x = 94, .y = 390, .a = 0.0f},
        (pos2_t){.x = 94, .y = 388, .a = 0.0f},
        (pos2_t){.x = 92, .y = 386, .a = 0.0f},
        (pos2_t){.x = 90, .y = 382, .a = 0.0f},
        (pos2_t){.x = 90, .y = 378, .a = 0.0f},
        (pos2_t){.x = 100, .y = 362, .a = 0.0f},
        (pos2_t){.x = 102, .y = 358, .a = 0.0f},
        (pos2_t){.x = 104, .y = 358, .a = 0.0f},
        (pos2_t){.x = 106, .y = 360, .a = 0.0f},
        (pos2_t){.x = 112, .y = 358, .a = 0.0f},
        (pos2_t){.x = 112, .y = 356, .a = 0.0f},
        (pos2_t){.x = 112, .y = 356, .a = 0.0f},
        (pos2_t){.x = 110, .y = 352, .a = 0.0f},
        (pos2_t){.x = 106, .y = 348, .a = 0.0f},
        (pos2_t){.x = 102, .y = 342, .a = 0.0f},
        (pos2_t){.x = 100, .y = 340, .a = 0.0f},
        (pos2_t){.x = 98, .y = 340, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps167);
    control_set_waypoints(&shared_ctrl, wps167, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps168[] = {
        (pos2_t){.x = 118, .y = 356, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps168);
    control_set_waypoints(&shared_ctrl, wps168, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps169[] = {
        (pos2_t){.x = 116, .y = 358, .a = 0.0f},
        (pos2_t){.x = 112, .y = 364, .a = 0.0f},
        (pos2_t){.x = 108, .y = 372, .a = 0.0f},
        (pos2_t){.x = 104, .y = 380, .a = 0.0f},
        (pos2_t){.x = 100, .y = 386, .a = 0.0f},
        (pos2_t){.x = 98, .y = 388, .a = 0.0f},
        (pos2_t){.x = 96, .y = 392, .a = 0.0f},
        (pos2_t){.x = 96, .y = 394, .a = 0.0f},
        (pos2_t){.x = 98, .y = 398, .a = 0.0f},
        (pos2_t){.x = 102, .y = 402, .a = 0.0f},
        (pos2_t){.x = 108, .y = 406, .a = 0.0f},
        (pos2_t){.x = 116, .y = 408, .a = 0.0f},
        (pos2_t){.x = 118, .y = 408, .a = 0.0f},
        (pos2_t){.x = 120, .y = 406, .a = 0.0f},
        (pos2_t){.x = 124, .y = 402, .a = 0.0f},
        (pos2_t){.x = 124, .y = 396, .a = 0.0f},
        (pos2_t){.x = 124, .y = 394, .a = 0.0f},
        (pos2_t){.x = 126, .y = 394, .a = 0.0f},
        (pos2_t){.x = 130, .y = 392, .a = 0.0f},
        (pos2_t){.x = 132, .y = 390, .a = 0.0f},
        (pos2_t){.x = 136, .y = 384, .a = 0.0f},
        (pos2_t){.x = 134, .y = 376, .a = 0.0f},
        (pos2_t){.x = 130, .y = 366, .a = 0.0f},
        (pos2_t){.x = 122, .y = 358, .a = 0.0f},
        (pos2_t){.x = 120, .y = 356, .a = 0.0f},
        (pos2_t){.x = 118, .y = 356, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps169);
    control_set_waypoints(&shared_ctrl, wps169, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps170[] = {
        (pos2_t){.x = 122, .y = 372, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps170);
    control_set_waypoints(&shared_ctrl, wps170, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps171[] = {
        (pos2_t){.x = 128, .y = 380, .a = 0.0f},
        (pos2_t){.x = 128, .y = 380, .a = 0.0f},
        (pos2_t){.x = 128, .y = 382, .a = 0.0f},
        (pos2_t){.x = 128, .y = 382, .a = 0.0f},
        (pos2_t){.x = 126, .y = 382, .a = 0.0f},
        (pos2_t){.x = 122, .y = 380, .a = 0.0f},
        (pos2_t){.x = 116, .y = 376, .a = 0.0f},
        (pos2_t){.x = 116, .y = 376, .a = 0.0f},
        (pos2_t){.x = 116, .y = 372, .a = 0.0f},
        (pos2_t){.x = 118, .y = 368, .a = 0.0f},
        (pos2_t){.x = 120, .y = 370, .a = 0.0f},
        (pos2_t){.x = 122, .y = 372, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps171);
    control_set_waypoints(&shared_ctrl, wps171, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps172[] = {
        (pos2_t){.x = 112, .y = 386, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps172);
    control_set_waypoints(&shared_ctrl, wps172, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps173[] = {
        (pos2_t){.x = 116, .y = 392, .a = 0.0f},
        (pos2_t){.x = 116, .y = 396, .a = 0.0f},
        (pos2_t){.x = 116, .y = 396, .a = 0.0f},
        (pos2_t){.x = 112, .y = 396, .a = 0.0f},
        (pos2_t){.x = 108, .y = 390, .a = 0.0f},
        (pos2_t){.x = 110, .y = 386, .a = 0.0f},
        (pos2_t){.x = 112, .y = 386, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps173);
    control_set_waypoints(&shared_ctrl, wps173, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps174[] = {
        (pos2_t){.x = 194, .y = 362, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps174);
    control_set_waypoints(&shared_ctrl, wps174, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps175[] = {
        (pos2_t){.x = 192, .y = 366, .a = 0.0f},
        (pos2_t){.x = 190, .y = 370, .a = 0.0f},
        (pos2_t){.x = 190, .y = 378, .a = 0.0f},
        (pos2_t){.x = 186, .y = 394, .a = 0.0f},
        (pos2_t){.x = 182, .y = 406, .a = 0.0f},
        (pos2_t){.x = 182, .y = 408, .a = 0.0f},
        (pos2_t){.x = 178, .y = 406, .a = 0.0f},
        (pos2_t){.x = 172, .y = 404, .a = 0.0f},
        (pos2_t){.x = 166, .y = 408, .a = 0.0f},
        (pos2_t){.x = 164, .y = 412, .a = 0.0f},
        (pos2_t){.x = 164, .y = 414, .a = 0.0f},
        (pos2_t){.x = 166, .y = 416, .a = 0.0f},
        (pos2_t){.x = 172, .y = 420, .a = 0.0f},
        (pos2_t){.x = 178, .y = 422, .a = 0.0f},
        (pos2_t){.x = 188, .y = 424, .a = 0.0f},
        (pos2_t){.x = 192, .y = 424, .a = 0.0f},
        (pos2_t){.x = 198, .y = 420, .a = 0.0f},
        (pos2_t){.x = 202, .y = 414, .a = 0.0f},
        (pos2_t){.x = 202, .y = 414, .a = 0.0f},
        (pos2_t){.x = 202, .y = 414, .a = 0.0f},
        (pos2_t){.x = 202, .y = 412, .a = 0.0f},
        (pos2_t){.x = 194, .y = 412, .a = 0.0f},
        (pos2_t){.x = 192, .y = 410, .a = 0.0f},
        (pos2_t){.x = 192, .y = 408, .a = 0.0f},
        (pos2_t){.x = 196, .y = 396, .a = 0.0f},
        (pos2_t){.x = 198, .y = 384, .a = 0.0f},
        (pos2_t){.x = 198, .y = 374, .a = 0.0f},
        (pos2_t){.x = 198, .y = 366, .a = 0.0f},
        (pos2_t){.x = 196, .y = 364, .a = 0.0f},
        (pos2_t){.x = 194, .y = 362, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps175);
    control_set_waypoints(&shared_ctrl, wps175, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps176[] = {
        (pos2_t){.x = 154, .y = 368, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps176);
    control_set_waypoints(&shared_ctrl, wps176, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps177[] = {
        (pos2_t){.x = 150, .y = 370, .a = 0.0f},
        (pos2_t){.x = 144, .y = 374, .a = 0.0f},
        (pos2_t){.x = 140, .y = 378, .a = 0.0f},
        (pos2_t){.x = 138, .y = 384, .a = 0.0f},
        (pos2_t){.x = 136, .y = 390, .a = 0.0f},
        (pos2_t){.x = 136, .y = 396, .a = 0.0f},
        (pos2_t){.x = 138, .y = 404, .a = 0.0f},
        (pos2_t){.x = 138, .y = 414, .a = 0.0f},
        (pos2_t){.x = 142, .y = 418, .a = 0.0f},
        (pos2_t){.x = 144, .y = 418, .a = 0.0f},
        (pos2_t){.x = 146, .y = 418, .a = 0.0f},
        (pos2_t){.x = 150, .y = 418, .a = 0.0f},
        (pos2_t){.x = 152, .y = 418, .a = 0.0f},
        (pos2_t){.x = 158, .y = 412, .a = 0.0f},
        (pos2_t){.x = 164, .y = 406, .a = 0.0f},
        (pos2_t){.x = 168, .y = 394, .a = 0.0f},
        (pos2_t){.x = 170, .y = 384, .a = 0.0f},
        (pos2_t){.x = 168, .y = 376, .a = 0.0f},
        (pos2_t){.x = 166, .y = 370, .a = 0.0f},
        (pos2_t){.x = 162, .y = 368, .a = 0.0f},
        (pos2_t){.x = 158, .y = 366, .a = 0.0f},
        (pos2_t){.x = 154, .y = 368, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps177);
    control_set_waypoints(&shared_ctrl, wps177, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps178[] = {
        (pos2_t){.x = 158, .y = 378, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps178);
    control_set_waypoints(&shared_ctrl, wps178, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_down();
    pos2_t wps179[] = {
        (pos2_t){.x = 160, .y = 382, .a = 0.0f},
        (pos2_t){.x = 160, .y = 386, .a = 0.0f},
        (pos2_t){.x = 160, .y = 394, .a = 0.0f},
        (pos2_t){.x = 156, .y = 400, .a = 0.0f},
        (pos2_t){.x = 154, .y = 404, .a = 0.0f},
        (pos2_t){.x = 150, .y = 406, .a = 0.0f},
        (pos2_t){.x = 148, .y = 406, .a = 0.0f},
        (pos2_t){.x = 146, .y = 404, .a = 0.0f},
        (pos2_t){.x = 146, .y = 398, .a = 0.0f},
        (pos2_t){.x = 146, .y = 392, .a = 0.0f},
        (pos2_t){.x = 146, .y = 388, .a = 0.0f},
        (pos2_t){.x = 150, .y = 382, .a = 0.0f},
        (pos2_t){.x = 154, .y = 378, .a = 0.0f},
        (pos2_t){.x = 156, .y = 378, .a = 0.0f},
        (pos2_t){.x = 158, .y = 378, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps179);
    control_set_waypoints(&shared_ctrl, wps179, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    pen_up();
    pos2_t wps180[] = {
        (pos2_t){.x = 0, .y = 0, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps180);
    pen_up();
    control_set_waypoints(&shared_ctrl, wps180, wps_len);
    control_task_wait_target_default(((float)wps_len) * 1000.0f, 10.0f);

    // drawing end

    pos2_t wps181[] = {
        (pos2_t){.x = 0.0f, .y = 0.0f, .a = 10.0f * M_PI},
    };
    control_set_waypoints(&shared_ctrl, wps181, wps_len);
    control_task_wait_target_default(15.0f, 10.0f);
}
