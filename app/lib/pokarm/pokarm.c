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
    err |= tmc2209_set_ihold_irun(&shared_pokarm.z_stepper, 1, 8);
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
    tmc2209_set_speed(&shared_pokarm.z_stepper, 49000);
    k_sleep(K_MSEC(500));
    tmc2209_set_speed(&shared_pokarm.z_stepper, 0);
}

void _test_pokarm_drawing(void) {
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
    control_task_wait_target_default(100000.0f, 10.0f);

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
    control_task_wait_target_default(100000.0f, 10.0f);

    pen_up();
    pos2_t wps2[] = {
        (pos2_t){.x = 183, .y = 396, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps2);
    control_set_waypoints(&shared_ctrl, wps2, wps_len);
    control_task_wait_target_default(100000.0f, 10.0f);

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
    control_task_wait_target_default(100000.0f, 10.0f);

    pen_up();
    pos2_t wps4[] = {
        (pos2_t){.x = 195, .y = 384, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps4);
    control_set_waypoints(&shared_ctrl, wps4, wps_len);
    control_task_wait_target_default(100000.0f, 10.0f);

    pen_down();
    pos2_t wps5[] = {
        (pos2_t){.x = 207, .y = 366, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps5);
    control_set_waypoints(&shared_ctrl, wps5, wps_len);
    control_task_wait_target_default(100000.0f, 10.0f);

    pen_up();
    pos2_t wps6[] = {
        (pos2_t){.x = 342, .y = 396, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps6);
    control_set_waypoints(&shared_ctrl, wps6, wps_len);
    control_task_wait_target_default(100000.0f, 10.0f);

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
    control_task_wait_target_default(100000.0f, 10.0f);

    pen_up();
    pos2_t wps8[] = {
        (pos2_t){.x = 348, .y = 387, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps8);
    control_set_waypoints(&shared_ctrl, wps8, wps_len);
    control_task_wait_target_default(100000.0f, 10.0f);

    pen_down();
    pos2_t wps9[] = {
        (pos2_t){.x = 360, .y = 366, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps9);
    control_set_waypoints(&shared_ctrl, wps9, wps_len);
    control_task_wait_target_default(100000.0f, 10.0f);

    pen_up();
    pos2_t wps10[] = {
        (pos2_t){.x = 288, .y = 210, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps10);
    control_set_waypoints(&shared_ctrl, wps10, wps_len);
    control_task_wait_target_default(100000.0f, 10.0f);

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
    control_task_wait_target_default(100000.0f, 10.0f);

    pen_up();
    pos2_t wps12[] = {
        (pos2_t){.x = 342, .y = 171, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps12);
    control_set_waypoints(&shared_ctrl, wps12, wps_len);
    control_task_wait_target_default(100000.0f, 10.0f);

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
    control_task_wait_target_default(100000.0f, 10.0f);

    pen_up();
    pos2_t wps14[] = {
        (pos2_t){.x = 219, .y = 135, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps14);
    control_set_waypoints(&shared_ctrl, wps14, wps_len);
    control_task_wait_target_default(100000.0f, 10.0f);

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
    control_task_wait_target_default(100000.0f, 10.0f);

    pen_up();
    pos2_t wps16[] = {
        (pos2_t){.x = 114, .y = 72, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps16);
    control_set_waypoints(&shared_ctrl, wps16, wps_len);
    control_task_wait_target_default(100000.0f, 10.0f);

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
    control_task_wait_target_default(100000.0f, 10.0f);

    pen_up();
    pos2_t wps18[] = {
        (pos2_t){.x = 384, .y = 582, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps18);
    control_set_waypoints(&shared_ctrl, wps18, wps_len);
    control_task_wait_target_default(100000.0f, 10.0f);

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
    control_task_wait_target_default(100000.0f, 10.0f);

    pen_up();
    pos2_t wps20[] = {
        (pos2_t){.x = 0, .y = 0, .a = 0.0f},
    };
    wps_len = ARRAY_SIZE(wps20);
    pen_up();
    control_set_waypoints(&shared_ctrl, wps20, wps_len);
    control_task_wait_target_default(100000.0f, 10.0f);

    // drawing end

    pos2_t wps21[] = {
        (pos2_t){.x = 0.0f, .y = 0.0f, .a = -2.0f},
    };
    control_set_waypoints(&shared_ctrl, wps21, wps_len);
    control_task_wait_target_default(15.0f, 10.0f);
}
