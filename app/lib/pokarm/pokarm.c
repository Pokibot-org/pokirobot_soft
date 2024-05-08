#include "pokarm.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

#include "servo_pwm/servo_pwm.h"
#include "shared.h"
#include "control/control.h"
#include "tmc2209/tmc2209.h"
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>

LOG_MODULE_REGISTER(pokarm);

/**
 * @brief pokarm
 * Module contolling the robot arm of the pokibot
 * AXES:
 * - Z : stepper
 * - rotation on Z axis : servo
 * - rotation on Y axis : servo
 */

#if (DT_NODE_HAS_STATUS(DT_ALIAS(servo_orientation), okay) &&                                      \
     DT_NODE_HAS_STATUS(DT_ALIAS(servo_arm), okay) &&                                              \
     DT_NODE_HAS_STATUS(DT_ALIAS(sw_arm_top), okay))
typedef struct pokarm {
    servo_pwm_t servo_orientation;
    servo_pwm_t servo_arm;
    tmc2209_t z_stepper;
    struct gpio_dt_spec top_sw;
} pokarm_t;
static pokarm_t shared_pokarm = {
    .servo_orientation.spec = PWM_DT_SPEC_GET(DT_ALIAS(servo_orientation)),
    .servo_arm.spec = PWM_DT_SPEC_GET(DT_ALIAS(servo_arm)),
    .top_sw = GPIO_DT_SPEC_GET(DT_ALIAS(sw_arm_top), gpios)
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

void _test_pokarm_drawing(void) {
//    LOG_INF("_test_pokarm_drawing");
//#if !(CONFIG_CONTROL_TASK)
//    LOG_ERR("control task not launched");
//#endif
//    shared_ctrl.start_init = true;
//    while (1) {
//        if (!shared_ctrl.ready) {
//            k_sleep(K_MSEC(100));
//            continue;
//        }
//        break;
//    }
//    LOG_DBG("alive");
//    // gpio_pin_toggle(led.port, led.pin);
//    pos2_t target = (pos2_t){0.0f, 0.0f, 0.0f};
//    control_set_pos(&shared_ctrl, (pos2_t){0.0f, 0.0f, 0.0f});
//    control_set_waypoints(&shared_ctrl, &target, 1);
//    k_sleep(K_MSEC(1000));
//    shared_ctrl.start = true;
//
//    // start of drawing
//
//    // end of drawing
}

#endif