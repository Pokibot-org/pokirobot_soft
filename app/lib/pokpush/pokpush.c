#include "pokpush.h"

#include "pokutils.h"
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

#include "servo_pwm/servo_pwm.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pokpush);

/**
 * @brief pokarm
 * Module contolling the robot arm of the pokibot
 * AXES:
 * - Z : stepper
 * - rotation on Z axis : servo
 * - rotation on Y axis : servo
 */

static struct pokpush {
    servo_pwm_t servo;
} obj = {
    .servo.spec = PWM_DT_SPEC_GET(DT_ALIAS(servo_pokpush)),
};

static int pokpush_init(const struct device *dev)
{
    int err = 0;

    const servo_pwm_config_t servo_config = {.period = NSEC_PER_SEC / 50,
                                             .min_angle = 0,
                                             .max_angle = M_PI,
                                             .min_pulse = 500000,
                                             .max_pulse = 2500000};
    obj.servo.config = servo_config;

    err |= servo_pwm_init(&obj.servo);
    if (err) {
        LOG_ERR("Error in init %d", err);
    }
    LOG_INF("Init done");
    return err;
}

SYS_INIT(pokpush_init, APPLICATION, 90);

int pokpush_deploy(void)
{
    LOG_DBG("deploying");
    int err = 0;
    err |= servo_pwm_set_angle(&obj.servo, M_PI * 4 / 10);
    return err;
}
int pokpush_retract(void)
{
    LOG_DBG("retracting");

    int err = 0;
    err |= servo_pwm_set_angle(&obj.servo, M_PI * 80 / 100);
    return err;
}
