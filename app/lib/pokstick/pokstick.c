#include "pokstick.h"

#include "pokutils.h"
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

#include "servo_pwm/servo_pwm.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pokstick);


#if (DT_NODE_HAS_STATUS(DT_ALIAS(servo_stick), okay))
static struct pokstick {
    servo_pwm_t servo;
} obj = {
    .servo.spec = PWM_DT_SPEC_GET(DT_ALIAS(servo_stick)),
};

static int pokstick_init(void)
{
    int err = 0;

    const servo_pwm_config_t servo_config = {.period = NSEC_PER_SEC / 50,
                                             .min_angle = 0,
                                             .max_angle = 180,
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

SYS_INIT(pokstick_init, APPLICATION, 90);

#define POS_DEPLOYED  45
#define POS_RETRACTED 160

int pokstick_deploy(void)
{
    LOG_DBG("deploying");
    int err = 0;
    err |= servo_pwm_set_angle(&obj.servo, POS_DEPLOYED);
    return err;
}

int pokstick_retract(void)
{
    LOG_DBG("retracting");

    int err = 0;
    err |= servo_pwm_set_angle(&obj.servo, POS_RETRACTED);
    return err;
}

#endif