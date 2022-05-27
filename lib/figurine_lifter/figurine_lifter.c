#include "figurine_lifter.h"

#include <device.h>
#include <devicetree.h>
#include <zephyr.h>

#include "utils.h"
#include <drivers/gpio.h>
#include <logging/log.h>
#include <servo_pwm/servo_pwm.h>

LOG_MODULE_REGISTER(figurine_lifter);

typedef struct figurine_lifter {
    servo_pwm_t servo_1;
    servo_pwm_t servo_2;
    struct gpio_dt_spec magnet_spec;
} figurine_lifter_t;

#if !(DT_NODE_HAS_STATUS(DT_ALIAS(servo_dicristaline1), okay) &&               \
      DT_NODE_HAS_STATUS(DT_ALIAS(servo_dicristaline2), okay) &&               \
      DT_NODE_HAS_STATUS(DT_ALIAS(sw_electromagnet), okay))
#error Device tree not valid, missing stuff for figurine lifter
static figurine_lifter_t obj;
#else
static figurine_lifter_t obj = {
    .servo_1.spec = PWM_DT_SPEC_GET(DT_ALIAS(servo_dicristaline1)),
    .servo_2.spec = PWM_DT_SPEC_GET(DT_ALIAS(servo_dicristaline2)),
    .magnet_spec = GPIO_DT_SPEC_GET(DT_ALIAS(sw_electromagnet), gpios),
};
#endif

int fl_set_magnet(int value) {
    return gpio_pin_set_dt(&obj.magnet_spec, value);
}

int figurine_lifter_init(void) {
    int err = 0;
    const servo_pwm_config_t servo_config = {.period = NSEC_PER_SEC / 50,
        .min_angle = 0,
        .max_angle = M_PI,
        .min_pulse = 500000,
        .max_pulse = 2500000};
    obj.servo_1.config = servo_config;
    obj.servo_2.config = servo_config;
    err |= servo_pwm_init(&obj.servo_1);
    err |= servo_pwm_init(&obj.servo_2);
    err |= gpio_pin_configure_dt(&obj.magnet_spec, GPIO_OUTPUT_LOW);
    if (err) {
        LOG_ERR("Error in init %d", err);
    }

    LOG_INF("Init done");
    return err;
}

int figurine_lifter_up_inside(void) {
    int err = 0;
    err |= servo_pwm_set_angle(&obj.servo_1, 0);
    gpio_pin_set_dt(&obj.magnet_spec, 0);
    return err;
}

int figurine_lifter_up_transport(void) {
    int err = 0;
    // err |= servo_pwm_set_angle(&obj.servo_1, M_PI * 0.1);
    err |= servo_pwm_set_angle_ramp(&obj.servo_1, M_PI * 0.1, 2000);
    return err;
}

int figurine_lifter_grab(void) {
    int err = 0;
    err |= servo_pwm_set_angle(&obj.servo_1, M_PI * 0.65);
    k_sleep(K_MSEC(1000));
    gpio_pin_set_dt(&obj.magnet_spec, 1);
    k_sleep(K_MSEC(1000));
    figurine_lifter_up_transport();
    return err;
}

int figurine_lifter_put(void) {
    int err = 0;
    err |= servo_pwm_set_angle(&obj.servo_1, M_PI * 0.65);
    k_sleep(K_MSEC(1000));
    gpio_pin_set_dt(&obj.magnet_spec, 0);
    k_sleep(K_MSEC(1000));
    figurine_lifter_up_inside();
    return err;
}
