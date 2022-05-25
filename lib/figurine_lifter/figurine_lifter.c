#include "figurine_lifter.h"

#include <device.h>
#include <devicetree.h>
#include <zephyr.h>

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
    const unsigned int period_ns = USEC_PER_SEC / 50;
    err |= servo_pwm_init(&obj.servo_1, period_ns);
    err |= servo_pwm_init(&obj.servo_2, period_ns);
    err |= gpio_pin_configure_dt(&obj.magnet_spec, GPIO_OUTPUT_LOW);
    if (err) {
        LOG_ERR("Error in init %d", err);
    }

    LOG_INF("Init done");
    return err;
}

int figurine_lifter_grab(void) {
    int err = 0;
    return err;
}

int figurine_lifter_put(void) {
    int err = 0;
    return err;
}
