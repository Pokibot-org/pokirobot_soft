#include "hmi_led.h"

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include "led_pwm/led_pwm.h"
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(led_control);


rgb_t hmi_led = {
    .red = {.spec = PWM_DT_SPEC_GET(DT_ALIAS(red_pwm_led))},
    .green = {.spec = PWM_DT_SPEC_GET(DT_ALIAS(green_pwm_led))},
    .blue = {.spec = PWM_DT_SPEC_GET(DT_ALIAS(blue_pwm_led))},
};

int hmi_led_init(void) {
    LOG_INF("initializing HMI LED");
    int ret = !rgb_is_ready(&hmi_led);
    if (ret) {
        LOG_ERR("RGB device not ready");
        if (!led_is_ready(&hmi_led.red)) {
            LOG_ERR(
                "red LED device is not ready: %s", hmi_led.red.spec.dev->name);
        }
        if (!led_is_ready(&hmi_led.green)) {
            LOG_ERR("green LED device is not ready: %s",
                hmi_led.green.spec.dev->name);
        }
        if (!led_is_ready(&hmi_led.blue)) {
            LOG_ERR("blue LED device is not ready: %s",
                hmi_led.blue.spec.dev->name);
        }
    }
    LOG_INF("HMI LED initialization done (ret: %d)", ret);
    return ret;
}

int hmi_led_error(void) {
    return rgb_set(&hmi_led, 255, 0, 0);
}

int hmi_led_success(void) {
    return rgb_set(&hmi_led, 0, 255, 0);
}
