#include <device.h>
#include <zephyr.h>

#include <drivers/pwm.h>
#include <logging/log.h>
#include <sys/printk.h>

#include "led_control.h"

LOG_MODULE_REGISTER(led_control);

/*
 * Extract devicetree configuration.
 */

#define RED_NODE DT_ALIAS(red_pwm_led)
#define GREEN_NODE DT_ALIAS(green_pwm_led)
#define BLUE_NODE DT_ALIAS(blue_pwm_led)

#if DT_NODE_HAS_STATUS(RED_NODE, okay)
#define RED_CTLR_NODE DT_PWMS_CTLR(RED_NODE)
#define RED_CHANNEL DT_PWMS_CHANNEL(RED_NODE)
#define RED_FLAGS DT_PWMS_FLAGS(RED_NODE)
#else
#error "Unsupported board: red-pwm-led devicetree alias is not defined"
#define RED_CTLR_NODE DT_INVALID_NODE
#define RED_CHANNEL 0
#define RED_FLAGS 0
#endif

#if DT_NODE_HAS_STATUS(GREEN_NODE, okay)
#define GREEN_CTLR_NODE DT_PWMS_CTLR(GREEN_NODE)
#define GREEN_CHANNEL DT_PWMS_CHANNEL(GREEN_NODE)
#define GREEN_FLAGS DT_PWMS_FLAGS(GREEN_NODE)
#else
#error "Unsupported board: green-pwm-led devicetree alias is not defined"
#define GREEN_CTLR_NODE DT_INVALID_NODE
#define GREEN_CHANNEL 0
#define GREEN_FLAGS 0
#endif

#if DT_NODE_HAS_STATUS(BLUE_NODE, okay)
#define BLUE_CTLR_NODE DT_PWMS_CTLR(BLUE_NODE)
#define BLUE_CHANNEL DT_PWMS_CHANNEL(BLUE_NODE)
#define BLUE_FLAGS DT_PWMS_FLAGS(BLUE_NODE)
#else
#error "Unsupported board: blue-pwm-led devicetree alias is not defined"
#define BLUE_CTLR_NODE DT_INVALID_NODE
#define BLUE_CHANNEL 0
#define BLUE_FLAGS 0
#endif

/*
 * 50 is flicker fusion threshold. Modulated light will be perceived
 * as steady by our eyes when blinking rate is at least 50.
 */
#define PERIOD_USEC (USEC_PER_SEC / 200U)

static int pwm_set(const struct device* pwm_dev, uint32_t pwm_pin, uint8_t pulse_width, pwm_flags_t flags) {
    return pwm_pin_set_usec(pwm_dev, pwm_pin, PERIOD_USEC, PERIOD_USEC * pulse_width / 255, flags);
}

enum { RED, GREEN, BLUE };
const struct device* pwm_dev[3];

void led_control_init(void) {
    LOG_INF("PWM-based RGB LED control");

    pwm_dev[RED] = DEVICE_DT_GET(RED_CTLR_NODE);
    pwm_dev[GREEN] = DEVICE_DT_GET(GREEN_CTLR_NODE);
    pwm_dev[BLUE] = DEVICE_DT_GET(BLUE_CTLR_NODE);

    if (!device_is_ready(pwm_dev[RED])) {
        LOG_ERR("Error: red PWM device %s is not ready", pwm_dev[RED]->name);
        return;
    }

    if (!device_is_ready(pwm_dev[GREEN])) {
        LOG_ERR("Error: green PWM device %s is not ready", pwm_dev[GREEN]->name);
        return;
    }

    if (!device_is_ready(pwm_dev[BLUE])) {
        LOG_ERR("Error: blue PWM device %s is not ready", pwm_dev[BLUE]->name);
        return;
    }
}


void rgb_led_set(uint8_t r, uint8_t g, uint8_t b) {
    uint32_t err;
    err = pwm_set(pwm_dev[RED], RED_CHANNEL, r, RED_FLAGS);
    if (err) {
        LOG_ERR("pwm_set err %d", err);
        return;
    }
    err = pwm_set(pwm_dev[GREEN], GREEN_CHANNEL, g, GREEN_FLAGS);
    if (err) {
        LOG_ERR("pwm_set err %d", err);
        return;
    }
    err = pwm_set(pwm_dev[BLUE], BLUE_CHANNEL, b, BLUE_FLAGS);
    if (err) {
        LOG_ERR("pwm_set err %d", err);
        return;
    }
}