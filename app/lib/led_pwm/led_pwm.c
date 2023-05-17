#include "led_pwm.h"

#include <zephyr/kernel.h>

#include <zephyr/drivers/pwm.h>

int led_set(led_t *led, uint8_t val)
{
    uint32_t pulse = val * (LED_PWM_PERIOD_NS / 255);
    return pwm_set_dt(&led->spec, LED_PWM_PERIOD_NS, pulse);
}

bool led_is_ready(led_t *led)
{
    return device_is_ready(led->spec.dev);
}

int rgb_set(rgb_t *led, uint8_t r, uint8_t g, uint8_t b)
{
    int ret = 0;
    ret |= led_set(&led->red, r);
    ret |= led_set(&led->green, g);
    ret |= led_set(&led->blue, b);
    return ret;
}

bool rgb_is_ready(rgb_t *led)
{
    return led_is_ready(&led->red) && led_is_ready(&led->green) && led_is_ready(&led->blue);
}
