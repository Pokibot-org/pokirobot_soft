#ifndef LED_PWM_H
#define LED_PWM_H

#include <stdint.h>

#include <zephyr/drivers/pwm.h>

#define LED_PWM_PERIOD_NS 10000000 // 10ms - 100Hz

typedef struct led {
    struct pwm_dt_spec spec;
} led_t;

typedef struct rgb {
    led_t red;
    led_t green;
    led_t blue;
} rgb_t;

int led_set(led_t *led, uint8_t val);
bool led_is_ready(led_t *led);
int rgb_set(rgb_t *led, uint8_t r, uint8_t g, uint8_t b);
bool rgb_is_ready(rgb_t *led);

#endif
