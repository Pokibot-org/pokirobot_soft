#ifndef HMI_LED_H
#define HMI_LED_H

#include <stdint.h>

typedef struct hmi_led {
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint32_t blink_period;
} hmi_led_t;

int hmi_led_init(void);

int hmi_led_error(void);
int hmi_led_success(void);


// void rgb_led_set(uint8_t r, uint8_t g, uint8_t b);

#endif
