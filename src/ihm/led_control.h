#ifndef LED_CONTROL_H
#define LED_CONTROL_H
#include "stdint.h"

void led_control_init(void);
void rgb_led_set(uint8_t r, uint8_t g, uint8_t b);

#endif