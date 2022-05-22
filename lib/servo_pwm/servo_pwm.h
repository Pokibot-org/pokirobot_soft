#ifndef SERVO_PWM_H
#define SERVO_PWM_H
#include <stdint.h>

#include <drivers/pwm.h>

typedef struct servo_pwm {
    struct pwm_dt_spec spec;
    uint32_t period_ns;
} servo_pwm_t;

int servo_pwm_init(servo_pwm_t* obj, uint32_t pwm_period_ns);
int servo_pwm_set_angle(servo_pwm_t* servo, float angle_rad);
#endif
