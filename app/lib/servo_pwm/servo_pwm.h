#ifndef SERVO_PWM_H
#define SERVO_PWM_H
#include <stdint.h>

#include <zephyr/drivers/pwm.h>

typedef struct servo_pwm_config {
	uint32_t period;
	uint32_t min_pulse;
	uint32_t max_pulse;
	float min_angle;
	float max_angle;
} servo_pwm_config_t;

typedef struct servo_pwm {
	struct pwm_dt_spec spec;
	servo_pwm_config_t config;
	float current_angle;
} servo_pwm_t;

int servo_pwm_init(servo_pwm_t *obj);
int servo_pwm_set_angle(servo_pwm_t *servo, float angle_rad);
int servo_pwm_set_angle_ramp(servo_pwm_t *obj, float angle_rad, uint32_t time_ms);
#endif
