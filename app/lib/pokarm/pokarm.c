#include "pokarm.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

#include "servo_pwm/servo_pwm.h"
#include "shared.h"
#include "tmc2209/tmc2209.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pokarm);

/**
 * @brief pokarm
 * Module contolling the robot arm of the pokibot
 * AXES:
 * - Z : stepper
 * - rotation on Z axis : servo
 * - rotation on Y axis : servo
 */

typedef struct pokarm {
	servo_pwm_t servo_orientation;
	servo_pwm_t servo_arm;
	tmc2209_t z_stepper;
} pokarm_t;
static pokarm_t obj = {
	.servo_orientation.spec = PWM_DT_SPEC_GET(DT_ALIAS(servo_orientation)),
	.servo_arm.spec = PWM_DT_SPEC_GET(DT_ALIAS(servo_arm)),
};

int pokarm_init(void)
{
	int err = 0;

	const servo_pwm_config_t servo_config = {.period = NSEC_PER_SEC / 50,
											 .min_angle = 0,
											 .max_angle = M_PI,
											 .min_pulse = 500000,
											 .max_pulse = 2500000};
	obj.servo_orientation.config = servo_config;
	obj.servo_arm.config = servo_config;

	err |= servo_pwm_init(&obj.servo_orientation);
	err |= servo_pwm_init(&obj.servo_arm);
	err |= tmc2209_init(&obj.z_stepper, &steppers_uart_hdb, 3);
	if (err) {
		LOG_ERR("Error in init %d", err);
	}
	LOG_INF("Init done");
	return err;
}

int pokarm_up(void)
{
	int err = 0;
	err |= servo_pwm_set_angle(&obj.servo_arm, 0);
	return err;
}

int pokarm_pos_flat_hexagone(void)
{
	int err = 0;
	err |= servo_pwm_set_angle(&obj.servo_arm, M_PI * 3 / 4);
	return err;
}

int pokarm_pos_put_haxagone_display(void)
{
	int err = 0;
	err |= servo_pwm_set_angle(&obj.servo_arm, M_PI / 2);
	return err;
}

void pokarm_test(void)
{
	LOG_INF("---------- TEST -------------");
	int err;
	uint8_t steps = 6;
	for (float angle = 0; angle < M_PI; angle += (M_PI / steps)) {
		err = servo_pwm_set_angle(&obj.servo_arm, angle);
		if (err) {
			LOG_WRN("err %d", err);
		}
		k_sleep(K_MSEC(1000));
	}
	while (1) {
		k_sleep(K_MSEC(1000));
	}
}
