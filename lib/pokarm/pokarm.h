#ifndef POKARM_H
#define POKARM_H

#include <zephyr.h>

#include "servo_pwm/servo_pwm.h"
#include "tmc2209/tmc2209.h"

/**
 * @brief pokarm
 * Module contolling the robot arm of the pokibot
 * AXES:
 * - Z : stepper
 * - rotation on Z axis : servo
 * - rotation on Y axis : servo
 */

typedef struct pokarm {
    servo_pwm_t z_rot_servo;
    servo_pwm_t y_rot_servo;
    tmc2209_t z_stepper;
} pokarm_t;

int pokarm_init(pokarm_t* obj);

#endif
