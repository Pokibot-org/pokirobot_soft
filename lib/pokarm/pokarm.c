#include "pokarm.h"

#include <device.h>
#include <devicetree.h>
#include <zephyr.h>

#include "servo_pwm/servo_pwm.h"
#include "shared.h"
#include "tmc2209/tmc2209.h"
#include <logging/log.h>

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
    servo_pwm_t z_rot_servo;
    servo_pwm_t y_rot_servo;
    tmc2209_t z_stepper;
} pokarm_t;
static pokarm_t obj;


#define Z_SPEED (10 * TMC2209_VACTUAL_MAX / 100)

int pokarm_init() {
    int err = 0;
    const unsigned int period_ns = USEC_PER_SEC / 50;
    err |= servo_pwm_init(&obj.z_rot_servo, period_ns);
    err |= servo_pwm_init(&obj.y_rot_servo, period_ns);
    err |= tmc2209_init(&obj.z_stepper, &steppers_uart_hdb, 3);
    if (err) {
        LOG_ERR("Error in init %d", err);
    }
    LOG_INF("Init done");
    return err;
}
