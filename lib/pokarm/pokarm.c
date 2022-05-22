#include "pokarm.h"

#include <device.h>
#include <devicetree.h>
#include <zephyr.h>

#include <logging/log.h>

LOG_MODULE_REGISTER(pokarm);

#define Z_SPEED (10*TMC2209_VACTUAL_MAX/100)


int pokarm_init(pokarm_t* obj) {
    int err = 0;
    const unsigned int period_ns = USEC_PER_SEC/50;
    err |= servo_pwm_init(&obj->z_rot_servo, period_ns);
    err |= servo_pwm_init(&obj->y_rot_servo, period_ns);

    if (err)
    {
        LOG_ERR("Error in init %d", err);
    }
    
    LOG_INF("Init done");
    return err;
}