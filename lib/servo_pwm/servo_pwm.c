#include "servo_pwm.h"

#include <math.h>
#include <zephyr.h>

#include <drivers/pwm.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(servo);

#define M_PI 3.14159265358979323846


int servo_pwm_set_angle(servo_pwm_t* obj, float angle_rad) {
    float angle_ratio = (angle_rad - obj->config.min_angle) /
                        fabsf(obj->config.max_angle - obj->config.min_angle);
    uint32_t pulse =
        obj->config.min_pulse +
        angle_ratio * (obj->config.max_pulse - obj->config.min_pulse);
    LOG_DBG("PWM(%s) channel %d set %d/%d", obj->spec.dev->name,
        obj->spec.channel, pulse, obj->config.period);
    return pwm_set_dt(&obj->spec, obj->config.period, pulse);
}

int servo_pwm_init(servo_pwm_t* obj) {
    if (!device_is_ready(obj->spec.dev)) {
        return -1;
    }
    if (obj->config.period == 0) {
        LOG_ERR("Period cant be null");
        return -2;
    }

    if (obj->config.min_pulse >= obj->config.max_pulse ||
        obj->config.min_angle == -obj->config.max_angle) {
        LOG_ERR("Config min max angles/pulses");
        return -3;
    }

    LOG_DBG("Init obj(%p), dev %p, period %d", obj, obj->spec.dev,
        obj->spec.period);
    return 0;
}
