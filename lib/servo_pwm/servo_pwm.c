#include "servo_pwm.h"
#include <zephyr.h>
#include <drivers/pwm.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(servo);

#define M_PI 3.14159265358979323846


int servo_pwm_set_angle(servo_pwm_t* obj, float angle_rad) {
    if (angle_rad < 0 || angle_rad > M_PI) {
        return -1;
    }
    uint32_t pulse = angle_rad * (obj->period_ns / M_PI);
    return pwm_set_dt(&obj->spec, obj->period_ns, pulse);
}

int servo_pwm_init(servo_pwm_t* obj, uint32_t pwm_period_ns) {
    LOG_DBG("Init obj(%p), dev %p", obj, obj->spec.dev);
    if (!device_is_ready(obj->spec.dev)) {
        return -1;
    }

    obj->period_ns = pwm_period_ns;
    return 0;
}
