#include "servo_pwm.h"

#include <math.h>
#include <zephyr/kernel.h>

#include "pokutils.h"
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(servo, 2);

int servo_pwm_set_angle(servo_pwm_t *obj, float angle_rad)
{
    obj->current_angle = angle_rad;
    float angle_ratio =
        (angle_rad - obj->config.min_angle) / fabsf(obj->config.max_angle - obj->config.min_angle);
    uint32_t pulse =
        obj->config.min_pulse + angle_ratio * (obj->config.max_pulse - obj->config.min_pulse);
    LOG_DBG("PWM(%s) channel %d set %d/%d", obj->spec.dev->name, obj->spec.channel, pulse,
            obj->config.period);
    return pwm_set_dt(&obj->spec, obj->config.period, pulse);
}

int servo_pwm_set_angle_ramp(servo_pwm_t *obj, float angle_rad, uint32_t time_ms)
{
    int err = 0;
    uint32_t nb_steps_per_s = NSEC_PER_SEC / obj->config.period;
    uint32_t nb_steps = nb_steps_per_s * time_ms / 1000;
    float initial_angle = obj->current_angle;
    float increment_step = (angle_rad - initial_angle) / nb_steps;

    for (size_t i = 0; i < nb_steps; i++) {
        float new_angle = initial_angle + increment_step * i;
        err |= servo_pwm_set_angle(obj, new_angle);
        k_sleep(K_MSEC(1000 / nb_steps_per_s));
    }
    err |= servo_pwm_set_angle(obj, angle_rad);
    return err;
}

int servo_pwm_init(servo_pwm_t *obj)
{
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

    LOG_DBG("Init obj(%p), dev %p, period %d", obj, obj->spec.dev, obj->spec.period);
    return 0;
}
