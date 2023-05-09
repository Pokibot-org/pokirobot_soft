#include "control.h"

#include <zephyr/kernel.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

#include "shared.h"
#include "tmc2209/tmc2209.h"
#include "pokutils.h"
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(control);

tmc2209_t train_motor_1;
tmc2209_t train_motor_2;
tmc2209_t train_motor_3;
control_t shared_ctrl;

#define CONTROL_LOCKVAR_SETTER(_var, _type)                                                        \
	int control_set_##_var(control_t *dev, _type _var)                                             \
	{                                                                                              \
		int err = 0;                                                                               \
		SET_LOCKVAR(dev->_var, _var, err, CONTROL_MUTEX_TIMEOUT);                                  \
		if (err) {                                                                                 \
			LOG_ERR("could not lock _var mutex for write access");                                 \
			goto exit_error;                                                                       \
		}                                                                                          \
		return 0;                                                                                  \
	exit_error:                                                                                    \
		return -1;                                                                                 \
	}

#define CONTROL_LOCKVAR_GETTER(_var, _type)                                                        \
	int control_get_##_var(control_t *dev, _type *_var)                                            \
	{                                                                                              \
		int err = 0;                                                                               \
		READ_LOCKVAR(dev->_var, *_var, err, CONTROL_MUTEX_TIMEOUT);                                \
		if (err) {                                                                                 \
			LOG_ERR("could not lock _var mutex for write access");                                 \
			goto exit_error;                                                                       \
		}                                                                                          \
		return 0;                                                                                  \
	exit_error:                                                                                    \
		return -1;                                                                                 \
	}

CONTROL_LOCKVAR_SETTER(pos, pos2_t)
CONTROL_LOCKVAR_SETTER(target, pos2_t)

CONTROL_LOCKVAR_GETTER(pos, pos2_t)
CONTROL_LOCKVAR_GETTER(target, pos2_t)

int control_set_brake(control_t *dev, bool brake)
{
	dev->brake = brake;
	return 0;
}

int control_init(control_t *ctrl, tmc2209_t *m1, tmc2209_t *m2, tmc2209_t *m3)
{
	k_sleep(K_MSEC(200));
	int ret = 0;
	ctrl->start = false;
	ctrl->brake = false;
	ctrl->at_target = false;
	ctrl->planar_target_sensivity = CONTROL_PLANAR_TARGET_SENSITIVITY_DEFAULT;
	ctrl->angular_target_sensivity = CONTROL_ANGULAR_TARGET_SENSITIVITY_DEFAULT;
	INIT_LOCKVAR(ctrl->pos);
	INIT_LOCKVAR(ctrl->target);
	control_set_pos(ctrl, (pos2_t){0.0f, 0.0f, 0.0f});
	control_set_target(ctrl, (pos2_t){0.0f, 0.0f, 0.0f});
	ctrl->m1 = m1;
	ctrl->m2 = m2;
	ctrl->m3 = m3;
	for (int i = 0; i < 10; i++) {
		if (tmc2209_init(ctrl->m1, &steppers_uart_hdb, 0)) {
			LOG_ERR("failed to init control motor 1");
			ret = -1;
		}
		if (tmc2209_init(ctrl->m2, &steppers_uart_hdb, 1)) {
			LOG_ERR("failed to init control motor 2");
			ret = -1;
		}
		if (tmc2209_init(ctrl->m3, &steppers_uart_hdb, 2)) {
			LOG_ERR("failed to init control motor 3");
			ret = -1;
		}
		k_sleep(K_MSEC(10));
	}
	ctrl->ready = true;
	return ret;
}

void control_force_motor_stop(void)
{
	shared_ctrl.ready = false;
	for (int i = 0; i < 10; i++) {
		tmc2209_set_speed(&train_motor_1, 0);
		tmc2209_set_speed(&train_motor_2, 0);
		tmc2209_set_speed(&train_motor_3, 0);
	}
	// delay used to be sure that tmc task send the messages
	// to be changed !
	k_sleep(K_MSEC(1000));
}

vel2_t world_vel_from_delta(pos2_t delta, vel2_t prev_vel)
{
	// planar speed capping + acceleration ramp
	float vx = PLANAR_FACTOR * delta.x;
	float vy = PLANAR_FACTOR * delta.y;
	const float planar_speed = sqrtf(vx * vx + vy * vy);
	const float planar_speed_prev = sqrtf(prev_vel.vx * prev_vel.vx + prev_vel.vy * prev_vel.vy);
	const float planar_speed_clamped =
		MIN(planar_speed, MIN(planar_speed_prev + PLANAR_RAMP, PLANAR_VMAX));
	if (planar_speed > planar_speed_clamped) {
		const float planar_factor = planar_speed_clamped / planar_speed;
		vx *= planar_factor;
		vy *= planar_factor;
	}
	// angular speed capping + acceleration ramp
	const float angular_speed_ramped = fabsf(prev_vel.w) + ANGULAR_RAMP;
	float w = Z_CLAMP(ANGULAR_FACTOR * delta.a, MAX(-angular_speed_ramped, -ANGULAR_VMAX),
					  MIN(angular_speed_ramped, ANGULAR_VMAX));
	// returning built vel
	vel2_t world_vel = {
		.vx = vx,
		.vy = vy,
		.w = w,
	};
	return world_vel;
}

vel2_t world_vel_from_local(pos2_t pos, vel2_t local_vel)
{
	vel2_t world_vel = {
		.vx = cosf(pos.a) * local_vel.vx - sinf(pos.a) * local_vel.vy,
		.vy = sinf(pos.a) * local_vel.vx + cosf(pos.a) * local_vel.vy,
		.w = -local_vel.w,
	};
	return world_vel;
}

vel2_t local_vel_from_world(pos2_t pos, vel2_t world_vel)
{
	vel2_t local_vel = {
		.vx = cosf(pos.a) * world_vel.vx + sinf(pos.a) * world_vel.vy,
		.vy = -sinf(pos.a) * world_vel.vx + cosf(-pos.a) * world_vel.vy,
		.w = -world_vel.w,
	};
	return local_vel;
}

vel2_t local_vel_from_omni(omni3_t omni)
{
	vel2_t local_vel = {
		.vx = (2.0f * omni.v2 - omni.v1 - omni.v3) / 3.0f,
		.vy = M_SQRT3 * (omni.v3 - omni.v1) / 3.0f,
		.w = omni.v1 + omni.v2 + omni.v3 / (3.0f * ROBOT_L),
	};
	return local_vel;
}

omni3_t omni_from_local_vel(vel2_t local_vel)
{
	const float w_factor = ROBOT_L * local_vel.w;
	omni3_t omni3 = {
		.v1 = (-local_vel.vx - M_SQRT3 * local_vel.vy) / 2.0 + w_factor,
		.v2 = local_vel.vx + w_factor,
		.v3 = (-local_vel.vx + M_SQRT3 * local_vel.vy) / 2.0 + w_factor,
	};
	return omni3;
}

void control_task_wait_ready()
{
	while (!shared_ctrl.ready) {
		k_sleep(K_MSEC(10));
	}
}

bool control_task_wait_target(float planar_sensivity, float angular_sensivity, uint32_t timeout_ms)
{
	shared_ctrl.planar_target_sensivity = planar_sensivity;
	shared_ctrl.angular_target_sensivity = angular_sensivity;
	shared_ctrl.at_target = false;
	for (int i = 0; i < timeout_ms; i++) {
		if (shared_ctrl.at_target) {
			break;
		}
		k_sleep(K_MSEC(1));
	}
	return shared_ctrl.at_target;
}

static void control_task_wait_start()
{
	while (!shared_ctrl.start) {
		k_sleep(K_MSEC(10));
	}
}

static int control_task(void)
{
	LOG_INF("control task init");
	int ret = 0;
	pos2_t pos = {.x = 0.0f, .y = 0.0f, .a = 0.0f};
	vel2_t world_vel = {.vx = 0.0f, .vy = 0.0f, .w = 0.0f};
	vel2_t local_vel = {.vx = 0.0f, .vy = 0.0f, .w = 0.0f};
	omni3_t motors_v = {.v1 = 0.0f, .v2 = 0.0f, .v3 = 0.0f};
	while (!shared_ctrl.start_init) {
		k_sleep(K_MSEC((uint64_t)CONTROL_PERIOD_MS));
	}

	if (control_init(&shared_ctrl, &train_motor_1, &train_motor_2, &train_motor_3)) {
		LOG_ERR("failed to init control object");
		ret = -1;
	}
	LOG_INF("control task wait");
	control_task_wait_start();
	LOG_INF("control task start");
	while (shared_ctrl.ready) {
		pos2_t target;
		control_get_pos(&shared_ctrl, &pos);
		// update pos
		// control_get_motors_v(&shared_ctrl, &motors_v);
		// local_vel = local_vel_from_omni(motors_v);
		// world_vel = world_vel_from_local(old_pos, local_vel);
		pos = (pos2_t){
			.x = pos.x + world_vel.vx * CONTROL_PERIOD_MS / 1000.0f,
			.y = pos.y + world_vel.vy * CONTROL_PERIOD_MS / 1000.0f,
			.a = pos.a + world_vel.w * CONTROL_PERIOD_MS / 1000.0f,
		};
		// update speed
		if (shared_ctrl.brake) {
			world_vel = (vel2_t){.vx = 0.0f, .vy = 0.0f, .w = 0.0f};
			local_vel = (vel2_t){.vx = 0.0f, .vy = 0.0f, .w = 0.0f};
			motors_v = (omni3_t){.v1 = 0.0f, .v2 = 0.0f, .v3 = 0.0f};
		} else {
			control_get_target(&shared_ctrl, &target);
			pos2_t delta = pos2_diff(target, pos);
			if (vec2_abs((vec2_t){delta.x, delta.y}) < CONTROL_PLANAR_TARGET_SENSITIVITY_DEFAULT &&
				delta.a < CONTROL_ANGULAR_TARGET_SENSITIVITY_DEFAULT) {
				shared_ctrl.at_target = true;
			} else {
				shared_ctrl.at_target = false;
			}
			world_vel = world_vel_from_delta(delta, world_vel);
			local_vel = local_vel_from_world(pos, world_vel);
			motors_v = omni_from_local_vel(local_vel);
		}
		// commit transaction
		control_set_pos(&shared_ctrl, pos);
		tmc2209_set_speed(shared_ctrl.m1, (int32_t)(motors_v.v1 * MM_TO_USTEPS / WHEEL_PERIMETER));
		tmc2209_set_speed(shared_ctrl.m2, (int32_t)(motors_v.v2 * MM_TO_USTEPS / WHEEL_PERIMETER));
		tmc2209_set_speed(shared_ctrl.m3, (int32_t)(motors_v.v3 * MM_TO_USTEPS / WHEEL_PERIMETER));
		// sleep
		// LOG_DBG("pos: %.2f %.2f %.2f", pos.x, pos.y, pos.a);
		// LOG_DBG("target: %.2f %.2f %.2f", target.x, target.y, target.a);
		// LOG_DBG("speed: %.2f %.2f %.2f", motors_v.v1, motors_v.v2, motors_v.v3);
		k_sleep(K_MSEC((uint64_t)CONTROL_PERIOD_MS));
	}
	LOG_INF("control task done (ret=%d)", ret);
	return ret;
}

#if CONFIG_CONTROL_TASK
K_THREAD_DEFINE(control_task_name, CONFIG_CONTROL_THREAD_STACK, control_task, NULL, NULL, NULL,
				CONFIG_CONTROL_THREAD_PRIORITY, 0, 0);
#endif

void _test_gconf()
{
	uint32_t gconf;
	// control_init(&shared_ctrl, &train_motor_1, &train_motor_2, &train_motor_3);
	k_sleep(K_MSEC(1000));
	while (1) {
		tmc2209_get_gconf(&train_motor_1, &gconf);
		k_sleep(K_MSEC(8000));
	}
}

void _test_motor_cmd()
{
	static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
	// control_init(&shared_ctrl, &train_motor_1, &train_motor_2, &train_motor_3);
	k_sleep(K_MSEC(1000));
	while (1) {
		gpio_pin_toggle(led.port, led.pin);
		tmc2209_set_speed(&train_motor_1, 0);
		tmc2209_set_speed(&train_motor_2, 0);
		tmc2209_set_speed(&train_motor_3, 0);
		gpio_pin_toggle(led.port, led.pin);
		tmc2209_set_speed(&train_motor_1, 10000);
		tmc2209_set_speed(&train_motor_2, 20000);
		tmc2209_set_speed(&train_motor_3, 40000);
		k_sleep(K_MSEC(1000));
		gpio_pin_toggle(led.port, led.pin);
		tmc2209_set_speed(&train_motor_1, 0);
		tmc2209_set_speed(&train_motor_2, 0);
		tmc2209_set_speed(&train_motor_3, 0);
		k_sleep(K_MSEC(1000));
		gpio_pin_toggle(led.port, led.pin);
		tmc2209_set_speed(&train_motor_1, 10000);
		tmc2209_set_speed(&train_motor_2, 0);
		tmc2209_set_speed(&train_motor_3, 0);
		k_sleep(K_MSEC(1000));
		gpio_pin_toggle(led.port, led.pin);
		tmc2209_set_speed(&train_motor_1, 0);
		tmc2209_set_speed(&train_motor_2, 10000);
		tmc2209_set_speed(&train_motor_3, 0);
		k_sleep(K_MSEC(1000));
		gpio_pin_toggle(led.port, led.pin);
		tmc2209_set_speed(&train_motor_1, 0);
		tmc2209_set_speed(&train_motor_2, 0);
		tmc2209_set_speed(&train_motor_3, 10000);
		k_sleep(K_MSEC(1000));
	}
}

void _test_target()
{
	LOG_INF("_test_target");
	static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
#if !(CONFIG_CONTROL_TASK)
	LOG_ERR("control task not launched");
#endif
	while (1) {
		if (!shared_ctrl.ready) {
			k_sleep(K_MSEC(100));
			continue;
		}
		break;
	}
	shared_ctrl.start = true;
	k_sleep(K_MSEC(5000));
	while (1) {
		gpio_pin_toggle(led.port, led.pin);
		control_set_target(&shared_ctrl, (pos2_t){100.0f, 100.0f, 1.0f * M_PI});
		LOG_DBG("1");
		k_sleep(K_MSEC(5000));
		gpio_pin_toggle(led.port, led.pin);
		control_set_target(&shared_ctrl, (pos2_t){0.0f, 0.0f, 0.0f * M_PI});
		LOG_DBG("2");
		k_sleep(K_MSEC(5000));
		gpio_pin_toggle(led.port, led.pin);
		control_set_target(&shared_ctrl, (pos2_t){100.0f, -100.0f, -2.0f * M_PI});
		LOG_DBG("3");
		k_sleep(K_MSEC(5000));
	}
}

void _test_calibration_distance()
{
	LOG_INF("_test_calibration");
	// static const struct gpio_dt_spec led =
	//     GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
	// int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
#if !(CONFIG_CONTROL_TASK)
	LOG_ERR("control task not launched");
#endif
	shared_ctrl.start_init = true;
	while (1) {
		if (!shared_ctrl.ready) {
			k_sleep(K_MSEC(100));
			continue;
		}
		break;
	}
	LOG_DBG("alive");
	// gpio_pin_toggle(led.port, led.pin);
	control_set_pos(&shared_ctrl, (pos2_t){0.0f, 0.0f, 0.0f});
	control_set_target(&shared_ctrl, (pos2_t){0.0f, 0.0f, 0.0f});
	LOG_DBG("pos: %.2f %.2f %.2f", shared_ctrl.pos.val.x, shared_ctrl.pos.val.y,
			shared_ctrl.pos.val.a);
	LOG_DBG("target: %.2f %.2f %.2f", shared_ctrl.target.val.x, shared_ctrl.target.val.y,
			shared_ctrl.target.val.a);
	k_sleep(K_MSEC(1000));
	shared_ctrl.start = true;
	// gpio_pin_toggle(led.port, led.pin);
	control_set_target(&shared_ctrl, (pos2_t){0.0f, 1300.0f, 0.0f * M_PI});
	LOG_DBG("pos: %.2f %.2f %.2f", shared_ctrl.pos.val.x, shared_ctrl.pos.val.y,
			shared_ctrl.pos.val.a);
	LOG_DBG("target: %.2f %.2f %.2f", shared_ctrl.target.val.x, shared_ctrl.target.val.y,
			shared_ctrl.target.val.a);
	k_sleep(K_MSEC(15000));
}

void _test_calibration_angle()
{
	LOG_INF("_test_calibration");
	// static const struct gpio_dt_spec led =
	//     GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
	// int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
#if !(CONFIG_CONTROL_TASK)
	LOG_ERR("control task not launched");
#endif
	shared_ctrl.start_init = true;
	while (1) {
		if (!shared_ctrl.ready) {
			k_sleep(K_MSEC(100));
			continue;
		}
		break;
	}
	LOG_DBG("alive");
	// gpio_pin_toggle(led.port, led.pin);
	control_set_pos(&shared_ctrl, (pos2_t){0.0f, 0.0f, 0.0f});
	control_set_target(&shared_ctrl, (pos2_t){0.0f, 0.0f, 0.0f});
	LOG_DBG("pos: %.2f %.2f %.2f", shared_ctrl.pos.val.x, shared_ctrl.pos.val.y,
			shared_ctrl.pos.val.a);
	LOG_DBG("target: %.2f %.2f %.2f", shared_ctrl.target.val.x, shared_ctrl.target.val.y,
			shared_ctrl.target.val.a);
	k_sleep(K_MSEC(1000));
	shared_ctrl.start = true;
	// gpio_pin_toggle(led.port, led.pin);
	control_set_target(&shared_ctrl, (pos2_t){0.0f, 0.0f, 20.0f * M_PI});
	k_sleep(K_MSEC(15000));
	LOG_DBG("pos: %.2f %.2f %.2f", shared_ctrl.pos.val.x, shared_ctrl.pos.val.y,
			shared_ctrl.pos.val.a);
	LOG_DBG("target: %.2f %.2f %.2f", shared_ctrl.target.val.x, shared_ctrl.target.val.y,
			shared_ctrl.target.val.a);
}

void _test_calibration_mix()
{
	LOG_INF("_test_calibration");
	// static const struct gpio_dt_spec led =
	//     GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
	// int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
#if !(CONFIG_CONTROL_TASK)
	LOG_ERR("control task not launched");
#endif
	shared_ctrl.start_init = true;
	while (1) {
		if (!shared_ctrl.ready) {
			k_sleep(K_MSEC(100));
			continue;
		}
		break;
	}
	LOG_DBG("alive");
	// gpio_pin_toggle(led.port, led.pin);
	control_set_pos(&shared_ctrl, (pos2_t){0.0f, 0.0f, 0.0f});
	control_set_target(&shared_ctrl, (pos2_t){0.0f, 0.0f, 0.0f});
	LOG_DBG("pos: %.2f %.2f %.2f", shared_ctrl.pos.val.x, shared_ctrl.pos.val.y,
			shared_ctrl.pos.val.a);
	LOG_DBG("target: %.2f %.2f %.2f", shared_ctrl.target.val.x, shared_ctrl.target.val.y,
			shared_ctrl.target.val.a);
	k_sleep(K_MSEC(1000));
	shared_ctrl.start = true;
	control_set_target(&shared_ctrl, (pos2_t){0.0f, 0.0f, 1.0f * M_PI});
	k_sleep(K_MSEC(5000));
	LOG_DBG("pos: %.2f %.2f %.2f", shared_ctrl.pos.val.x, shared_ctrl.pos.val.y,
			shared_ctrl.pos.val.a);
	LOG_DBG("target: %.2f %.2f %.2f", shared_ctrl.target.val.x, shared_ctrl.target.val.y,
			shared_ctrl.target.val.a);
	control_set_target(&shared_ctrl, (pos2_t){0.0f, 1000.0f, 1.0f * M_PI});
	k_sleep(K_MSEC(5000));
	LOG_DBG("pos: %.2f %.2f %.2f", shared_ctrl.pos.val.x, shared_ctrl.pos.val.y,
			shared_ctrl.pos.val.a);
	LOG_DBG("target: %.2f %.2f %.2f", shared_ctrl.target.val.x, shared_ctrl.target.val.y,
			shared_ctrl.target.val.a);
}

void _test_connerie()
{
	LOG_INF("_test_connerie");
#if !(CONFIG_CONTROL_TASK)
	LOG_ERR("control task not launched");
#endif
	shared_ctrl.start_init = true;
	while (1) {
		if (!shared_ctrl.ready) {
			k_sleep(K_MSEC(100));
			continue;
		}
		break;
	}
	shared_ctrl.start = true;
	LOG_DBG("alive");
	// gpio_pin_toggle(led.port, led.pin);
	control_set_pos(&shared_ctrl, (pos2_t){0.0f, 0.0f, 0.0f});
	control_set_target(&shared_ctrl, (pos2_t){0.0f, 0.0f, 100.0f});
	k_sleep(K_MSEC(15000));
	shared_ctrl.brake = true;
}
