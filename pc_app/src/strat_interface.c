#include "strat/strat_interface.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(strat_interface);
#define LOG_POS(pos) LOG_DBG("position<x: %3.2f,y: %3.2f,a: %3.2f>", (pos).x, (pos).y, (pos).a)

#define CONTROL_PERIOD_MS	  2.0f
#define CONTROL_MUTEX_TIMEOUT (K_MSEC(30))

typedef struct control {
	bool start;
	bool at_target;
	LOCKVAR(pos2_t) pos;
	LOCKVAR(pos2_t) target;
} control_t;
control_t fake_ctrl;

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

int strat_get_robot_pos(pos2_t *pos)
{
	int ret = control_get_pos(&fake_ctrl, pos);
	LOG_POS(*pos);
	return ret;
}

int strat_set_robot_pos(pos2_t pos)
{
	LOG_POS(pos);
	return control_set_pos(&fake_ctrl, pos);
}

bool control_task_wait_target(uint32_t timeout_ms)
{
	fake_ctrl.at_target = false;
	for (int i = 0; i < timeout_ms; i++) {
		if (fake_ctrl.at_target) {
			LOG_DBG("At target");
			break;
		}
		k_sleep(K_MSEC(1));
	}
	return fake_ctrl.at_target;
}

int strat_move_robot_to(pos2_t pos, k_timeout_t timeout)
{
	if (!fake_ctrl.start) {
		fake_ctrl.start = true;
	}

	if (control_set_target(&fake_ctrl, pos)) {
		return -1;
	}

	if (!control_task_wait_target(k_ticks_to_ms_near64(timeout.ticks))) {
		return -2;
	}
	return 0;
}

int strat_grab_layer(pos2_t layer_pos, k_timeout_t timeout)
{
	k_sleep(K_SECONDS(2));
	return 0;
}

int strat_put_layer(pos2_t plate_pos, uint8_t current_cake_height, k_timeout_t timeout)
{
	k_sleep(K_SECONDS(1));
	return 0;
}

void fake_control_task(void)
{
	while (1) {
		pos2_t current_pos, target_pos;
		control_get_pos(&fake_ctrl, &current_pos);
		control_get_target(&fake_ctrl, &target_pos);
		point2_t current_point = {.x = current_pos.x, .y = current_pos.y};
		point2_t target_point = {.x = target_pos.x, .y = target_pos.y};

		vec2_t diff = point2_diff(target_point, current_point);

		vec2_t norm = vec2_normalize(diff);

		const float speed = 5.0f;
		current_point.x += fminf(norm.dx * speed, diff.dx);
		current_point.y += fminf(norm.dy * speed, diff.dy);

		if (target_point.x == current_point.x && target_point.y == current_point.y) {
			fake_ctrl.at_target = true;
		}
		control_set_pos(&fake_ctrl,
						(pos2_t){.x = current_point.x, .y = current_point.y, .a = target_pos.a});
		k_sleep(K_MSEC((uint64_t)CONTROL_PERIOD_MS));
	}
}

K_THREAD_DEFINE(fake_control, 1024, fake_control_task, NULL, NULL, NULL, 5, 0, 0);

int fake_init(const struct device *dev)
{
	INIT_LOCKVAR(fake_ctrl.pos);
	INIT_LOCKVAR(fake_ctrl.target);
	return 0;
}

SYS_INIT(fake_init, APPLICATION, 1);