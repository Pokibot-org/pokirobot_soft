#include "strat_interface.h"
#include "control/control.h"

int strat_get_robot_pos(pos2_t *pos)
{
	return control_get_pos(&shared_ctrl, pos);
}

int strat_set_robot_pos(pos2_t pos)
{
	return control_set_pos(&shared_ctrl, pos);
}

int strat_move_robot_to(pos2_t pos, k_timeout_t timeout)
{
	if (!shared_ctrl.start) {
		shared_ctrl.start = true;
	}

	if (control_set_target(&shared_ctrl, pos)) {
		return -1;
	}

	if (!control_task_wait_target(30.0f, DEG_TO_RAD(10.0f), k_ticks_to_ms_near64(timeout.ticks))) {
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
	return -1;
}
