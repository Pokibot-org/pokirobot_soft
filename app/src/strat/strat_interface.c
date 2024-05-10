#include "strat_interface.h"
#include "control/control.h"
#include "pokuicom/pokuicom.h"

int strat_set_robot_brake(bool brake)
{
    return control_set_brake(&shared_ctrl, brake);
}

int strat_get_robot_pos(pos2_t *pos)
{
    return control_get_pos(&shared_ctrl, pos);
}

float strat_get_robot_dir_angle(void)
{
    return shared_ctrl.dir_angle;
}

int strat_set_robot_pos(pos2_t pos)
{
    return control_set_pos(&shared_ctrl, pos);
}

int strat_set_target(pos2_t pos)
{
    return control_set_waypoints(&shared_ctrl, &pos, 1);
}

int strat_set_waypoints(pos2_t *pos_list, int n)
{
    return control_set_waypoints(&shared_ctrl, pos_list, n);
}

int strat_wait_target(float planar_sensivity, float angular_sensivity, uint32_t timeout_target_ms,
                      uint32_t timeout_brake_ms)
{
    return control_task_wait_target(planar_sensivity, angular_sensivity, timeout_target_ms,
                                    timeout_brake_ms);
}

void strat_force_motor_stop(void)
{
    control_force_motor_stop();
}

void strat_control_start(void) {
    shared_ctrl.start = true;
}

int strat_move_robot_to(pos2_t pos, k_timeout_t timeout)
{
    if (!shared_ctrl.start) {
        shared_ctrl.start = true;
    }

    if (strat_set_target(pos)) {
        return -1;
    }

    if (strat_wait_target_default(k_ticks_to_ms_near64(timeout.ticks),
                                  k_ticks_to_ms_near64(timeout.ticks))) {
        return -2;
    }
    return 0;
}

void start_control_set_ihold_irun(int irun, int ihold)
{
    control_set_ihold_irun(irun, ihold);
}