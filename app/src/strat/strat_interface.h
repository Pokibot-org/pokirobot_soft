#ifndef STRAT_INTERFACE_H
#define STRAT_INTERFACE_H
#include "pokutils.h"

#define ROBOT_RADIUS                             190
#define ROBOT_POKSTICK_LEN                       20
#define STRAT_PLANAR_TARGET_SENSITIVITY_DEFAULT  2.0f             // 2mm
#define STRAT_ANGULAR_TARGET_SENSITIVITY_DEFAULT DEG_TO_RAD(3.0f) // 3 deg
#define STRAT_WAIT_OK                            0
#define STRAT_WAIT_TIMEOUT_TARGET                (-1)
#define STRAT_WAIT_TIMEOUT_BRAKE                 (-2)


void strat_control_start(void);
int strat_set_robot_brake(bool brake);
int strat_get_robot_pos(pos2_t *pos);
int strat_set_robot_pos(pos2_t pos);
int strat_set_target(pos2_t pos);
float strat_get_robot_dir_angle(void);
int strat_set_waypoints(pos2_t *pos_list, int n);
int strat_wait_target(float planar_sensivity, float angular_sensivity, uint32_t timeout_target_ms,
                      uint32_t timeout_brake_ms);
#define strat_wait_target_default(_timeout_target_ms, _timeout_brake_ms)                           \
    strat_wait_target(STRAT_PLANAR_TARGET_SENSITIVITY_DEFAULT,                                     \
                      STRAT_ANGULAR_TARGET_SENSITIVITY_DEFAULT, _timeout_target_ms,                \
                      _timeout_target_ms)
void strat_force_motor_stop(void);

int strat_move_robot_to(pos2_t pos, k_timeout_t timeout);

#endif
