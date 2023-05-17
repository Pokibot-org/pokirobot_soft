#ifndef STRAT_INTERFACE_H
#define STRAT_INTERFACE_H
#include "pokutils.h"

#define ROBOT_RADIUS                             190
#define STRAT_PLANAR_TARGET_SENSITIVITY_DEFAULT  5.0f             // 5mm
#define STRAT_ANGULAR_TARGET_SENSITIVITY_DEFAULT DEG_TO_RAD(3.0f) // 3 deg

int strat_set_robot_brake(bool brake);
int strat_get_robot_pos(pos2_t *pos);
int strat_set_robot_pos(pos2_t pos);
int strat_set_target(pos2_t pos);
bool strat_wait_target(float planar_sensivity, float angular_sensivity, uint32_t timeout_ms);
#define strat_wait_target_default(_timeout_ms)                                                     \
    strat_wait_target(STRAT_PLANAR_TARGET_SENSITIVITY_DEFAULT,                                     \
                      STRAT_ANGULAR_TARGET_SENSITIVITY_DEFAULT, _timeout_ms)

int strat_move_robot_to(pos2_t pos, k_timeout_t timeout);

int strat_grab_layer(pos2_t layer_pos, k_timeout_t timeout);
int strat_put_layer(pos2_t plate_pos, uint8_t current_cake_height, k_timeout_t timeout);

#endif
