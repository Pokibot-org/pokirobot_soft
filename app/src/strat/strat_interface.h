#ifndef STRAT_INTERFACE_H
#define STRAT_INTERFACE_H
#include "pokutils.h"

#define ROBOT_RADIUS 150

int strat_get_robot_pos(pos2_t *pos);
int strat_set_robot_pos(pos2_t pos);
int strat_move_robot_to(pos2_t pos, k_timeout_t timeout);

int strat_grab_layer(pos2_t layer_pos, k_timeout_t timeout);
int strat_put_layer(pos2_t plate_pos, uint8_t current_cake_height, k_timeout_t timeout);

#endif
