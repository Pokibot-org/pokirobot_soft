#ifndef OBSTACLE_MANAGER_H
#define OBSTACLE_MANAGER_H
#include "stdbool.h"
#include "obstacles/obstacle.h"

#define ROBOT_MAX_RADIUS_MM 155
#define ROBOT_MIN_RADIUS_MM 50

typedef void (*obstacle_manager_collision_clbk)(bool collision);

void obstacle_manager_init(obstacle_manager_collision_clbk fun);
uint8_t obstacle_manager_get_obstacle_snapshot(obstacle_holder_t *obj);
void obstacle_manager_kill(void);
#endif
