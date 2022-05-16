#ifndef OBSTACLE_MANAGER_H
#define OBSTACLE_MANAGER_H
#include "obstacles/obstacle.h"

#define ROBOT_MAX_RADIUS_MM 200
#define ROBOT_MIN_RADIUS_MM 50

void obstacle_manager_init(void);
uint8_t obstacle_manager_get_obstacle_snapshot(obstacle_holder_t *obj);

#endif
