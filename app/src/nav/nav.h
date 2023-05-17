#ifndef NAV_H
#define NAV_H

#include "pokutils.h"
#include "obstacles/obstacle.h"

int nav_go_to_with_pathfinding(pos2_t end_pos, obstacle_t *obstacle_list,
                               uint8_t obstacle_list_len);
void nav_init(void);
void nav_stop(void);

#endif