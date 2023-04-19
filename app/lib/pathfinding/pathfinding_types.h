#ifndef PATHFINDING_TYPES_H
#define PATHFINDING_TYPES_H

#include <stdint.h>

#include "pokutils.h"

typedef struct path_node {
	struct path_node *parent_node;
	struct path_node *son_node;
	struct path_node *son_node_next;
	point2_t coordinate;
	uint8_t is_used;
	float distance_to_start;
} path_node_t;

/**
 * @brief Specify the path search space
 * example : for a space of 2m*2m with a resolution of 1mm
 * boundaries.x = 2000;
 * boundaries.y = 2000;
 */
typedef struct boundaries {
	float max_x;
	float max_y;
	float min_x;
	float min_y;
} boundaries_t;

#endif
