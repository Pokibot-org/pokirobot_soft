#ifndef PATHFINDING_H
#define PATHFINDING_H
#include <stdint.h>

#include "obstacles/obstacle.h"
#include "pathfinding_errors.h"
#include "pathfinding_types.h"

#define PATHFINDING_MAX_NUM_OF_NODES 1024

typedef struct pathfinding_configuration {
	boundaries_t field_boundaries;
	float delta_distance;
	float radius_of_security; // Must be radius of the robot + some
	float node_remapping_distance;
	uint8_t rand_goal_probability; // A sampling rate of 1/2 = 256 / 2
	uint8_t rand_waypoint_probability;
} pathfinding_configuration_t;

typedef struct pathfinding_object {
	path_node_t nodes[PATHFINDING_MAX_NUM_OF_NODES];
	pathfinding_configuration_t config;
	uint32_t next_free_node_nb;
} pathfinding_object_t;

int pathfinding_object_configure(pathfinding_object_t *obj, pathfinding_configuration_t *config);
int pathfinding_find_path(pathfinding_object_t *obj, obstacle_holder_t *ob_hold, point2_t start,
						  point2_t end, path_node_t **end_node);
int pathfinding_optimize_path(pathfinding_object_t *obj, obstacle_holder_t *ob_hold,
							  path_node_t *solved_path_end_node, uint16_t nb_of_nodes_to_add);

uint16_t pathfinding_get_number_of_used_nodes(pathfinding_object_t *obj);
#ifdef UNIT_TEST
void get_new_valid_coordinates(pathfinding_object_t *obj, point2_t crd_tree_node,
							   point2_t crd_random_node, point2_t *crd_new_node);
void pathfinding_debug_print(pathfinding_object_t *obj, obstacle_holder_t *obs_holder);
void pathfinding_debug_print_found_path(pathfinding_object_t *obj, obstacle_holder_t *obs_holder,
										path_node_t *end_node);

path_node_t *get_closest_node(pathfinding_object_t *obj, point2_t crd);
#endif

#endif
