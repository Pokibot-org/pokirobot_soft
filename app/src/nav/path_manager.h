#ifndef PATH_MANAGER_H
#define PATH_MANAGER_H
#include <stdint.h>

#include "pathfinding/pathfinding_types.h"

typedef void (*path_manager_found_path_clbk)(const path_node_t *, void *); // void * == user data
typedef void (*path_manager_found_updated_path_clbk)(const path_node_t *, void *);

/**
 * @brief Configuration structure for the path_manager functions
 * You must add a found_path_clbk.
 * The field found_updated_path_clbk is optional.
 * If registered the path_manager will periodicaly ask the obstacle_manager
 * updated info and recalculate path. The field user_config will be passed to
 * callback functions
 */
typedef struct path_manager_config {
	path_manager_found_path_clbk found_path_clbk;
	path_manager_found_updated_path_clbk found_updated_path_clbk;
	uint32_t nb_node_optimisation;
	void *user_config;
} path_manager_config_t;

uint8_t path_manager_find_path(point2_t start, point2_t end, path_manager_config_t config);
uint16_t path_manager_retrieve_path(point2_t *array, uint32_t array_size,
									point2_t **ptr_array_start, const path_node_t *end_node);
#endif
