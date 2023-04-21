#include "pathfinding.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include "obstacles/obstacle.h"
#include "pokutils/robot_utils.h"

LOG_MODULE_REGISTER(PATHFINDING, 1);

#define DEBUG_TAB_SIZE_X 120
#define DEBUG_TAB_SIZE_Y 40

#define PATHFINDING_GET_ARRAY_OF_CLOSEST_NODES_MAX_NUM 40

// DECLARATION

static uint8_t get_closest_point_of_collision(const pathfinding_object_t *obj,
											  const obstacle_holder_t *ob_hold,
											  const point2_t rand_coordinates,
											  const point2_t closest_node_coordinates,
											  point2_t *out_crd);

// FUNCTIONS

uint16_t pathfinding_get_number_of_used_nodes(pathfinding_object_t *obj)
{
	uint16_t res = 0;
	for (uint16_t i = 0; i < PATHFINDING_MAX_NUM_OF_NODES; i++) {
		if (obj->nodes[i].is_used) {
			res += 1;
		}
	}
	return res;
}

int pathfinding_object_configure(pathfinding_object_t *obj, pathfinding_configuration_t *config)
{
	// TODO: Check if delta is under distance to goal
	obj->config = *config;
	if ((obj->config.field_boundaries.max_x == obj->config.field_boundaries.max_y) &&
		(obj->config.field_boundaries.max_x == 0)) {
		obj->config.field_boundaries =
			(boundaries_t){__FLT_MAX__, __FLT_MAX__, __FLT_MIN__, __FLT_MIN__};
	}
	if (!obj->config.delta_distance) {
		obj->config.delta_distance =
			(obj->config.field_boundaries.max_x + obj->config.field_boundaries.max_y) / 10;
	}
	if (!obj->config.radius_of_security) {
		obj->config.radius_of_security = 10;
	}
	if (!obj->config.rand_goal_probability) {
		obj->config.rand_goal_probability = 256 / 8;
	}
	if (!obj->config.rand_waypoint_probability) {
		obj->config.rand_waypoint_probability = 256 / 2;
	}

	obj->config.node_remapping_distance = obj->config.delta_distance - 1;

	obj->next_free_node_nb = 0;

	return PATHFINDING_ERROR_NONE;
}

path_node_t *get_closest_node(pathfinding_object_t *obj, point2_t crd)
{
	path_node_t *closest_node_p = NULL;
	uint32_t closest_node_distance = UINT32_MAX;

	for (size_t i = 0; i < obj->next_free_node_nb; i++) {
		if (obj->nodes[i].is_used) {
			uint32_t distance = vec2_abs(point2_diff(obj->nodes[i].coordinate, crd));
			if (distance < closest_node_distance) {
				closest_node_distance = distance;
				closest_node_p = &obj->nodes[i];
			}
		}
	}
	return closest_node_p;
}

uint8_t get_array_of_closest_node(pathfinding_object_t *obj, point2_t crd, path_node_t **out_nodes)
{
	uint8_t found_nodes = 0;
	for (size_t i = 0; i < obj->next_free_node_nb; i++) {
		path_node_t *current_node = &obj->nodes[i];
		if (current_node->is_used) {
			float distance = vec2_abs(point2_diff(current_node->coordinate, crd));
			if (distance <= obj->config.node_remapping_distance) {
				out_nodes[found_nodes] = current_node;
				found_nodes += 1;
				if (found_nodes == PATHFINDING_GET_ARRAY_OF_CLOSEST_NODES_MAX_NUM) {
					break;
				}
			}
		}
	}

	return found_nodes;
}

void remap_nodes_to_new_node_if_closer_to_start(pathfinding_object_t *obj,
												obstacle_holder_t *ob_hold, path_node_t **nodes,
												uint8_t nb_nodes, path_node_t *new_node)
{
	for (size_t i = 0; i < nb_nodes; i++) {
		point2_t out_crd;
		int err = get_closest_point_of_collision(obj, ob_hold, new_node->coordinate,
												 nodes[i]->coordinate, &out_crd);
		if (err) {
			continue;
		}
		float total_distance = vec2_abs(point2_diff(new_node->coordinate, nodes[i]->coordinate)) +
							   new_node->distance_to_start;

		if (total_distance < nodes[i]->distance_to_start) {
			nodes[i]->distance_to_start = total_distance;
			nodes[i]->parent_node = new_node;
		}
	}
}

int get_new_valid_coordinates(pathfinding_object_t *obj, point2_t crd_tree_node,
							  point2_t crd_random_node, point2_t *crd_new_node)
{
	float segment_len = vec2_abs(point2_diff(crd_tree_node, crd_random_node));
	if (segment_len <= obj->config.delta_distance) {
		*crd_new_node = crd_random_node;
		return PATHFINDING_ERROR_NONE;
	}

	float frac = (float)obj->config.delta_distance / segment_len;

	crd_new_node->x = crd_tree_node.x + (crd_random_node.x - crd_tree_node.x) * frac;
	crd_new_node->y = crd_tree_node.y + (crd_random_node.y - crd_tree_node.y) * frac;
	return PATHFINDING_ERROR_NONE;
}

/**
 * @brief Retun 0 if there is no collision
 *
 */
uint8_t get_closest_point_of_collision(const pathfinding_object_t *obj,
									   const obstacle_holder_t *ob_hold, point2_t rand_coordinates,
									   point2_t closest_node_coordinates, point2_t *out_crd)
{
	*out_crd = rand_coordinates;
	point2_t obstacle_checked_crd = {0};
	uint8_t collision_happened = 0;
	float closest_obstacle = MAXFLOAT;
	for (size_t index_obstacle = 0; index_obstacle < OBSTACLE_HOLDER_MAX_NUMBER_OF_OBSTACLE;
		 index_obstacle++) {
		const obstacle_t *current_ob = &ob_hold->obstacles[index_obstacle];
		if (current_ob->type != obstacle_type_none) {
			int status = obstacle_get_point_of_collision_with_segment(
				closest_node_coordinates, rand_coordinates, current_ob,
				obj->config.radius_of_security, &obstacle_checked_crd);
			if (status == 1) {
				collision_happened = 1;
				// FIXME: check if it is the closest intersection point
				float dist = vec2_abs(point2_diff(obstacle_checked_crd, rand_coordinates));
				if (dist < closest_obstacle) {
					*out_crd = obstacle_checked_crd;
				}
				// printf("status %d, HOY seg : x:%d y:%d | x:%d y:%d\n goal
				// x:%d y:%d\n",status, closest_node_p->coordinate.x,
				// closest_node_p->coordinate.y, rand_coordinates.x,
				// rand_coordinates.y, path_free_crd.x, path_free_crd.y);
			} else if (status == 2) {
				collision_happened = 1;
				break;
			}
		}
	}
	return collision_happened;
}

enum obstacle_collision_status check_collision(const pathfinding_object_t *obj,
											   const obstacle_holder_t *ob_hold, point2_t pt_seg_a,
											   point2_t pt_seg_b)
{
	point2_t obstacle_checked_crd;
	for (size_t index_obstacle = 0; index_obstacle < ob_hold->write_head; index_obstacle++) {
		const obstacle_t *current_ob = &ob_hold->obstacles[index_obstacle];
		if (current_ob->type != obstacle_type_none) {
			uint8_t status = obstacle_get_point_of_collision_with_segment(
				pt_seg_a, pt_seg_b, current_ob, obj->config.radius_of_security,
				&obstacle_checked_crd);
			if (status == OBSTACLE_COLLISION_DETECTED) {
				return OBSTACLE_COLLISION_DETECTED;
			}
		}
	}
	return OBSTACLE_COLLISION_NONE;
}

enum obstacle_collision_status check_obstacle_collision(const pathfinding_object_t *obj,
														const obstacle_holder_t *ob_hold,
														const obstacle_t *obstacle)
{
	for (size_t index_obstacle = 0; index_obstacle < ob_hold->write_head; index_obstacle++) {
		const obstacle_t *current_ob = &ob_hold->obstacles[index_obstacle];
		if (current_ob->type != obstacle_type_none) {
			uint8_t status = obstacle_are_they_colliding(current_ob, obstacle);
			if (status == OBSTACLE_COLLISION_DETECTED) {
				return OBSTACLE_COLLISION_DETECTED;
			}
		}
	}
	return OBSTACLE_COLLISION_NONE;
}

point2_t pathfinding_generate_rand_coordinates(const pathfinding_object_t *obj, point2_t end)
{
	uint8_t new_crd_on_goal = (utils_get_rand32() & 0x000000FF) < obj->config.rand_goal_probability;
	point2_t rand_coordinates;
	if (new_crd_on_goal) {
		rand_coordinates = end;
	} else {
		rand_coordinates.x = utils_get_randf_in_range(obj->config.field_boundaries.min_x,
													  obj->config.field_boundaries.max_x);
		rand_coordinates.y = utils_get_randf_in_range(obj->config.field_boundaries.min_y,
													  obj->config.field_boundaries.max_y);
	}
	return rand_coordinates;
}

point2_t pathfinding_generate_rebuild_rand_coordinates(const pathfinding_object_t *obj,
													   const point2_t end)
{
	uint32_t rand_num = utils_get_rand32();
	uint8_t point_gen_type_probability = rand_num & 0x000000FF;
	point2_t rand_coordinates;
	if (point_gen_type_probability < obj->config.rand_goal_probability) {
		rand_coordinates = end;
	} else if ((point_gen_type_probability <
				obj->config.rand_goal_probability + obj->config.rand_waypoint_probability) &&
			   obj->nb_of_coordinate_cached) {
		rand_coordinates = obj->coordinate_cache[(rand_num >> 24) % obj->nb_of_coordinate_cached];
	} else {
		rand_coordinates.x = utils_get_randf_in_range(obj->config.field_boundaries.min_x,
													  obj->config.field_boundaries.max_x);
		rand_coordinates.y = utils_get_randf_in_range(obj->config.field_boundaries.min_y,
													  obj->config.field_boundaries.max_y);
	}
	return rand_coordinates;
}

int pathfinding_cache_waypoint(pathfinding_object_t *obj, path_node_t *end_node)
{
	obj->nb_of_coordinate_cached = 0;
	path_node_t *current_node = end_node;
	while (current_node->parent_node != NULL) {
		if (obj->nb_of_coordinate_cached == PATHFINDING_MAX_NUM_OF_COORDINATES_CACHE) {
			return -1;
		}
		obj->coordinate_cache[obj->nb_of_coordinate_cached] = current_node->coordinate;
		obj->nb_of_coordinate_cached += 1;
		current_node = current_node->parent_node;
	}
	return 0;
}

int pathfinding_find_path(pathfinding_object_t *obj, obstacle_holder_t *ob_hold, point2_t start,
						  point2_t end, path_node_t **end_node)
{
	LOG_INF("Stating pathfinding at node x:%f, y%f towards x:%f, y%f ", start.x, start.y, end.x,
			end.y);
	*end_node = NULL;
	// TODO: Check input validity, must be between 0 and pathfinding_boundaries
	// Init start node
	// TODO :must be bound to current tree if existing
	obj->nodes[0].coordinate = start;
	obj->nodes[0].parent_node = NULL;
	obj->nodes[0].is_used = 1;
	obj->nodes[0].distance_to_start = 0;

	if (start.x < obj->config.field_boundaries.min_x ||
		start.y < obj->config.field_boundaries.min_y ||
		start.x >= obj->config.field_boundaries.max_x ||
		start.y >= obj->config.field_boundaries.max_y) {
		return PATHFINDING_ERROR_WRONG_INPUTS;
	}

	for (obj->next_free_node_nb = 1; obj->next_free_node_nb < PATHFINDING_MAX_NUM_OF_NODES;
		 obj->next_free_node_nb++) {
		path_node_t *current_node = &obj->nodes[obj->next_free_node_nb];
		point2_t rand_coordinates = pathfinding_generate_rand_coordinates(obj, end);
		LOG_DBG("Random coordinates generated : x:%f y:%f", rand_coordinates.x, rand_coordinates.y);
		path_node_t *closest_node_p = get_closest_node(obj, rand_coordinates);
		LOG_DBG("Closest node %p", closest_node_p);
		if (check_collision(obj, ob_hold, closest_node_p->coordinate, rand_coordinates) ==
			OBSTACLE_COLLISION_DETECTED) {
			LOG_DBG("Collison, dumping node");
			obj->next_free_node_nb -= 1;
			continue;
		}
		point2_t new_coordinates;
		get_new_valid_coordinates(obj, closest_node_p->coordinate, rand_coordinates,
								  &new_coordinates);
		LOG_DBG("New valid coordinates generated : x:%f y:%f", new_coordinates.x,
				new_coordinates.y);
		// Remaping nodes for RRT*
		path_node_t *to_be_remaped_nodes[PATHFINDING_GET_ARRAY_OF_CLOSEST_NODES_MAX_NUM] = {0};
		uint8_t nb_of_to_be_remaped_nodes =
			get_array_of_closest_node(obj, new_coordinates, to_be_remaped_nodes);

		current_node->is_used = 1;
		current_node->coordinate = new_coordinates;
		current_node->parent_node = closest_node_p;
		current_node->distance_to_start =
			closest_node_p->distance_to_start +
			vec2_abs(point2_diff(closest_node_p->coordinate, current_node->coordinate));

		remap_nodes_to_new_node_if_closer_to_start(obj, ob_hold, to_be_remaped_nodes,
												   nb_of_to_be_remaped_nodes, current_node);

		if ((new_coordinates.x == end.x) && (new_coordinates.y == end.y)) {
			*end_node = current_node;
			int err = pathfinding_cache_waypoint(obj, current_node);
			LOG_DBG("End of pathfinding");
			if (err) {
				return PATHFINDING_ERROR_NOT_ENOUGH_CACHE;
			}
			return PATHFINDING_ERROR_NONE;
		}
	}

	return PATHFINDING_ERROR_PATH_NOT_FOUND;
}

int pathfinding_rebuild(pathfinding_object_t *obj, obstacle_holder_t *ob_hold, point2_t start,
						point2_t end, path_node_t **end_node)
{
	*end_node = NULL;
	// TODO: Check input validity, must be between 0 and pathfinding_boundaries
	// Init start node
	// TODO :must be bound to current tree if existing
	obj->nodes[0].coordinate = start;
	obj->nodes[0].parent_node = NULL;
	obj->nodes[0].is_used = 1;
	obj->nodes[0].distance_to_start = 0;

	if (start.x < obj->config.field_boundaries.min_x ||
		start.y < obj->config.field_boundaries.min_y ||
		start.x >= obj->config.field_boundaries.max_x ||
		start.y >= obj->config.field_boundaries.max_y) {
		return PATHFINDING_ERROR_WRONG_INPUTS;
	}

	for (obj->next_free_node_nb = 1; obj->next_free_node_nb < PATHFINDING_MAX_NUM_OF_NODES;
		 obj->next_free_node_nb++) {
		path_node_t *current_node = &obj->nodes[obj->next_free_node_nb];
		point2_t rand_coordinates = pathfinding_generate_rebuild_rand_coordinates(obj, end);
		path_node_t *closest_node_p = get_closest_node(obj, rand_coordinates);

		if (check_collision(obj, ob_hold, rand_coordinates, closest_node_p->coordinate) ==
			OBSTACLE_COLLISION_DETECTED) {
			obj->next_free_node_nb -= 1;
			continue;
		}
		point2_t new_coordinates;
		get_new_valid_coordinates(obj, closest_node_p->coordinate, rand_coordinates,
								  &new_coordinates);

		// Remaping nodes for RRT*
		path_node_t *to_be_remaped_nodes[PATHFINDING_GET_ARRAY_OF_CLOSEST_NODES_MAX_NUM] = {0};
		uint8_t nb_of_to_be_remaped_nodes =
			get_array_of_closest_node(obj, new_coordinates, to_be_remaped_nodes);

		current_node->is_used = 1;
		current_node->coordinate = new_coordinates;
		current_node->parent_node = closest_node_p;
		current_node->distance_to_start =
			closest_node_p->distance_to_start +
			vec2_abs(point2_diff(closest_node_p->coordinate, current_node->coordinate));

		remap_nodes_to_new_node_if_closer_to_start(obj, ob_hold, to_be_remaped_nodes,
												   nb_of_to_be_remaped_nodes, current_node);

		if ((new_coordinates.x == end.x) && (new_coordinates.y == end.y)) {
			*end_node = current_node;
			return PATHFINDING_ERROR_NONE;
		}
	}
	return PATHFINDING_ERROR_PATH_NOT_FOUND;
}

// TODO: Optimize path and reconfigure when there is an obtacle

int pathfinding_optimize_path(pathfinding_object_t *obj, obstacle_holder_t *ob_hold,
							  path_node_t *solved_path_end_node, uint16_t nb_of_nodes_to_add)
{
	uint16_t counter = 0;
	path_node_t *to_be_remaped_nodes[PATHFINDING_GET_ARRAY_OF_CLOSEST_NODES_MAX_NUM] = {0};

	for (; obj->next_free_node_nb < PATHFINDING_MAX_NUM_OF_NODES; obj->next_free_node_nb++) {
		path_node_t *current_node = &obj->nodes[obj->next_free_node_nb];
		if (current_node->is_used) {
			uint8_t nb_of_to_be_remaped_nodes =
				get_array_of_closest_node(obj, current_node->coordinate, to_be_remaped_nodes);
			remap_nodes_to_new_node_if_closer_to_start(obj, ob_hold, to_be_remaped_nodes,
													   nb_of_to_be_remaped_nodes, current_node);
			continue;
		}
		// FIXME: in middle of the node array thereis unused nodes, you must
		// optimise all existing nodes and not stop to the first unused nodes
		// when nb_of_nodes_to_add = 0
		point2_t rand_coordinates;
		rand_coordinates.x = utils_get_randf_in_range(obj->config.field_boundaries.min_x,
													  obj->config.field_boundaries.max_x);
		rand_coordinates.y = utils_get_randf_in_range(obj->config.field_boundaries.min_y,
													  obj->config.field_boundaries.max_y);
		path_node_t *closest_node_p = get_closest_node(obj, rand_coordinates);
		// printf("rand crd x:%d y:%d\n Closest node x:%d y:%d\n",
		// rand_coordinates.x, rand_coordinates.y, closest_node_p->coordinate.x,
		// closest_node_p->coordinate.y);

		point2_t path_free_crd;
		int err = get_closest_point_of_collision(obj, ob_hold, rand_coordinates,
												 closest_node_p->coordinate, &path_free_crd);
		if (err) {
			continue;
		}
		// TODO: check if collision
		point2_t new_coordinates;
		get_new_valid_coordinates(obj, closest_node_p->coordinate, path_free_crd, &new_coordinates);

		// Remaping nodes for RRT*
		uint8_t nb_of_to_be_remaped_nodes =
			get_array_of_closest_node(obj, new_coordinates, to_be_remaped_nodes);

		current_node->is_used = 1;
		current_node->coordinate = new_coordinates;
		current_node->parent_node = closest_node_p;
		current_node->distance_to_start =
			closest_node_p->distance_to_start +
			vec2_abs(point2_diff(closest_node_p->coordinate, current_node->coordinate));

		remap_nodes_to_new_node_if_closer_to_start(obj, ob_hold, to_be_remaped_nodes,
												   nb_of_to_be_remaped_nodes, current_node);
		counter++;
		if (counter >= nb_of_nodes_to_add) {
			break;
		}
	}
	return PATHFINDING_ERROR_NONE;
}

// DEBUG FUNCTIONS
#ifdef UNIT_TEST
#include <stdio.h>

void pathfinding_debug_print(pathfinding_object_t *obj, obstacle_holder_t *obs_holder)
{
	uint8_t tab[DEBUG_TAB_SIZE_Y][DEBUG_TAB_SIZE_X] = {0};
	for (size_t i = 0; i < PATHFINDING_MAX_NUM_OF_NODES; i++) {
		if (obj->nodes[i].is_used) {
			uint16_t y =
				obj->nodes[i].coordinate.y * DEBUG_TAB_SIZE_Y / obj->config.field_boundaries.max_y;
			uint16_t x =
				obj->nodes[i].coordinate.x * DEBUG_TAB_SIZE_X / obj->config.field_boundaries.max_x;
			// printf("%d %d | %d %d\n", y, x, obj->nodes[i].coordinate.y,
			// obj->nodes[i].coordinate.x);
			tab[y][x] = 1;
		}
	}
	// printf("FB: %d %d\n", obj->config.field_boundaries.x,
	// obj->config.field_boundaries.y);
	for (size_t y = 0; y < DEBUG_TAB_SIZE_Y; y++) {
		for (size_t x = 0; x < DEBUG_TAB_SIZE_X; x++) {
			obstacle_t obs = (obstacle_t){
				.type = obstacle_type_circle,
				.data.circle = (circle_t){
					.coordinates =
						{
							.x = x * obj->config.field_boundaries.max_x / DEBUG_TAB_SIZE_X,
							.y = y * obj->config.field_boundaries.max_y / DEBUG_TAB_SIZE_Y,
						},
					.radius = DEBUG_TAB_SIZE_Y}};
			char b = '.';
			if (check_obstacle_collision(obj, obs_holder, &obs) == OBSTACLE_COLLISION_DETECTED) {
				b = '^';
			}
			char c = tab[y][x] ? 'X' : b;
			printf("%c", c);
		}
		printf("\n");
	}
};

void pathfinding_debug_print_found_path(pathfinding_object_t *obj, obstacle_holder_t *obs_holder,
										path_node_t *end_node)
{
	printf("\n================================================================================\n");
	if (end_node == NULL) {
		printf("end_node is NULL! \n");
		return;
	}
	uint8_t tab[DEBUG_TAB_SIZE_Y][DEBUG_TAB_SIZE_X] = {0};
	uint8_t path_valid = 0;
	path_node_t *current_node = end_node;
	for (size_t i = 0; i < PATHFINDING_MAX_NUM_OF_NODES; i++) {
		uint16_t y =
			current_node->coordinate.y * DEBUG_TAB_SIZE_Y / obj->config.field_boundaries.max_y;
		uint16_t x =
			current_node->coordinate.x * DEBUG_TAB_SIZE_X / obj->config.field_boundaries.max_x;
		tab[y][x] = 1;
		if (current_node->parent_node == NULL) {
			path_valid = 1;
			break;
		}
		current_node = current_node->parent_node;
	}
	if (!path_valid) {
		printf("Not a valid path!\n");
		return;
	}
	// printf("FB: %d %d\n", obj->config.field_boundaries.x,
	// obj->config.field_boundaries.y);
	for (size_t y = 0; y < DEBUG_TAB_SIZE_Y; y++) {
		for (size_t x = 0; x < DEBUG_TAB_SIZE_X; x++) {
			obstacle_t obs = (obstacle_t){
				.type = obstacle_type_circle,
				.data.circle = (circle_t){
					.coordinates =
						{
							.x = x * obj->config.field_boundaries.max_x / DEBUG_TAB_SIZE_X,
							.y = y * obj->config.field_boundaries.max_y / DEBUG_TAB_SIZE_Y,
						},
					.radius = DEBUG_TAB_SIZE_Y}};

			char b = '.';
			if (check_obstacle_collision(obj, obs_holder, &obs) == OBSTACLE_COLLISION_DETECTED) {
				b = '^';
			}
			char c = tab[y][x] ? 'X' : b;
			printf("%c", c);
		}
		printf("\n");
	}
};

void pathfinding_debug_write_found_path(pathfinding_object_t *obj, path_node_t *end_node,
										char *file_path)
{
	if (end_node == NULL) {
		printf("end_node is NULL! \n");
		return;
	}
	FILE *fd = fopen(file_path, "w+");
	uint8_t(*tab)[(int)obj->config.field_boundaries.max_x] = malloc(
		obj->config.field_boundaries.max_y * obj->config.field_boundaries.max_x * sizeof(uint8_t));
	uint8_t path_valid = 0;
	path_node_t *current_node = end_node;
	for (size_t i = 0; i < PATHFINDING_MAX_NUM_OF_NODES; i++) {
		uint16_t y = current_node->coordinate.y;
		uint16_t x = current_node->coordinate.x;
		tab[y][x] = 1;
		if (current_node->parent_node == NULL) {
			path_valid = 1;
			break;
		}
		current_node = current_node->parent_node;
	}
	if (!path_valid) {
		fprintf(fd, "Not a valid path!\n");
		fclose(fd);
		free(tab);
		return;
	}
	// printf("FB: %d %d\n", obj->config.field_boundaries.x,
	// obj->config.field_boundaries.y);
	for (size_t y = 0; y < obj->config.field_boundaries.max_y; y++) {
		for (size_t x = 0; x < obj->config.field_boundaries.max_x; x++) {
			char c = tab[y][x] ? 'X' : '.';
			fprintf(fd, "%c", c);
		}
		fprintf(fd, "\n");
	}

	fclose(fd);
	free(tab);
};

void pathfinding_debug_write_found_path_list(pathfinding_object_t *obj, path_node_t *end_node,
											 char *file_path)
{
	if (end_node == NULL) {
		printf("end_node is NULL! \n");
		return;
	}
	FILE *fd = fopen(file_path, "w+");
	uint8_t path_valid = 0;
	path_node_t *current_node = end_node;
	fprintf(fd, "[");
	for (size_t i = 0; i < PATHFINDING_MAX_NUM_OF_NODES; i++) {
		uint16_t y = current_node->coordinate.y;
		uint16_t x = current_node->coordinate.x;
		fprintf(fd, "(%d,%d)", x, y);
		if (current_node->parent_node == NULL) {
			path_valid = 1;
			break;
		}
		fprintf(fd, ",");
		current_node = current_node->parent_node;
	}

	fprintf(fd, "]");
	fclose(fd);
};
#endif
