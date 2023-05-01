#include <zephyr/ztest.h>
#include "pathfinding/pathfinding.h"
#include "obstacles/obstacle.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <zephyr/timing/timing.h>

#ifndef M_SQRT2
#define M_SQRT2 1.41421356237309504880f
#endif

// #define FORCE_PRINT

#ifdef FORCE_PRINT
#define FORCE_PRINT_CONSTANT 1
#else
#define FORCE_PRINT_CONSTANT 0
#endif

// #define FORCE_RANDOM

struct pf_suite_fixture {
	pathfinding_object_t pathfinding_obj;
	obstacle_holder_t ob_hold;
};

static void *pf_suite_setup(void)
{
	struct pf_suite_fixture *fixture = malloc(sizeof(struct pf_suite_fixture));
	pathfinding_configuration_t config;
	config.field_boundaries.max_x = 3000; // 3m
	config.field_boundaries.max_y = 2000; // 2m
	config.field_boundaries.min_x = 0;
	config.field_boundaries.min_y = 0;
	config.delta_distance = 400;	 // jump of Xcm
	config.radius_of_security = 150; // Basicaly the robot radius in mm
	memset(&fixture->pathfinding_obj.nodes, 0, PATHFINDING_MAX_NUM_OF_NODES * sizeof(path_node_t));
	pathfinding_object_configure(&fixture->pathfinding_obj, &config);
	memset(&fixture->ob_hold, 0, sizeof(fixture->ob_hold));
	timing_init();

	return fixture;
}

static void pf_suite_before(void *f)
{
	struct pf_suite_fixture *fixture = (struct pf_suite_fixture *)f;
	fixture->pathfinding_obj.next_free_node_nb = 0;
	memset(&fixture->pathfinding_obj.nodes, 0, PATHFINDING_MAX_NUM_OF_NODES * sizeof(path_node_t));
	memset(&fixture->ob_hold, 0, sizeof(fixture->ob_hold));

	timing_start();
}

static void pf_suite_teardown(void *f)
{
	// struct pf_suite_fixture *fixture = (struct pf_suite_fixture *)f;
	timing_stop();
	free(f);
}

ZTEST_SUITE(pf_suite, NULL, pf_suite_setup, pf_suite_before, NULL, pf_suite_teardown);

ZTEST_F(pf_suite, test_get_new_valid_coordinates)
{
	point2_t start = {
		.x = 10,
		.y = 10,
	};
	point2_t end = {
		.x = 10,
		.y = 1000,
	};
	point2_t must_be_crd = {.x = 10, .y = 10 + fixture->pathfinding_obj.config.delta_distance};
	point2_t new;
	get_new_valid_coordinates(&fixture->pathfinding_obj, start, end, &new);
	zassert_equal(must_be_crd.y, new.y, "Y");
	zassert_equal(must_be_crd.x, new.x, "X");

	// DIAGONAL
	end = (point2_t){1000, 1000};
	must_be_crd = (point2_t){10 + fixture->pathfinding_obj.config.delta_distance * M_SQRT2 / 2,
							 10 + fixture->pathfinding_obj.config.delta_distance * M_SQRT2 / 2};
	get_new_valid_coordinates(&fixture->pathfinding_obj, start, end, &new);
	zassert_equal(must_be_crd.y, new.y, "Y");
	zassert_equal(must_be_crd.x, new.x, "X");

	// DIAGONAL 2
	start = (point2_t){500, 500};
	end = (point2_t){10, 10};
	must_be_crd = (point2_t){500 - fixture->pathfinding_obj.config.delta_distance * M_SQRT2 / 2,
							 500 - fixture->pathfinding_obj.config.delta_distance * M_SQRT2 / 2};
	get_new_valid_coordinates(&fixture->pathfinding_obj, start, end, &new);
	zassert_true(new.y > must_be_crd.y * 0.99 && new.y < must_be_crd.y * 1.01,
				 "Diag 2 y"); // not perfectly precise
	zassert_true(new.x > must_be_crd.x * 0.99 && new.x < must_be_crd.x * 1.01, "Diag 2 x");
	// TO CLOSE
	start = (point2_t){10, 10};
	end = (point2_t){20, 20};
	must_be_crd = (point2_t){20, 20};
	get_new_valid_coordinates(&fixture->pathfinding_obj, start, end, &new);
	zassert_equal(must_be_crd.y, new.y, "Y");
	zassert_equal(must_be_crd.x, new.x, "X");
}

ZTEST_F(pf_suite, test_in_free_space_path_must_be_found_simple_config)
{
	path_node_t *end_node;
	point2_t start = {
		.x = fixture->pathfinding_obj.config.field_boundaries.max_x / 2,
		.y = fixture->pathfinding_obj.config.field_boundaries.max_y / 2,
	};
	point2_t end = {
		.x = 50,
		.y = 50,
	};
	int err =
		pathfinding_find_path(&fixture->pathfinding_obj, &fixture->ob_hold, start, end, &end_node);
	if (FORCE_PRINT_CONSTANT || err) {
		pathfinding_debug_print(&fixture->pathfinding_obj, &fixture->ob_hold);
	}
	zassert_equal(PATHFINDING_ERROR_NONE, err);
	if (FORCE_PRINT_CONSTANT || err) {
		pathfinding_debug_print_found_path(&fixture->pathfinding_obj, &fixture->ob_hold, end_node);
		printf("Found in %d nodes!\n",
			   pathfinding_get_number_of_used_nodes(&fixture->pathfinding_obj));
	}
}

ZTEST_F(pf_suite, test_in_free_space_path_must_be_found_hard_config)
{
	path_node_t *end_node;
	point2_t start = {
		.x = 10,
		.y = 10,
	};
	point2_t end = {
		.x = fixture->pathfinding_obj.config.field_boundaries.max_x - 10,
		.y = 10,
	};
	int err =
		pathfinding_find_path(&fixture->pathfinding_obj, &fixture->ob_hold, start, end, &end_node);
	if (FORCE_PRINT_CONSTANT || err) {
		pathfinding_debug_print(&fixture->pathfinding_obj, &fixture->ob_hold);
	}
	zassert_equal(PATHFINDING_ERROR_NONE, err);
	if (FORCE_PRINT_CONSTANT || err) {
		pathfinding_debug_print_found_path(&fixture->pathfinding_obj, &fixture->ob_hold, end_node);
		printf("Found in %d nodes!\n Now optimizing path : \n",
			   pathfinding_get_number_of_used_nodes(&fixture->pathfinding_obj));
		pathfinding_optimize_path(&fixture->pathfinding_obj, &fixture->ob_hold, end_node,
								  PATHFINDING_MAX_NUM_OF_NODES);
		pathfinding_debug_print_found_path(&fixture->pathfinding_obj, &fixture->ob_hold, end_node);
	}
}

ZTEST_F(pf_suite, test_with_obstacle_path_must_be_found_hard_config)
{
	obstacle_t rec = {
		.type = obstacle_type_rectangle,
		.data.rectangle = {
			.coordinates = {.x = fixture->pathfinding_obj.config.field_boundaries.max_x / 2,
							.y = 500},
			.height = 1000,
			.width = 200,
		}};
	obstacle_holder_push(&fixture->ob_hold, &rec);
	path_node_t *end_node;
	point2_t start = {
		.x = 10,
		.y = 10,
	};
	point2_t end = {
		.x = fixture->pathfinding_obj.config.field_boundaries.max_x - 10,
		.y = 10,
	};
	int err =
		pathfinding_find_path(&fixture->pathfinding_obj, &fixture->ob_hold, start, end, &end_node);
	if (FORCE_PRINT_CONSTANT || err) {
		pathfinding_debug_print(&fixture->pathfinding_obj, &fixture->ob_hold);
	}
	zassert_equal(PATHFINDING_ERROR_NONE, err);
	if (FORCE_PRINT_CONSTANT || err) {
		pathfinding_debug_print_found_path(&fixture->pathfinding_obj, &fixture->ob_hold, end_node);
		printf("Found in %d nodes | Len : %f\n Now optimizing: \n",
			   pathfinding_get_number_of_used_nodes(&fixture->pathfinding_obj),
			   end_node->distance_to_start);
		pathfinding_optimize_path(&fixture->pathfinding_obj, &fixture->ob_hold, end_node,
								  PATHFINDING_MAX_NUM_OF_NODES);
		pathfinding_debug_print_found_path(&fixture->pathfinding_obj, &fixture->ob_hold, end_node);

		printf("Optimized with total of %d nodes! | Len : %f\n Now optimizing: \n",
			   pathfinding_get_number_of_used_nodes(&fixture->pathfinding_obj),
			   end_node->distance_to_start);
	}
}

ZTEST_F(pf_suite, test_with_lidar_obstacle_path_must_be_found)
{
	obstacle_t obs = {.type = obstacle_type_circle,
					  .data.circle = {.coordinates = {.x = 1000, .y = 10}, .radius = 0}};
	for (size_t i = 0; i < 50; i++) {
		obstacle_holder_push(&fixture->ob_hold, &obs);
		obs.data.circle.coordinates.y += 30;
	}
	obs.data.circle.coordinates.x = 2000;
	obs.data.circle.coordinates.y = 2000;
	for (size_t i = 0; i < 50; i++) {
		obstacle_holder_push(&fixture->ob_hold, &obs);
		obs.data.circle.coordinates.y -= 35;
	}

	path_node_t *end_node;
	point2_t start = {
		.x = 40,
		.y = 40,
	};
	point2_t end = {
		.x = fixture->pathfinding_obj.config.field_boundaries.max_x - 40,
		.y = 500,
	};
	int err =
		pathfinding_find_path(&fixture->pathfinding_obj, &fixture->ob_hold, start, end, &end_node);

	if (FORCE_PRINT_CONSTANT || err) {
		pathfinding_debug_print(&fixture->pathfinding_obj, &fixture->ob_hold);
	}
	zassert_equal(PATHFINDING_ERROR_NONE, err);
	if (FORCE_PRINT_CONSTANT || err) {
		pathfinding_debug_print_found_path(&fixture->pathfinding_obj, &fixture->ob_hold, end_node);
		printf("Found in %d nodes! Len : %f\n Now optimizing: \n",
			   pathfinding_get_number_of_used_nodes(&fixture->pathfinding_obj),
			   end_node->distance_to_start);
		pathfinding_optimize_path(&fixture->pathfinding_obj, &fixture->ob_hold, end_node,
								  PATHFINDING_MAX_NUM_OF_NODES);
		pathfinding_debug_print_found_path(&fixture->pathfinding_obj, &fixture->ob_hold, end_node);

		printf("Optimized with total of %d nodes! Len : %f\n Now optimizing: \n",
			   pathfinding_get_number_of_used_nodes(&fixture->pathfinding_obj),
			   end_node->distance_to_start);
	}
}

ZTEST_F(pf_suite, benchmark)
{
	uint32_t nb_test = 50;
	obstacle_t obs = {.type = obstacle_type_circle,
					  .data.circle = {.coordinates = {.x = 1000, .y = 10}, .radius = 0}};
	for (size_t i = 0; i < 50; i++) {
		obstacle_holder_push(&fixture->ob_hold, &obs);
		obs.data.circle.coordinates.y += 30;
	}
	obs.data.circle.coordinates.x = 2000;
	obs.data.circle.coordinates.y = 2000;
	for (size_t i = 0; i < 50; i++) {
		obstacle_holder_push(&fixture->ob_hold, &obs);
		obs.data.circle.coordinates.y -= 35;
	}

	path_node_t *end_node;
	point2_t start = {
		.x = 40,
		.y = 40,
	};
	point2_t end = {
		.x = fixture->pathfinding_obj.config.field_boundaries.max_x - 40,
		.y = 500,
	};
	uint32_t nb_passed_test = 0;
	uint64_t total_time_spent = 0;
	for (size_t i = 0; i < nb_test; i++) {
		timing_t start_time = timing_counter_get();
		int err = pathfinding_find_path(&fixture->pathfinding_obj, &fixture->ob_hold, start, end,
										&end_node);
		timing_t end_time = timing_counter_get();
		uint64_t total_cycles = timing_cycles_get(&start_time, &end_time);
		uint64_t total_ns = timing_cycles_to_ns(total_cycles);
		zassert_equal(err, PATHFINDING_ERROR_NONE, "pathdinfig error %d", err);
		if (!err) {
			total_time_spent += total_ns / 1000;
			nb_passed_test += 1;
		}
	}

	printf("Average Time : %d us, passed %d/%d\n", (uint32_t)(total_time_spent / nb_passed_test),
		   nb_passed_test, nb_test);
}
