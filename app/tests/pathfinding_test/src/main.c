#include <zephyr/ztest.h>
#include "pathfinding/pathfinding.h"
#include "obstacles/obstacle.h"
#include <stdlib.h>
#include <time.h>
#include <stdio.h>
#include <string.h>

#ifndef M_SQRT2
#define M_SQRT2 1.41421356237309504880f
#endif

#define FORCE_PRINT

#ifdef FORCE_PRINT
#define FORCE_PRINT_CONSTANT 1
#else
#define FORCE_PRINT_CONSTANT 0
#endif

// #define FORCE_RANDOM

struct pf_suite_fixture {
	pathfinding_object_t pathfinding_obj;
	FILE *psr_fd;
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

	fixture->psr_fd = fopen("./psr.txt", "a+");
#ifdef FORCE_RANDOM
	srand(time(NULL));
	uint32_t tab[4];
	for (size_t i = 0; i < 4; i++) {
		tab[i] = rand();
	}
	utils_init_rand_seed(tab);
#endif
	return fixture;
}

static void pf_suite_before(void *f)
{
	struct pf_suite_fixture *fixture = (struct pf_suite_fixture *)f;
	fixture->pathfinding_obj.next_free_node_nb = 0;
	memset(&fixture->pathfinding_obj.nodes, 0, PATHFINDING_MAX_NUM_OF_NODES * sizeof(path_node_t));
}

static void pf_suite_teardown(void *f)
{
	struct pf_suite_fixture *fixture = (struct pf_suite_fixture *)f;
	fclose(fixture->psr_fd);
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
	zassert_equal(0, get_new_valid_coordinates(&fixture->pathfinding_obj, start, end, &new));
	zassert_equal(must_be_crd.y, new.y, "Y");
	zassert_equal(must_be_crd.x, new.x, "X");

	// DIAGONAL
	end = (point2_t){1000, 1000};
	must_be_crd = (point2_t){10 + fixture->pathfinding_obj.config.delta_distance * M_SQRT2 / 2,
							 10 + fixture->pathfinding_obj.config.delta_distance * M_SQRT2 / 2};
	zassert_equal(0, get_new_valid_coordinates(&fixture->pathfinding_obj, start, end, &new));
	zassert_equal(must_be_crd.y, new.y, "Y");
	zassert_equal(must_be_crd.x, new.x, "X");

	// DIAGONAL 2
	start = (point2_t){500, 500};
	end = (point2_t){10, 10};
	must_be_crd = (point2_t){500 - fixture->pathfinding_obj.config.delta_distance * M_SQRT2 / 2,
							 500 - fixture->pathfinding_obj.config.delta_distance * M_SQRT2 / 2};
	zassert_equal(0, get_new_valid_coordinates(&fixture->pathfinding_obj, start, end, &new));
	zassert_true(new.y > must_be_crd.y * 0.99 && new.y < must_be_crd.y * 1.01,
				 "Diag 2 y"); // not perfectly precise
	zassert_true(new.x > must_be_crd.x * 0.99 && new.x < must_be_crd.x * 1.01, "Diag 2 x");
	// TO CLOSE
	start = (point2_t){10, 10};
	end = (point2_t){20, 20};
	must_be_crd = (point2_t){20, 20};
	zassert_equal(0, get_new_valid_coordinates(&fixture->pathfinding_obj, start, end, &new));
	zassert_equal(must_be_crd.y, new.y, "Y");
	zassert_equal(must_be_crd.x, new.x, "X");
}

ZTEST_F(pf_suite, test_in_free_space_path_must_be_found_simple_config)
{
	obstacle_holder_t ob_hold = {0};
	path_node_t *end_node;
	point2_t start = {
		.x = fixture->pathfinding_obj.config.field_boundaries.max_x / 2,
		.y = fixture->pathfinding_obj.config.field_boundaries.max_y / 2,
	};
	point2_t end = {
		.x = 50,
		.y = 50,
	};
	int err = pathfinding_find_path(&fixture->pathfinding_obj, &ob_hold, start, end, &end_node);
	if (FORCE_PRINT_CONSTANT || err) {
		pathfinding_debug_print(&fixture->pathfinding_obj, &ob_hold);
	}
	zassert_equal(PATHFINDING_ERROR_NONE, err);
	if (FORCE_PRINT_CONSTANT || err) {
		pathfinding_debug_print_found_path(&fixture->pathfinding_obj, &ob_hold, end_node);
		printf("Found in %d nodes!\n",
			   pathfinding_get_number_of_used_nodes(&fixture->pathfinding_obj));
	}
}

ZTEST_F(pf_suite, test_in_free_space_path_must_be_found_hard_config)
{
	obstacle_holder_t ob_hold = {0};
	path_node_t *end_node;
	point2_t start = {
		.x = 10,
		.y = 10,
	};
	point2_t end = {
		.x = fixture->pathfinding_obj.config.field_boundaries.max_x - 10,
		.y = 10,
	};
	int err = pathfinding_find_path(&fixture->pathfinding_obj, &ob_hold, start, end, &end_node);
	if (FORCE_PRINT_CONSTANT || err) {
		pathfinding_debug_print(&fixture->pathfinding_obj, &ob_hold);
	}
	zassert_equal(PATHFINDING_ERROR_NONE, err);
	if (FORCE_PRINT_CONSTANT || err) {
		pathfinding_debug_print_found_path(&fixture->pathfinding_obj, &ob_hold, end_node);
		printf("Found in %d nodes!\n Now optimizing path : \n",
			   pathfinding_get_number_of_used_nodes(&fixture->pathfinding_obj));
		pathfinding_optimize_path(&fixture->pathfinding_obj, &ob_hold, end_node,
								  PATHFINDING_MAX_NUM_OF_NODES);
		pathfinding_debug_print_found_path(&fixture->pathfinding_obj, &ob_hold, end_node);
	}
}

ZTEST_F(pf_suite, test_with_obstacle_path_must_be_found_hard_config)
{
	obstacle_holder_t ob_hold = {0};
	obstacle_t rec = {
		.type = obstacle_type_rectangle,
		.data.rectangle = {
			.coordinates = {.x = fixture->pathfinding_obj.config.field_boundaries.max_x / 2,
							.y = 500},
			.height = 1000,
			.width = 200,
		}};
	obstacle_holder_push(&ob_hold, &rec);
	path_node_t *end_node;
	point2_t start = {
		.x = 10,
		.y = 10,
	};
	point2_t end = {
		.x = fixture->pathfinding_obj.config.field_boundaries.max_x - 10,
		.y = 10,
	};
	clock_t begin_clk = clock();
	int err = pathfinding_find_path(&fixture->pathfinding_obj, &ob_hold, start, end, &end_node);
	clock_t end_clk = clock();
	if (FORCE_PRINT_CONSTANT || err) {
		pathfinding_debug_print(&fixture->pathfinding_obj, &ob_hold);
	}
	zassert_equal(PATHFINDING_ERROR_NONE, err);
	if (FORCE_PRINT_CONSTANT || err) {
		pathfinding_debug_print_found_path(&fixture->pathfinding_obj, &ob_hold, end_node);
		float time_spent = (float)(end_clk - begin_clk) / CLOCKS_PER_SEC * 1000;
		printf("Found in %d nodes! Time : %f ms | Len : %f\n Now optimizing: \n",
			   pathfinding_get_number_of_used_nodes(&fixture->pathfinding_obj), time_spent,
			   end_node->distance_to_start);
		begin_clk = clock();
		pathfinding_optimize_path(&fixture->pathfinding_obj, &ob_hold, end_node,
								  PATHFINDING_MAX_NUM_OF_NODES);
		end_clk = clock();
		pathfinding_debug_print_found_path(&fixture->pathfinding_obj, &ob_hold, end_node);

		time_spent = (float)(end_clk - begin_clk) / CLOCKS_PER_SEC * 1000;
		printf("Optimized with total of %d nodes! Time : %f ms | Len : %f\n Now optimizing: \n",
			   pathfinding_get_number_of_used_nodes(&fixture->pathfinding_obj), time_spent,
			   end_node->distance_to_start);
		// pathfinding_debug_write_found_path_list(&fixture->pathfinding_obj, end_node,
		// "/tmp/path");
	}
}

ZTEST_F(pf_suite, test_with_lidar_obstacle_path_must_be_found)
{
	obstacle_holder_t ob_hold = {0};
	obstacle_t obs = {.type = obstacle_type_circle,
					  .data.circle = {.coordinates = {.x = 1500, .y = 10}, .radius = 0}};
	for (size_t i = 0; i < 30; i++) {
		obstacle_holder_push(&ob_hold, &obs);
		obs.data.circle.coordinates.y += 20;
	}
	obs.data.circle.coordinates.x = 2000;
	obs.data.circle.coordinates.y = 2000;
	for (size_t i = 0; i < 30; i++) {
		obstacle_holder_push(&ob_hold, &obs);
		obs.data.circle.coordinates.y -= 20;
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
	clock_t begin_clk = clock();
	int err = pathfinding_find_path(&fixture->pathfinding_obj, &ob_hold, start, end, &end_node);
	clock_t end_clk = clock();
	float time_spent = (float)(end_clk - begin_clk) / CLOCKS_PER_SEC * 1000;
	if (err) {
		fprintf(fixture->psr_fd, "Path not found\n");
	} else {
		fprintf(fixture->psr_fd, "Found in %d nodes! Time : %f ms | Len : %f\n",
				pathfinding_get_number_of_used_nodes(&fixture->pathfinding_obj), time_spent,
				end_node->distance_to_start);
	}
	if (FORCE_PRINT_CONSTANT || err) {
		pathfinding_debug_print(&fixture->pathfinding_obj, &ob_hold);
	}
	zassert_equal(PATHFINDING_ERROR_NONE, err);
	if (FORCE_PRINT_CONSTANT || err) {
		pathfinding_debug_print_found_path(&fixture->pathfinding_obj, &ob_hold, end_node);
		printf("Found in %d nodes! Time : %f ms | Len : %f\n Now optimizing: \n",
			   pathfinding_get_number_of_used_nodes(&fixture->pathfinding_obj), time_spent,
			   end_node->distance_to_start);
		begin_clk = clock();
		pathfinding_optimize_path(&fixture->pathfinding_obj, &ob_hold, end_node,
								  PATHFINDING_MAX_NUM_OF_NODES);
		end_clk = clock();
		pathfinding_debug_print_found_path(&fixture->pathfinding_obj, &ob_hold, end_node);

		time_spent = (float)(end_clk - begin_clk) / CLOCKS_PER_SEC * 1000;
		printf("Optimized with total of %d nodes! Time : %f ms | Len : %f\n Now optimizing: \n",
			   pathfinding_get_number_of_used_nodes(&fixture->pathfinding_obj), time_spent,
			   end_node->distance_to_start);
	}
}
