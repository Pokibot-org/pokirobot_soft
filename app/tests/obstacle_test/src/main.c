#include "obstacles/obstacle.h"
#include "stdio.h"
#include <zephyr/ztest.h>

ZTEST_SUITE(obstacles_test, NULL, NULL, NULL, NULL, NULL);

ZTEST(obstacles_test, test_collision_circles)
{
	obstacle_t a = {.type = obstacle_type_circle,
					.data.circle = {.coordinates = {.x = 50, .y = 50}, .radius = 10}};

	obstacle_t b = {.type = obstacle_type_circle,
					.data.circle = {.coordinates = {.x = 60, .y = 50}, .radius = 10}};

	zassert_equal(OBSTACLE_COLLISION_DETECTED, obstacle_are_they_colliding(&a, &b),
				  "A and B must collide");
	b.data.circle.coordinates.x = 100;
	zassert_equal(OBSTACLE_COLLISION_NONE, obstacle_are_they_colliding(&a, &b),
				  "A and B must not collide");
}

ZTEST(obstacles_test, test_collision_rectangles)
{
	ztest_test_skip(); // Unsupported

	obstacle_t a = {
		.type = obstacle_type_rectangle,
		.data.rectangle = {.coordinates = {.x = 50, .y = 50}, .height = 20, .width = 20}};

	obstacle_t b = {
		.type = obstacle_type_rectangle,
		.data.rectangle = {.coordinates = {.x = 60, .y = 60}, .height = 20, .width = 20}};

	zassert_equal(OBSTACLE_COLLISION_DETECTED, obstacle_are_they_colliding(&a, &b),
				  "A and B must collide, 255 not supported");
	b.data.rectangle.coordinates.x = 100;
	zassert_equal(OBSTACLE_COLLISION_NONE, obstacle_are_they_colliding(&a, &b),
				  "A and B must not collide");
}

ZTEST(obstacles_test, test_collision_rectangles_and_circles)
{
	obstacle_t a = {
		.type = obstacle_type_rectangle,
		.data.rectangle = {.coordinates = {.x = 50, .y = 50}, .height = 20, .width = 20}};

	obstacle_t b = {.type = obstacle_type_circle,
					.data.circle = {.coordinates = {.x = 60, .y = 50}, .radius = 10}};

	zassert_equal(OBSTACLE_COLLISION_DETECTED, obstacle_are_they_colliding(&a, &b),
				  "A and B must collide");
	zassert_equal(OBSTACLE_COLLISION_DETECTED, obstacle_are_they_colliding(&b, &a),
				  "A and B must collide");
	b.data.circle.coordinates.x = 100;
	zassert_equal(OBSTACLE_COLLISION_NONE, obstacle_are_they_colliding(&a, &b),
				  "A and B must not collide");
	zassert_equal(OBSTACLE_COLLISION_NONE, obstacle_are_they_colliding(&b, &a),
				  "A and B must not collide");
}

ZTEST(obstacles_test, test_object_holder)
{
	obstacle_holder_t holder = {0};
	obstacle_t ob = {.type = obstacle_type_circle,
					 .data.circle = {.coordinates = {.x = 60, .y = 50}, .radius = 10}};

	zassert_equal(0, obstacle_holder_push(&holder, &ob), "Push is not valid");
	zassert_equal(0, obstacle_holder_push(&holder, &ob), "Push is not valid");
	zassert_mem_equal(&ob, &holder.obstacles[1], sizeof(obstacle_t), "Store object is not valid");
}

ZTEST(obstacles_test, test_object_collision_with_seg_and_circle)
{
	const float precision = 0.01;

	point2_t start = {
		.x = 0,
		.y = 0,
	};
	point2_t end = {
		.x = 0,
		.y = 20,
	};

	obstacle_t ob = {
		.type = obstacle_type_circle,
		.data.circle.coordinates = {.x = 0, .y = 500},
		.data.circle.radius = 10,
	};

	float robot_radius = 10;
	point2_t intersection_pt = {0};
	int rcode = obstacle_get_point_of_collision_with_segment(start, end, &ob, robot_radius,
															 &intersection_pt);
	zassert_equal(OBSTACLE_COLLISION_NONE, rcode, "Intersection not found, rcode %d", rcode);

	ob.data.circle.coordinates.x = 0;
	ob.data.circle.coordinates.y = 25;
	rcode = obstacle_get_point_of_collision_with_segment(start, end, &ob, robot_radius,
														 &intersection_pt);
	zassert_equal(OBSTACLE_COLLISION_DETECTED, rcode, "Intersection found, rcode %d", rcode);

	ob.data.circle.coordinates.x = -5;
	ob.data.circle.coordinates.y = 0;
	rcode = obstacle_get_point_of_collision_with_segment(start, end, &ob, robot_radius,
														 &intersection_pt);
	zassert_equal(OBSTACLE_COLLISION_DETECTED, rcode, "Intersection found, rcode %d", rcode);

	ob.data.circle.coordinates.x = 20;
	ob.data.circle.coordinates.y = 10;
	rcode = obstacle_get_point_of_collision_with_segment(start, end, &ob, robot_radius,
														 &intersection_pt);
	zassert_equal(OBSTACLE_COLLISION_DETECTED, rcode, "Intersection found, rcode %d", rcode);

	zassert_between_inclusive(intersection_pt.x, 0 - precision, 0 + precision, "got %f",
							  intersection_pt.x);
	zassert_between_inclusive(intersection_pt.y, 10 - precision, 10 + precision, "got %f",
							  intersection_pt.y);

	ob.data.circle.coordinates.x = 21;
	ob.data.circle.coordinates.y = 10;
	rcode = obstacle_get_point_of_collision_with_segment(start, end, &ob, robot_radius,
														 &intersection_pt);
	zassert_equal(OBSTACLE_COLLISION_NONE, rcode, "Intersection not found, rcode %d", rcode);

	ob.data.circle.coordinates.x = 0;
	ob.data.circle.coordinates.y = 10;
	ob.data.circle.radius = 5;
	robot_radius = 0;

	rcode = obstacle_get_point_of_collision_with_segment(start, end, &ob, robot_radius,
														 &intersection_pt);
	zassert_equal(OBSTACLE_COLLISION_DETECTED, rcode, "Intersection found, rcode %d", rcode);

	zassert_between_inclusive(intersection_pt.x, 0 - precision, 0 + precision, "got %f",
							  intersection_pt.x);
	zassert_between_inclusive(intersection_pt.y, 5 - precision, 5 + precision, "got %f",
							  intersection_pt.y);

	rcode = obstacle_get_point_of_collision_with_segment(end, start, &ob, robot_radius,
														 &intersection_pt);
	zassert_equal(OBSTACLE_COLLISION_DETECTED, rcode, "Intersection found, rcode %d", rcode);

	zassert_between_inclusive(intersection_pt.x, 0 - precision, 0 + precision, "got %f",
							  intersection_pt.x);
	zassert_between_inclusive(intersection_pt.y, 15 - precision, 15 + precision, "got %f",
							  intersection_pt.y);
}

ZTEST(obstacles_test, test_object_collision_with_seg_and_rectangle)
{
	point2_t start = {
		.x = 0,
		.y = 0,
	};
	point2_t end = {
		.x = 0,
		.y = 1000,
	};

	obstacle_t ob = {
		.type = obstacle_type_rectangle,
		.data.rectangle.coordinates = {.x = 0, .y = 500},
		.data.rectangle.width = 10,
		.data.rectangle.height = 10,
	};

	float robot_radius = 10;
	point2_t intersection_pt = {0};
	int rcode = obstacle_get_point_of_collision_with_segment(start, end, &ob, robot_radius,
															 &intersection_pt);

	zassert_equal(OBSTACLE_COLLISION_DETECTED, rcode, "Intersection not found");
	ob.data.rectangle.coordinates.y = end.y;

	rcode = obstacle_get_point_of_collision_with_segment(start, end, &ob, robot_radius,
														 &intersection_pt);
	zassert_equal(OBSTACLE_COLLISION_DETECTED, rcode, "Intersection not found");

	ob.data.rectangle.coordinates.x = end.x + ob.data.rectangle.width / 2 + robot_radius + 1;

	rcode = obstacle_get_point_of_collision_with_segment(start, end, &ob, robot_radius,
														 &intersection_pt);
	zassert_equal(OBSTACLE_COLLISION_NONE, rcode, "Intersection found");

	start.x = 0;
	start.y = 0;
	end.x = 100;
	end.y = 100;

	ob.data.rectangle.coordinates.x = 60;
	ob.data.rectangle.coordinates.y = 40;
	ob.data.rectangle.width = 20;
	ob.data.rectangle.height = 20;
	rcode = obstacle_get_point_of_collision_with_segment(start, end, &ob, robot_radius,
														 &intersection_pt);
	zassert_equal(rcode, OBSTACLE_COLLISION_DETECTED);
	zassert_equal(intersection_pt.x, 40, "%d", (int)intersection_pt.x);
	zassert_equal(intersection_pt.y, 40, "%d", (int)intersection_pt.y);
}

ZTEST(obstacles_test, test_get_point_collision_two_segments)
{
	point2_t a1, a2, b1, b2, out, expected_out;
	a1.x = 0;
	a1.y = 40;
	a2.x = 100;
	a2.y = 40;

	b1.x = 50;
	b1.y = -200;
	b2.x = 50;
	b2.y = 200;

	out.x = -1;
	out.y = -1;

	expected_out.x = 50;
	expected_out.y = 40;
	uint8_t rcode = check_seg_collision(a1, a2, b1, b2, &out);
	zassert_equal(OBSTACLE_COLLISION_DETECTED, rcode, "The collision was not found...");
	zassert_equal(expected_out.x, out.x, "Wrongly calculated collision point x");
	zassert_equal(expected_out.y, out.y, "Wrongly calculated collision point y");

	// second test
	a1.x = 0;
	a1.y = 0;
	a2.x = 100;
	a2.y = 100;

	b1.x = 0;
	b1.y = 100;
	b2.x = 100;
	b2.y = 0;

	out.x = -1;
	out.y = -1;

	expected_out.x = 50;
	expected_out.y = 50;
	rcode = check_seg_collision(a1, a2, b1, b2, &out);
	zassert_equal(OBSTACLE_COLLISION_DETECTED, rcode, "The collision was not found...");
	zassert_equal(expected_out.x, out.x, "Wrongly calculated collision point x");
	zassert_equal(expected_out.y, out.y, "Wrongly calculated collision point y");
}