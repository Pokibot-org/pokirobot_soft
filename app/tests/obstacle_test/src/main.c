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

	zassert_equal(1, obstacle_are_they_colliding(&a, &b), "A and B must collide");
	b.data.circle.coordinates.x = 100;
	zassert_equal(0, obstacle_are_they_colliding(&a, &b), "A and B must not collide");
}

ZTEST(obstacles_test, test_collision_rectangles)
{
	obstacle_t a = {
		.type = obstacle_type_rectangle,
		.data.rectangle = {.coordinates = {.x = 50, .y = 50}, .height = 20, .width = 20}};

	obstacle_t b = {
		.type = obstacle_type_rectangle,
		.data.rectangle = {.coordinates = {.x = 60, .y = 60}, .height = 20, .width = 20}};

	zassert_equal(1, obstacle_are_they_colliding(&a, &b),
				  "A and B must collide, 255 not supported");
	b.data.rectangle.coordinates.x = 100;
	zassert_equal(0, obstacle_are_they_colliding(&a, &b), "A and B must not collide");
}

ZTEST(obstacles_test, test_collision_rectangles_and_circles)
{
	obstacle_t a = {
		.type = obstacle_type_rectangle,
		.data.rectangle = {.coordinates = {.x = 50, .y = 50}, .height = 20, .width = 20}};

	obstacle_t b = {.type = obstacle_type_circle,
					.data.circle = {.coordinates = {.x = 60, .y = 50}, .radius = 10}};

	zassert_equal(1, obstacle_are_they_colliding(&a, &b), "A and B must collide");
	zassert_equal(1, obstacle_are_they_colliding(&b, &a), "A and B must collide");
	b.data.circle.coordinates.x = 100;
	zassert_equal(0, obstacle_are_they_colliding(&a, &b), "A and B must not collide");
	zassert_equal(0, obstacle_are_they_colliding(&b, &a), "A and B must not collide");
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
	point2_t start = {
		.x = 10,
		.y = 10,
	};
	point2_t end = {
		.x = 10,
		.y = 1000,
	};

	obstacle_t ob = {
		.type = obstacle_type_circle,
		.data.circle.coordinates = {.x = 10, .y = 500},
		.data.circle.radius = 10,
	};

	float robot_radius = 10;
	point2_t intersection_pt = {0};
	int rcode = obstacle_get_point_of_collision_with_segment(&start, &end, &ob, robot_radius,
															 &intersection_pt);
	zassert_equal(1, rcode, "Intersection not found");

	ob.data.circle.coordinates.x = end.x + ob.data.circle.radius + robot_radius + 1;
	// FIXME: current method is aproximate even working with
	// ob.data.circle.coordinates.x -= 1
	rcode = obstacle_get_point_of_collision_with_segment(&start, &end, &ob, robot_radius,
														 &intersection_pt);
	zassert_equal(0, rcode, "Intersection found");
}

ZTEST(obstacles_test, test_object_collision_with_seg_and_rectangle)
{
	point2_t start = {
		.x = 10,
		.y = 10,
	};
	point2_t end = {
		.x = 10,
		.y = 1000,
	};

	obstacle_t ob = {
		.type = obstacle_type_rectangle,
		.data.rectangle.coordinates = {.x = 10, .y = 500},
		.data.rectangle.width = 10,
		.data.rectangle.height = 10,
	};

	float robot_radius = 10;
	point2_t intersection_pt = {0};
	int rcode = obstacle_get_point_of_collision_with_segment(&start, &end, &ob, robot_radius,
															 &intersection_pt);
	zassert_equal(1, rcode, "Intersection not found");

	ob.data.rectangle.coordinates.x = end.x + ob.data.rectangle.width / 2 + robot_radius + 1;
	// FIXME: current method is aproximate even working with
	// ob.data.circle.coordinates.x -= 1
	rcode = obstacle_get_point_of_collision_with_segment(&start, &end, &ob, robot_radius,
														 &intersection_pt);
	zassert_equal(0, rcode, "Intersection found");
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
	uint8_t rcode = check_seg_collision(&a1, &a2, &b1, &b2, &out);
	zassert_equal(1, rcode, "The collision was not found...");
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
	rcode = check_seg_collision(&a1, &a2, &b1, &b2, &out);
	zassert_equal(1, rcode, "The collision was not found...");
	zassert_equal(expected_out.x, out.x, "Wrongly calculated collision point x");
	zassert_equal(expected_out.y, out.y, "Wrongly calculated collision point y");
}