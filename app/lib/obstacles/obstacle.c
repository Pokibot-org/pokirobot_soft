#include "obstacles/obstacle.h"
#include "pokutils.h"

#include <zephyr/kernel.h>
#include <math.h>
#include <stdlib.h>

#define OBSTACLE_COLLISION_NB_MAX_SIDES 8

void obstacle_holder_clear(obstacle_holder_t *obj)
{
	obj->read_head = 0;
	obj->write_head = 0;
}

int16_t obstacle_holder_get_number_of_obstacles(obstacle_holder_t *obj)
{
	uint16_t res = 0;
	for (uint16_t i = 0; i < OBSTACLE_HOLDER_MAX_NUMBER_OF_OBSTACLE; i++) {
		if (obj->obstacles[i].type != obstacle_type_none) {
			res += 1;
		}
	}
	return res;
}

uint8_t obstacle_holder_compact(obstacle_holder_t *obj)
{
	int32_t head = -1;
	for (uint32_t i = 0; i < OBSTACLE_HOLDER_MAX_NUMBER_OF_OBSTACLE; i++) {
		if (obj->obstacles[i].type != obstacle_type_none) {
			if (head >= 0) {
				obj->obstacles[head] = obj->obstacles[i];
				obj->obstacles[i].type = obstacle_type_none;
				head++;
			}
		} else {
			if (head == -1) {
				head = i;
			}
		}
	}
	if (head >= 0) {
		obj->write_head = head;
	}
	return OBSTACLE_HOLDER_ERROR_NONE;
}

uint8_t obstacle_holder_push(obstacle_holder_t *obj, obstacle_t *obstacle)
{
	if (obj->write_head == OBSTACLE_HOLDER_MAX_NUMBER_OF_OBSTACLE) {
		return OBSTACLE_HOLDER_ERROR_TO_FULL;
	}
	obj->obstacles[obj->write_head] = *obstacle;
	obj->write_head += 1;
	return OBSTACLE_HOLDER_ERROR_NONE;
}

uint8_t obstacle_holder_get(obstacle_holder_t *obj, obstacle_t **obstacle)
{
	while (1) {
		if (obj->read_head == obj->write_head) {
			return OBSTACLE_HOLDER_ERROR_NO_OBSTACLE_FOUND;
		}
		if (obj->obstacles[obj->read_head].type != obstacle_type_none) {
			*obstacle = &obj->obstacles[obj->read_head];
			return OBSTACLE_HOLDER_ERROR_NONE;
		}
		obj->read_head += 1;
		if (obj->read_head == OBSTACLE_HOLDER_MAX_NUMBER_OF_OBSTACLE) {
			obj->read_head = 0;
		}
	}
}

uint8_t obstacle_holder_push_circular_buffer_mode(obstacle_holder_t *obj, obstacle_t *obstacle)
{
	obj->obstacles[obj->write_head] = *obstacle;
	obj->write_head += 1;
	if (obj->write_head == OBSTACLE_HOLDER_MAX_NUMBER_OF_OBSTACLE) {
		obj->write_head = 0;
	}
	return OBSTACLE_HOLDER_ERROR_NONE;
}

uint8_t obstacle_holder_delete_index(obstacle_holder_t *obj, uint16_t index)
{
	if (index >= OBSTACLE_HOLDER_MAX_NUMBER_OF_OBSTACLE) {
		return OBSTACLE_HOLDER_ERROR_INVALID_INDEX;
	}
	obj->obstacles[index].type = obstacle_type_none;
	return OBSTACLE_HOLDER_ERROR_NONE;
}

/**
 * @brief Delete the obstacle object from the obstacle_holder object.
 * THE OBSTACLE OBJECT ADRESS MUST BE IN THE obstacle_holder_t OBJECT
 */
uint8_t obstacle_holder_delete(obstacle_holder_t *obj, obstacle_t *obstacle)
{
	uint32_t index = (obstacle - obj->obstacles) / sizeof(obstacle_t);
	return obstacle_holder_delete_index(obj, index);
}

enum obstacle_collision_status are_rectangle_and_circle_colliding(const rectangle_t *rec,
																  const circle_t *cir)
{
	uint32_t circle_distance_x = fabsf((int32_t)cir->coordinates.x - (int32_t)rec->coordinates.x);
	uint32_t circle_distance_y = fabsf((int32_t)cir->coordinates.y - (int32_t)rec->coordinates.y);

	if (circle_distance_x > (rec->width / 2 + cir->radius)) {
		return OBSTACLE_COLLISION_NONE;
	}
	if (circle_distance_y > (rec->height / 2 + cir->radius)) {
		return OBSTACLE_COLLISION_NONE;
	}

	if (circle_distance_x <= (rec->width / 2)) {
		return OBSTACLE_COLLISION_DETECTED;
	}
	if (circle_distance_y <= (rec->height / 2)) {
		return OBSTACLE_COLLISION_DETECTED;
	}

	uint32_t corner_distance_sq = SQUARE((int32_t)circle_distance_x - rec->width / 2) +
								  SQUARE((int32_t)circle_distance_y - rec->height / 2);

	return (corner_distance_sq <= SQUARE(cir->radius));
}

enum obstacle_collision_status obstacle_are_they_colliding(const obstacle_t *a, const obstacle_t *b)
{
	if ((a->type == obstacle_type_none) || (b->type == obstacle_type_none)) {
		return OBSTACLE_COLLISION_ERROR;
	}
	if (a->type == b->type) {
		if (a->type == obstacle_type_circle) {
			return vec2_distance(a->data.circle.coordinates, b->data.circle.coordinates) <=
				   ((a->data.circle.radius + b->data.circle.radius));
		} else {
			return OBSTACLE_COLLISION_ERROR; // FIXME: NOT SUPPORTED
		}
	}

	if (a->type == obstacle_type_circle) {
		return are_rectangle_and_circle_colliding(&b->data.rectangle, &a->data.circle);
	} else {
		return are_rectangle_and_circle_colliding(&a->data.rectangle, &b->data.circle);
	}
	return OBSTACLE_COLLISION_ERROR;
}

enum obstacle_collision_status check_seg_collision(const point2_t a1, const point2_t a2,
												   const point2_t b1, const point2_t b2,
												   point2_t *out)
{
	vec2_t vec_a, vec_b;
	vec_a.dx = a2.x - a1.x;
	vec_a.dy = a2.y - a1.y;
	vec_b.dx = b2.x - b1.x;
	vec_b.dy = b2.y - b1.y;

	int32_t den = vec_a.dx * vec_b.dy - vec_a.dy * vec_b.dx;
	if (!den) {
		*out = a1;
		return OBSTACLE_COLLISION_NONE;
	}
	float coeff_point_sur_a =
		((float)(-a1.x * vec_b.dy + b1.x * vec_b.dy + vec_b.dx * a1.y - vec_b.dx * b1.y)) / den;
	float coeff_point_sur_b =
		((float)(vec_a.dx * a1.y - vec_a.dx * b1.y - vec_a.dy * a1.x + vec_a.dy * b1.x)) / den;

	if (coeff_point_sur_a > 0 && coeff_point_sur_a < 1 && coeff_point_sur_b > 0 &&
		coeff_point_sur_b < 1) {
		out->x = a1.x + coeff_point_sur_a * vec_a.dx;
		out->y = a1.y + coeff_point_sur_a * vec_a.dy;
		return OBSTACLE_COLLISION_DETECTED;
	}
	return OBSTACLE_COLLISION_NONE;
}

static enum obstacle_collision_status
obstacle_get_point_of_collision_with_rectangle(const point2_t start_point, const point2_t end_point,
											   const rectangle_t *rectangle, const float seg_radius,
											   point2_t *out_crd)
{
	const uint8_t sides_to_check = 4;
	const float demi_w = (rectangle->width / 2 + seg_radius);
	const float demi_h = (rectangle->height / 2 + seg_radius);
	point2_t points[] = {
		[0] =
			{
				.x = rectangle->coordinates.x - demi_w,
				.y = rectangle->coordinates.y - demi_h,
			},
		[1] =
			{
				.x = rectangle->coordinates.x + demi_w,
				.y = rectangle->coordinates.y - demi_h,
			},
		[2] =
			{
				.x = rectangle->coordinates.x + demi_w,
				.y = rectangle->coordinates.y + demi_h,
			},
		[3] =
			{
				.x = rectangle->coordinates.x - demi_w,
				.y = rectangle->coordinates.y + demi_h,
			},
	};

	point2_t out_pt_coll[sides_to_check];
	uint8_t nb_coll = 0;
	uint8_t is_colliding = check_seg_collision(start_point, end_point, points[0],
											   points[sides_to_check - 1], &out_pt_coll[nb_coll]);
	nb_coll += is_colliding;
	for (size_t i = 1; i < sides_to_check; i++) {
		is_colliding = check_seg_collision(start_point, end_point, points[i - 1], points[i],
										   &out_pt_coll[nb_coll]);
		nb_coll += is_colliding;
	}

	if (!nb_coll) {
		*out_crd = end_point;
		return OBSTACLE_COLLISION_NONE;
	}

	float closest_dist = UINT16_MAX;
	for (size_t i = 0; i < nb_coll; i++) {
		float dist = vec2_distance(out_pt_coll[i], start_point);
		if (dist <= closest_dist) {
			closest_dist = dist;
			*out_crd = out_pt_coll[i];
		}
	}

	return OBSTACLE_COLLISION_DETECTED;
}

static enum obstacle_collision_status
obstacle_get_point_of_collision_with_circle(const point2_t start, const point2_t end,
											const circle_t *circle, const float seg_radius,
											point2_t *out_crd)
{
	// Calculer le vecteur directionnel du segment de ligne
	vec2_t d = point2_diff(end, start);
	// Calculer le vecteur entre le centre du cercle et le point de départ du segment de ligne
	vec2_t f = point2_diff(start, circle->coordinates);
	// Résoudre l'équation quadratique pour trouver t
	float a = vec2_dot(d, d);
	float b = 2 * vec2_dot(f, d);
	float c = vec2_dot(f, f) - (circle->radius + seg_radius) * (circle->radius + seg_radius);
	float discriminant = b * b - 4 * a * c;
	// Si le discriminant est négatif, il n'y a pas d'intersection
	if (discriminant < 0) {
		return OBSTACLE_COLLISION_NONE;
	}
	// Sinon, il y a au moins une intersection
	discriminant = sqrt(discriminant);
	float t1 = (-b - discriminant) / (2 * a);
	float t2 = (-b + discriminant) / (2 * a);
	// Vérifier si t1 est une intersection valide
	if (t1 >= 0 && t1 <= 1) {
		// Trouver les coordonnées du point d'intersection
		out_crd->x = start.x + t1 * d.dx;
		out_crd->y = start.y + t1 * d.dy;
		return OBSTACLE_COLLISION_DETECTED;
	}
	// Vérifier si t2 est une intersection valide
	if (t2 >= 0 && t2 <= 1) {
		// Trouver les coordonnées du point d'intersection
		out_crd->x = start.x + t2 * d.dx;
		out_crd->y = start.y + t2 * d.dy;
		return OBSTACLE_COLLISION_DETECTED;
	}
	// Sinon, il n'y a pas d'intersection valide
	return OBSTACLE_COLLISION_NONE;
}

enum obstacle_collision_status
obstacle_get_point_of_collision_with_segment(const point2_t start_point, const point2_t end_point,
											 const obstacle_t *obstacle, const float seg_radius,
											 point2_t *out_crd)
{
	if (obstacle->type == obstacle_type_rectangle) {
		return obstacle_get_point_of_collision_with_rectangle(
			start_point, end_point, &obstacle->data.rectangle, seg_radius, out_crd);
	} else if (obstacle->type == obstacle_type_circle) {
		return obstacle_get_point_of_collision_with_circle(
			start_point, end_point, &obstacle->data.circle, seg_radius, out_crd);
	} else {
		return OBSTACLE_COLLISION_ERROR;
	}
}
