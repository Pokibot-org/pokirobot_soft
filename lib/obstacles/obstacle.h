#ifndef OBSTACLE_H
#define OBSTACLE_H
#include <stdint.h>

#include "pokutils/robot_utils.h"
#include "utils.h"

// TODO: How much point we need for a 360 lidar scan?
#define OBSTACLE_HOLDER_MAX_NUMBER_OF_OBSTACLE 420
#define OBSTACLE_HOLDER_ERROR_NONE 0
#define OBSTACLE_HOLDER_ERROR_TO_FULL 1
#define OBSTACLE_HOLDER_ERROR_INVALID_INDEX 2
#define OBSTACLE_HOLDER_ERROR_NO_OBSTACLE_FOUND 2

#define OBSTACLE_COLLISION_ERROR_NONE 0
#define OBSTACLE_COLLISION_ERROR_UNSUPPORTED 255

typedef struct circle {
    point2_t coordinates;
    uint16_t radius;
} circle_t;

typedef struct rectangle {
    point2_t coordinates;
    uint16_t width;
    uint16_t height;
} rectangle_t;

typedef enum {
    obstacle_type_none,
    obstacle_type_circle,
    obstacle_type_rectangle
} Obstacle_type_t;

typedef struct obstacle {
    Obstacle_type_t type : 2;
    union {
        rectangle_t rectangle;
        circle_t circle;
    } data;
} obstacle_t;

typedef struct obstacle_holder {
    obstacle_t obstacles[OBSTACLE_HOLDER_MAX_NUMBER_OF_OBSTACLE];
    uint16_t write_head;
    uint16_t read_head;
} obstacle_holder_t;

uint8_t obstacle_are_they_colliding(const obstacle_t* a, const obstacle_t* b);

void obstacle_holder_clear(obstacle_holder_t* obj);
int16_t obstacle_holder_get_number_of_obstacles(obstacle_holder_t* obj);
uint8_t obstacle_holder_push(obstacle_holder_t* obj, obstacle_t* obstacle);
uint8_t obstacle_holder_get(obstacle_holder_t* obj, obstacle_t** obstacle);
uint8_t obstacle_holder_push_circular_buffer_mode(
    obstacle_holder_t* obj, obstacle_t* obstacle);
uint8_t obstacle_holder_delete_index(obstacle_holder_t* obj, uint16_t index);
uint8_t obstacle_holder_delete(obstacle_holder_t* obj, obstacle_t* obstacle);
uint8_t obstacle_get_point_of_collision_with_segment(
    const point2_t* start_point, const point2_t* end_point,
    const obstacle_t* obstacle, const float seg_radius, point2_t* out_crd);
#ifdef UNIT_TEST
uint8_t obstacle_holder_compact(obstacle_holder_t* obj);
uint8_t check_seg_collision(const point2_t* a1, const point2_t* a2,
    const point2_t* b1, const point2_t* b2, point2_t* out);
#endif

#endif
