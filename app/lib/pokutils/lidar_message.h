#ifndef LIDAR_MESSAGE_H
#define LIDAR_MESSAGE_H

#include "stdint.h"

#define NUMBER_OF_LIDAR_POINTS 8

typedef struct {
    uint16_t distance;
    uint8_t quality;
} lidar_point_t;

typedef struct {
    float start_angle;
    float end_angle;
    lidar_point_t points[NUMBER_OF_LIDAR_POINTS];
} lidar_message_t;

#endif
