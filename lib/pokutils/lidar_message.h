#ifndef LIDAR_MESSAGE_H
#define LIDAR_MESSAGE_H

#include "stdint.h"

typedef struct {
    uint16_t distance;
    uint8_t quality;
} lidar_point_t;

typedef struct {
    float start_angle;
    float end_angle;
    lidar_point_t* points;
    uint32_t number_of_points;
} lidar_message_t;

#endif
