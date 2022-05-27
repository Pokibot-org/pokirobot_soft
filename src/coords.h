#ifndef COORDS_H
#define COORDS_H


#include "utils.h"

#define SIDE_YELLOW 1
#define SIDE_PURPLE 0

#define TRANSFORM_YELLOW(_yellow_coord) ({ \
        .x = - _yellow_coord.x, \
        .y = _yellow_coord.y, \
        .a = - _yellow_coord.a})

#define TRANSFORM_EDGE (158.978f)
#define TRANSFORM_SIDE (133.923f)
#define TRANSFORM_WHEEL_EXTERNAL_FLAT (TRANSFORM_EDGE+22.0f)
#define TRANSFORM_WHEEL_EXTERNAL_PROJECTION (TRANSFORM_SIDE+38.8f)

#define COORDS_START ((pos2_t){ \
        .x = 1500.0f - TRANSFORM_EDGE, \
        .y = 1000.0f + TRANSFORM_WHEEL_EXTERNAL_PROJECTION, \
        .a = 0.5f * M_PI})

#define COORDS_CARREFOUILLE ((pos2_t){ \
        .x = 647.5f, \
        .y = 0.0f + TRANSFORM_EDGE, \
        .a = 1.0f * M_PI})

#define COORDS_VITRINE ((pos2_t){ \
        .x = 1275.0f, \
        .y = 2000.0f - TRANSFORM_WHEEL_EXTERNAL_FLAT, \
        .a = 0.0f * M_PI})

#define COORDS_VITRINE ((pos2_t){ \
        .x = 1275.0f, \
        .y = 2000.0f - TRANSFORM_WHEEL_EXTERNAL_FLAT, \
        .a = 0.0f * M_PI})

#define COORDS_STATUETTE ((pos2_t){ \
        .x = 1191.0f + TRANSFORM_WHEEL_EXTERNAL_PROJECTION, \
        .y = 251.0f - TRANSFORM_WHEEL_EXTERNAL_PROJECTION, \
        .a = 5.0f/4.0f * M_PI})

#endif // COORDS_H

