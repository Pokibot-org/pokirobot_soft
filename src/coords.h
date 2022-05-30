#ifndef COORDS_H
#define COORDS_H


#include "utils.h"

#define SIDE_YELLOW 0
#define SIDE_PURPLE 1

#define TRANSFORM_SIDE(_side, _pos) \
    (_side == SIDE_YELLOW ? TRANSFORM_YELLOW(_pos) : _pos)

#define TRANSFORM_YELLOW(_purple_coord) ((pos2_t){ \
        .x = - _purple_coord.x, \
        .y = _purple_coord.y, \
        .a = - _purple_coord.a})

#define OFFSET_EDGE 158.978f
#define OFFSET_SIDE 133.923f
#define OFFSET_WHEEL_EXTERNAL_FLAT (OFFSET_EDGE+22.0f)
#define OFFSET_WHEEL_EXTERNAL_PROJECTION (OFFSET_SIDE+38.8f)


#define COORDS_START ((pos2_t){ \
        .x = 1500.0f - OFFSET_EDGE, \
        .y = 1000.0f + OFFSET_WHEEL_EXTERNAL_PROJECTION, \
        .a = 0.5f * M_PI})

#define COORDS_HOME ((pos2_t){ \
        .x = 1300.0f - OFFSET_EDGE, \
        .y = 1300.0f + OFFSET_WHEEL_EXTERNAL_PROJECTION, \
        .a = -0.5f * M_PI})


#define COORDS_CARREFOUILLE ((pos2_t){ \
        .x = 647.5f, \
        .y = 0.0f + OFFSET_EDGE, \
        .a = 1.0f * M_PI})

#define COORDS_CARREFOUILLE_B1 ((pos2_t){ \
        .x = 925.0f, \
        .y = 575.0f, \
        .a = 1.0f * M_PI})

#define COORDS_CARREFOUILLE_B2 ((pos2_t){ \
        .x = 925.0f, \
        .y = 325.0f, \
        .a = 1.0f * M_PI})

#define COORDS_CARREFOUILLE_B3 ((pos2_t){ \
        .x = 647.5f, \
        .y = 325.0f, \
        .a = 1.0f * M_PI})

#define COORDS_CARREFOUILLE_A1 ((pos2_t){ \
        .x = 647.5f, \
        .y = 325.0f, \
        .a = (1.0f/2.0f-(1.0f/3.0f+1.0/6.0f+1.0f/12.0f)) * M_PI})


#define COORDS_VITRINE ((pos2_t){ \
        .x = 1500.0f - 345.0f - 130.0f, \
        .y = 2000.0f - OFFSET_WHEEL_EXTERNAL_FLAT, \
        .a = -(-1.0f/3.0f-1.0f/6.0f) * M_PI})

#define COORDS_VITRINE_B1 ((pos2_t){ \
        .x = 1500.0f - 345.0f, \
        .y = 2000.0f - OFFSET_WHEEL_EXTERNAL_FLAT, \
        .a = -(-1.0f/3.0f-1.0f/6.0f) * M_PI})


#define COORDS_STATUETTE ((pos2_t){ \
        .x = 1191.0f + OFFSET_WHEEL_EXTERNAL_PROJECTION, \
        .y = 251.0f + OFFSET_WHEEL_EXTERNAL_PROJECTION, \
        .a = (1.0f/2.0f-(1.0f/3.0f+1.0/6.0f+1.0f/12.0f)) * M_PI})


#endif // COORDS_H

