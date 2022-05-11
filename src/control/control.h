#ifndef CONTROL_H
#define CONTROL_H

#include <zephyr.h>

#include "utils.h"


#define CONTROL_MUTEX_TIMEOUT (K_MSEC(30))

#define ROBOT_L 150.0f
#define WHEEL_PERIMETER 267.840f


typedef struct omni3 {
    float v1;
    float v2;
    float v3;
} omni3_t;

typedef struct control {
    LOCKVAR(pos2_t) pos;
    LOCKVAR(pos2_t) target;
    LOCKVAR(omni3_t) motors_v;
} control_t;


int control_init(control_t* dev);

int control_set_pos(control_t* dev, pos2_t pos);
int control_set_target(control_t* dev, pos2_t target);
int control_set_motors_v(control_t* dev, omni3_t motors_v);

int control_get_pos(control_t* dev, pos2_t* pos);
int control_get_target(control_t* dev, pos2_t* target);
int control_get_motors_v(control_t* dev, omni3_t* motors_v);

vel2_t world_vel2_from_delta(pos2_t delta, vel2_t prev_vel);
vel2_t local_vel2_from_world(pos2_t pos, vel2_t world_vel);
omni3_t omni3_from_vel2(vel2_t vel);

#endif
