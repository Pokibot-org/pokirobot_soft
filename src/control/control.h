#ifndef CONTROL_H
#define CONTROL_H

#include <zephyr.h>

#include "tmc2209/tmc2209.h"
#include "uart_hdb/uart_hdb.h"
#include "utils.h"


#define CONTROL_MUTEX_TIMEOUT (K_MSEC(30))

#define CONTROL_PERIOD_MS 10.0
#define ROBOT_L 150.0f
#define WHEEL_PERIMETER 267.840f
#define MM_TO_USTEPS 95238.0f

#define PLANAR_VMAX 300.0f // 100 mm/s
#define PLANAR_FACTOR (0.007f * PLANAR_VMAX)
#define PLANAR_RAMP                                                            \
    (0.2f * PLANAR_VMAX * CONTROL_PERIOD_MS) // 1/0.2 seconds to reach vmax

#define ANGULAR_VMAX (2.0f * M_PI) // 0.1 rotation/s
#define ANGULAR_FACTOR (0.5f * ANGULAR_VMAX)
#define ANGULAR_RAMP                                                           \
    (0.2f * ANGULAR_VMAX * CONTROL_PERIOD_MS) // 1/0.2 seconds to reach vmax


typedef struct omni3 {
    float v1;
    float v2;
    float v3;
} omni3_t;

typedef struct control {
    bool start;
    bool brake;
    bool ready;
    LOCKVAR(pos2_t) pos;
    LOCKVAR(pos2_t) target;
    tmc2209_t* m1;
    tmc2209_t* m2;
    tmc2209_t* m3;
} control_t;


extern tmc2209_t train_motor_1;
extern tmc2209_t train_motor_2;
extern tmc2209_t train_motor_3;
extern control_t shared_ctrl;


int control_set_pos(control_t* dev, pos2_t pos);
int control_set_target(control_t* dev, pos2_t target);

int control_get_pos(control_t* dev, pos2_t* pos);
int control_get_target(control_t* dev, pos2_t* target);

int control_init(control_t* dev, tmc2209_t* m1, tmc2209_t* m2, tmc2209_t* m3);

vel2_t world_vel_from_delta(pos2_t delta, vel2_t prev_vel);
vel2_t local_vel_from_world(pos2_t pos, vel2_t world_vel);
vel2_t world_vel_from_local(pos2_t pos, vel2_t local_vel);
omni3_t omni_from_local_vel(vel2_t local_vel);
vel2_t local_vel_from_omni(omni3_t omni);


#endif
