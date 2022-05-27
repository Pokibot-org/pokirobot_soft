#ifndef CONTROL_H
#define CONTROL_H

#include <zephyr.h>

#include "tmc2209/tmc2209.h"
#include "uart_hdb/uart_hdb.h"
#include "utils.h"


#define CONTROL_MUTEX_TIMEOUT (K_MSEC(30))

#define CONTROL_PERIOD_MS 2.0f
#define ROBOT_L 137.6f
#define WHEEL_PERIMETER 267.840f
#define MM_TO_USTEPS 95238.0f

#define PLANAR_VMAX 300.0f // 200 mm/s
#define PLANAR_FACTOR (0.01f * PLANAR_VMAX)
#define PLANAR_RAMP                                                            \
    (1.0f * PLANAR_VMAX * CONTROL_PERIOD_MS / 1000.0f) // 1/0.2 seconds to reach vmax

#define ANGULAR_VMAX (0.5f * M_PI) // 0.5 rotation/s
#define ANGULAR_FACTOR (0.5f * ANGULAR_VMAX)
#define ANGULAR_RAMP                                                           \
    (1.0f * ANGULAR_VMAX * CONTROL_PERIOD_MS / 1000.0f) // 1/0.1 seconds to reach vmax

#define CONTROL_PLANAR_TARGET_SENSITIVITY_DEFAULT 5.0f // 5mm
#define CONTROL_ANGULAR_TARGET_SENSITIVITY_DEFAULT DEG_TO_RAD(3.0f) // 3 deg


typedef struct omni3 {
    float v1;
    float v2;
    float v3;
} omni3_t;

typedef struct control {
    bool start;
    bool start_init;
    bool brake;
    bool ready;
    bool at_target;
    float planar_target_sensivity;
    float angular_target_sensivity;
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

void control_task_wait_ready();
bool control_task_wait_target(float planar_sensivity, float angular_sensivity,
        uint32_t timeout_ms);
#define control_task_wait_target_default(_timeout_ms) \
    control_task_wait_target( \
            CONTROL_PLANAR_TARGET_SENSITIVITY_DEFAULT, \
            CONTROL_ANGULAR_TARGET_SENSITIVITY_DEFAULT, \
            _timeout_ms)

void _test_gconf();
void _test_motor_cmd();
void _test_target();
void _test_calibration();
void _test_connerie();

#endif
