#ifndef CONTROL_H
#define CONTROL_H

#include <zephyr/kernel.h>

#include "tmc2209/tmc2209.h"
#include "uart_hdb/uart_hdb.h"
#include "pokutils.h"

#define CONTROL_MUTEX_TIMEOUT (K_MSEC(30))
#define CONTROL_WAYPOINTS_N   256

#define CONTROL_WAIT_OK             0
#define CONTROL_WAIT_TIMEOUT_TARGET (-1)
#define CONTROL_WAIT_TIMEOUT_BRAKE  (-2)

#define CONTROL_PERIOD_MS 2.0f
#define ROBOT_L           160.404f
#define WHEEL_PERIMETER   358.142f
#define MM_TO_USTEPS      102657.14f

#define PLANAR_VMAX   400.0f // 700 mm/s
#define PLANAR_FACTOR (0.06f * PLANAR_VMAX)
#define PLANAR_RAMP   (2.0f * PLANAR_VMAX * CONTROL_PERIOD_MS / 1000.0f) // 2 seconds to reach vmax

#define ANGULAR_VMAX   (0.7f * M_PI) // 0.5 rotation/s
#define ANGULAR_FACTOR (0.7f * ANGULAR_VMAX)
#define ANGULAR_RAMP   (0.5f * ANGULAR_VMAX * CONTROL_PERIOD_MS / 1000.0f) // 1 seconds to reach vmax

// normal
// #define CONTROL_PLANAR_TARGET_SENSITIVITY_DEFAULT  5.0f             // 5mm
// #define CONTROL_ANGULAR_TARGET_SENSITIVITY_DEFAULT DEG_TO_RAD(3.0f) // 3 deg
// #define WP_DIST_BIAS                               100.0f
// #define WP_SENSITIVITY                             300.0f
// drawing
#define CONTROL_PLANAR_TARGET_SENSITIVITY_DEFAULT  5.0f
#define CONTROL_ANGULAR_TARGET_SENSITIVITY_DEFAULT DEG_TO_RAD(3.0f)
#define WP_DIST_BIAS                               60.0f
#define WP_SENSITIVITY                             80.0f

typedef struct waypoints {
    pos2_t *wps;
    int n;
    int idx;
    struct k_mutex lock;
} waypoints_t;

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
    float dir_angle;
    LOCKVAR(pos2_t) pos;
    waypoints_t waypoints;
    tmc2209_t *m1;
    tmc2209_t *m2;
    tmc2209_t *m3;
} control_t;

extern tmc2209_t train_motor_1;
extern tmc2209_t train_motor_2;
extern tmc2209_t train_motor_3;
extern control_t shared_ctrl;

int control_set_pos(control_t *dev, pos2_t pos);
int control_set_brake(control_t *dev, bool brake);
int control_set_waypoints(control_t *dev, pos2_t *src, int n);
int control_get_pos(control_t *dev, pos2_t *pos);

int control_init(control_t *dev, tmc2209_t *m1, tmc2209_t *m2, tmc2209_t *m3);

void control_force_motor_stop(void);

vel2_t world_vel_from_delta(pos2_t delta, vel2_t prev_vel);
vel2_t world_vel_from_delta2(pos2_t delta1, pos2_t delta2, vel2_t prev_vel);
vel2_t local_vel_from_world(pos2_t pos, vel2_t world_vel);
vel2_t world_vel_from_local(pos2_t pos, vel2_t local_vel);
omni3_t omni_from_local_vel(vel2_t local_vel);
vel2_t local_vel_from_omni(omni3_t omni);

void control_task_wait_ready();
int control_task_wait_target(float planar_sensivity, float angular_sensivity,
                             uint32_t timeout_target_ms, uint32_t timeout_brake_ms);
#define control_task_wait_target_default(_timeout_target_ms, _timeout_brake_ms)                    \
    control_task_wait_target(CONTROL_PLANAR_TARGET_SENSITIVITY_DEFAULT,                            \
                             CONTROL_ANGULAR_TARGET_SENSITIVITY_DEFAULT, _timeout_target_ms,       \
                             _timeout_brake_ms)

void _test_gconf();
void _test_motor_cmd();
void _test_target();
void _test_calibration_distance();
void _test_calibration_angle();
void _test_calibration_mix();
void _test_connerie();
void _test_drawing();

#endif
