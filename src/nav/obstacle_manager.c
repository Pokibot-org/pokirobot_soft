#include "obstacle_manager.h"
#include <math.h>
#include <zephyr.h>
#include "lidar/camsense_x1/camsense_x1.h"
#include "obstacles/relative_obstacle_storing.h"
#include "odometry/odometry.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(obstacle_manager);

// DEFINES
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#define M_PI_2 1.57079632679489661923f
#define M_PI_4 0.78539816339744830962f
#endif
#define MAX_LIDAR_MESSAGE 6

#define OBSTACLE_MANAGER_DECIMATION_FACTOR 4
// TYPES

typedef enum { om_message_lidar_data_received } om_message_type_t;

typedef struct {
    om_message_type_t type;
} obstacle_manager_message_t;

typedef struct obstacle_manager {
    obstacle_holder_t obstacles_holders[2];
    uint8_t current_obs_holder_index;
    char __aligned(4) lidar_msgq_buffer[MAX_LIDAR_MESSAGE * sizeof(lidar_message_t)];
    struct k_msgq lidar_msgq;
    obstacle_manager_collision_clbk collision_callback;
} obstacle_manager_t;

K_MSGQ_DEFINE(obstacle_manager_msgq, sizeof(obstacle_manager_message_t), 10, 1);

// PRIVATE VAR
static obstacle_manager_t obs_man_obj = {0};
K_SEM_DEFINE(obsacle_holder_lock, 1, 1);
// PRIVATE DEF
#define CAMSENSE_FACTORY_CENTER_OFFSET_DEG 16.0f
#define CAMSENSE_CENTER_OFFSET_DEG (CAMSENSE_FACTORY_CENTER_OFFSET_DEG)
// #define LIDAR_COUNTER_CLOCKWISE
#define LIDAR_DETECTION_DISTANCE_MM 250
#define LIDAR_DETECTION_ANGLE 100
// FUNC

void obstacle_manager_send_message(const obstacle_manager_message_t* msg) {
    while (k_msgq_put(&obstacle_manager_msgq, msg, K_NO_WAIT)) {
        LOG_ERR("Queue full, purging");
        k_msgq_purge(&obstacle_manager_msgq);
    }
}

uint8_t obstacle_manager_get_obstacle_snapshot(obstacle_holder_t* obj) {
    k_sem_take(&obsacle_holder_lock, K_FOREVER);
    memcpy(obj, &obs_man_obj.obstacles_holders[obs_man_obj.current_obs_holder_index], sizeof(obstacle_holder_t));
    k_sem_give(&obsacle_holder_lock);
    return 0;
}

uint8_t process_point(obstacle_manager_t* obj, distance_t point_distance, float point_angle) {
    uint8_t return_code = 0;
    obstacle_t new_obstacle = {
        .type = obstacle_type_circle,
        .data.circle.radius = 0 // FIXME: remove the magic number
    };

    pos_t actual_robot_pos = robot_get_pos();

    // LOG_INF("IN PROCEES POINT: angle: %f, distance: %d", point_angle, point_distance);

    if (((point_distance < ROBOT_MAX_RADIUS_MM) && (ABS(point_angle) > LIDAR_DETECTION_ANGLE / 2)) ||
        (point_distance < ROBOT_MIN_RADIUS_MM)) // in robot do nothing
    {
        // LOG_INF("Point in robot");
        return 0;
    }

    // if it is a near obstacle change return code
    if ((point_distance < ROBOT_MAX_RADIUS_MM + LIDAR_DETECTION_DISTANCE_MM) &&
        (ABS(point_angle) < LIDAR_DETECTION_ANGLE / 2)) {
        LOG_DBG("Obstacle detected | angle: %.3hi, distance: %.5hu", (int16_t)(point_angle), point_distance);
        return_code = 1;
    }

    float point_angle_absolute = ((point_angle * (M_PI / 180.0f)) + actual_robot_pos.a_rad);
    if (point_angle_absolute < -M_PI_2) {
        point_angle_absolute += M_PI;
    } else if (point_angle_absolute > M_PI_2) {
        point_angle_absolute -= M_PI;
    }

    new_obstacle.data.circle.coordinates.x = actual_robot_pos.x + sinf(point_angle_absolute) * point_distance;
    new_obstacle.data.circle.coordinates.y = actual_robot_pos.y + cosf(point_angle_absolute) * point_distance;
    // Uncomment the following lines if you want to use tools/lidar_point_visualiser.py
    // printk("<%hd:%hd>\n", new_obstacle.data.circle.coordinates.x, new_obstacle.data.circle.coordinates.y);

    // REMOVE THE POINTS IF THERE ARE NOT IN THE TABLE!
    if (new_obstacle.data.circle.coordinates.x < 0 || new_obstacle.data.circle.coordinates.x > 3000 ||
        new_obstacle.data.circle.coordinates.y < 0 || new_obstacle.data.circle.coordinates.y > 2000) {
        return 2;
    }
    obstacle_holder_push(&obj->obstacles_holders[obj->current_obs_holder_index], &new_obstacle);

    return return_code;
}

uint8_t process_lidar_message(obstacle_manager_t* obj, const lidar_message_t* message) {
    float step = 0.0f;
    uint8_t obstacle_detected = 0;
    static uint8_t decimation_counter;
    static float old_end_angle;
    if (message->end_angle > message->start_angle) {
        step = (message->end_angle - message->start_angle) / 8.0f;
    } else {
        step = (message->end_angle - (message->start_angle - 360.0f)) / 8.0f;
    }

    if (message->start_angle > message->end_angle || old_end_angle > message->start_angle) {
        LOG_DBG("Swapping buffer, old size: %d", obj->obstacles_holders[obj->current_obs_holder_index].write_head);
        obstacle_holder_clear(&obj->obstacles_holders[obj->current_obs_holder_index]);
        obj->current_obs_holder_index = !obj->current_obs_holder_index;
    }
    old_end_angle = message->end_angle;

    for (int i = 0; i < message->number_of_points; i++) // for each of the 8 samples
    {
        decimation_counter = (decimation_counter + 1) % OBSTACLE_MANAGER_DECIMATION_FACTOR;
        if (decimation_counter) {
            continue;
        }

        if (message->points[i].quality != 0) // Filter some noisy data
        {
#ifdef LIDAR_COUNTER_CLOCKWISE
            float point_angle = 360.0f - ((message->start_angle + step * i) + (CAMSENSE_CENTER_OFFSET_DEG + 180.0f));
#else
            float point_angle = (message->start_angle + step * i) + (CAMSENSE_CENTER_OFFSET_DEG + 180.0f);
#endif
            uint8_t err_code = process_point(&obs_man_obj, message->points[i].distance, point_angle);
            if (err_code == 1) // 0 ok, 1 in front of robot, 2 outside table
            {
                obstacle_detected = 1;
            }
        }
    }

    if (obstacle_detected) {
        if (obj->collision_callback)
        {
            obj->collision_callback();
        }
    }
    return 0;
}

void lidar_receive_data_callback(const lidar_message_t* message, void* user_data) {
    int err = k_msgq_put(&obs_man_obj.lidar_msgq, message, K_NO_WAIT);
    if (err) {
        LOG_WRN("No space in lidar buffer");
    }

    const obstacle_manager_message_t msg = {.type = om_message_lidar_data_received};
    obstacle_manager_send_message(&msg);
}

static void obstacle_manager_task() {
    int err;
    obstacle_manager_message_t msg;
    while (true) {
        k_msgq_get(&obstacle_manager_msgq, &msg, K_FOREVER);
        LOG_DBG("Recived message");
        if (msg.type == om_message_lidar_data_received) {
            LOG_DBG("Recived event");

            lidar_message_t lidar_message;
            k_sem_take(&obsacle_holder_lock, K_FOREVER);
            if (!k_msgq_get(&obs_man_obj.lidar_msgq, &lidar_message, K_NO_WAIT)) {
                err = process_lidar_message(&obs_man_obj, &lidar_message);
                if (err) {
                    LOG_ERR("obstacle_manager_task error when calling process_lidar_message %d", err);
                }
            }
            k_sem_give(&obsacle_holder_lock);
        }
    }
}

K_THREAD_DEFINE(obstacle_manager_thread, CONFIG_OBSTACLE_MANAGER_THREAD_STACK, obstacle_manager_task, NULL, NULL, NULL,
                CONFIG_OBSTACLE_MANAGER_THREAD_PRIORITY, 0, K_TICKS_FOREVER);

void obstacle_manager_init(obstacle_manager_collision_clbk fun) {
    obs_man_obj.collision_callback = fun;
    k_msgq_init(&obs_man_obj.lidar_msgq, obs_man_obj.lidar_msgq_buffer, sizeof(lidar_message_t), MAX_LIDAR_MESSAGE);
    camsense_x1_init(lidar_receive_data_callback, NULL);
    k_thread_start(obstacle_manager_thread);
    LOG_INF("Init done");
}
