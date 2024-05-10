#include "obstacle_manager.h"

#include <math.h>
#include <zephyr/kernel.h>

#include "strat/strat_interface.h"
#include "lidar/camsense_x1/camsense_x1.h"
#include "pokutils.h"
#include <zephyr/logging/log.h>
#include "global_def.h"

LOG_MODULE_REGISTER(obstacle_manager, CONFIG_OBSTACLE_MANAGER_LOG_LEVEL);

// DEFINES
#define MAX_LIDAR_MESSAGE                  32
#define CHECK_IF_OBSTACLE_INSIDE_TABLE     0
// TYPES

typedef enum {
    OM_MESSAGE_LIDAR_DATA_RECEIVED,
    OM_MESSAGE_QUIT
} om_message_type_t;

typedef struct {
    om_message_type_t type;
} obstacle_manager_message_t;

typedef struct obstacle_manager {
    obstacle_holder_t obstacles_holders[2];
    uint8_t current_obs_holder_index;
    char __aligned(4) lidar_msgq_buffer[MAX_LIDAR_MESSAGE * sizeof(lidar_message_t)];
    struct k_msgq lidar_msgq;
    obstacle_manager_collision_clbk collision_callback;
    bool obstacle_detected;
} obstacle_manager_t;

K_MSGQ_DEFINE(obstacle_manager_msgq, sizeof(obstacle_manager_message_t), 32, 1);

// PRIVATE VAR
static obstacle_manager_t obs_man_obj = {0};
K_SEM_DEFINE(obsacle_holder_lock, 1, 1);

void collision_callback_hander(struct k_work *work);
K_WORK_DELAYABLE_DEFINE(collision_callback_work, collision_callback_hander);

// PRIVATE DEF
#define CAMSENSE_OFFSET_IN_ROBOT    (-180.0f * 3.0f / 4.0f)
#define CAMSENSE_CENTER_OFFSET_DEG  (CAMSENSE_OFFSET_IN_ROBOT)
#define LIDAR_DETECTION_DISTANCE_MM 300
// 360 == detecting obstacles even behind
// FUNC

void obstacle_manager_send_message(const obstacle_manager_message_t *msg)
{
    while (k_msgq_put(&obstacle_manager_msgq, msg, K_NO_WAIT)) {
        LOG_ERR("Queue full, purging");
        k_msgq_purge(&obstacle_manager_msgq);
    }
}

uint8_t obstacle_manager_get_obstacle_snapshot(obstacle_holder_t *obj)
{
    k_sem_take(&obsacle_holder_lock, K_FOREVER);
    memcpy(obj, &obs_man_obj.obstacles_holders[obs_man_obj.current_obs_holder_index],
           sizeof(obstacle_holder_t));
    k_sem_give(&obsacle_holder_lock);
    return 0;
}

uint8_t process_point(obstacle_manager_t *obj, uint16_t point_distance, float point_angle)
{
    uint8_t return_code = 0;
    // obstacle_t new_obstacle = {
    //     .type = obstacle_type_circle,
    //     .data.circle.radius = 0 // FIXME: remove the magic number
    // };
    // pos2_t actual_robot_pos;
    // strat_get_robot_pos(&actual_robot_pos);
    float robot_dir = strat_get_robot_dir_angle();

    // LOG_INF("IN PROCESS POINT: angle: %f, distance: %d", point_angle, point_distance);

    if (point_distance < ROBOT_MIN_RADIUS_MM) // in robot do nothing
    {
        // LOG_INF("Point in robot");
        return 0;
    }

    // Calculate point in robot frame of reference
    // START OLD
    // float point_angle_rad = point_angle * (M_PI / 180.0f);
    // float point_x_local = cosf(point_angle_rad) * point_distance;
    // float point_y_local = sinf(point_angle_rad) * point_distance;

    // float point_x_robot =
    //     point_x_local * cosf(actual_robot_pos.a) - point_y_local * sinf(actual_robot_pos.a);
    // float point_y_robot =
    //     point_x_local * sinf(actual_robot_pos.a) + point_y_local * cosf(actual_robot_pos.a);

    // float point_x_world = actual_robot_pos.x + point_x_robot;
    // float point_y_world = actual_robot_pos.y + point_y_robot;

    // // Calculate point in table frame of reference
    // new_obstacle.data.circle.coordinates.x = point_x_world;
    // new_obstacle.data.circle.coordinates.y = -point_y_world;

    // float flip_angle = 2.0f * M_PI - point_angle_rad - 1.25f;
    // float delta_angle = fabsf(angle_modulo(flip_angle - robot_dir));

    // END OLD
    float point_angle_rad = DEG_TO_RAD(point_angle);
    float point_a_lidar = 2.0f * M_PI - point_angle_rad;
    float point_a_robot = fmodf(point_a_lidar + DEG_TO_RAD(60.0f) + DEG_TO_RAD(240.0f), 2 * M_PI);

    // float point_x_robot = cosf(point_a_robot) * point_distance;
    // float point_y_robot = sinf(point_a_robot) * point_distance;
    float delta_angle = fabsf(angle_modulo(point_a_robot - robot_dir));


    // if it is a near obstacle change return code
    if ((point_distance < (ROBOT_MAX_RADIUS_MM + LIDAR_DETECTION_DISTANCE_MM)) &&
        (delta_angle < M_PI / 3)) {
        LOG_DBG("obstacle on path: robot_dir=: %.1f, obstable_angle: %.1f, delta: %.1f",
                (double)RAD_TO_DEG(robot_dir), (double)RAD_TO_DEG(point_a_robot), (double)RAD_TO_DEG(delta_angle));
        return_code = 1;
    }

#if CHECK_IF_OBSTACLE_INSIDE_TABLE
    // REMOVE THE POINTS IF THERE ARE NOT IN THE TABLE!
    if (new_obstacle.data.circle.coordinates.x < BOARD_MIN_X ||
        new_obstacle.data.circle.coordinates.x > BOARD_MAX_X ||
        new_obstacle.data.circle.coordinates.y < BOARD_MIN_Y ||
        new_obstacle.data.circle.coordinates.y > BOARD_MAX_Y) {
        return 2;
    }
#endif
    // Uncomment the following lines if you want to use
    // tools/lidar_point_visualiser.py
    // printk("<%0.2f:%0.2f>\n", new_obstacle.data.circle.coordinates.x,
    // new_obstacle.data.circle.coordinates.y);
    // LOG_INF("Local %f %f %f", actual_robot_pos.x,actual_robot_pos.y,actual_robot_pos.a);
    // obstacle_holder_push(&obj->obstacles_holders[obj->current_obs_holder_index], &new_obstacle);

    return return_code;
}

void collision_callback_hander(struct k_work *work)
{
    obs_man_obj.obstacle_detected = false;
    if (obs_man_obj.collision_callback) {
        obs_man_obj.collision_callback(obs_man_obj.obstacle_detected);
    }
}

uint8_t process_lidar_message(obstacle_manager_t *obj, const lidar_message_t *message)
{
    float step = 0.0f;
    static float old_end_angle;
    static uint8_t filt_buff = 0;

    if (message->end_angle > message->start_angle) {
        step = (message->end_angle - message->start_angle) / 8.0f;
    } else {
        step = (message->end_angle - (message->start_angle - 360.0f)) / 8.0f;
    }

    if (message->start_angle > message->end_angle || old_end_angle > message->start_angle) {
        LOG_DBG("Swapping buffer, old size: %d",
                obj->obstacles_holders[obj->current_obs_holder_index].write_head);
        obstacle_holder_clear(&obj->obstacles_holders[obj->current_obs_holder_index]);
        obj->current_obs_holder_index = !obj->current_obs_holder_index;
    }
    old_end_angle = message->end_angle;

    for (int i = 0; i < NUMBER_OF_LIDAR_POINTS; i++) {
        // Filter some noisy data
        filt_buff = filt_buff << 1;
        // THE QUALITY PARAMETER IS RANDOM
        if (message->points[i].quality <= 30) {
            continue;
        }

        float point_angle = (message->start_angle + step * i) + CAMSENSE_CENTER_OFFSET_DEG + 180.0f;
        uint8_t err_code = process_point(&obs_man_obj, message->points[i].distance, point_angle);

        if (err_code == 1) // 0 ok, 1 in front of robot, 2 outside table
        {
            filt_buff |= 1;
        }


        uint8_t count_detections = 0;
        const uint8_t nb_detection_treshhold = 3;
        for (size_t i = 0; i < (sizeof(filt_buff) * 8); i++)
        {
            count_detections += (filt_buff >> i) & 0x1;
        }
        
        if (count_detections >= nb_detection_treshhold) {
            k_work_reschedule(&collision_callback_work, K_MSEC(600));
            if (!obj->obstacle_detected) {
                obj->obstacle_detected = true;
                if (obj->collision_callback) {
                    obj->collision_callback(obj->obstacle_detected);
                }
            }
        }
    }
    return 0;
}

void lidar_receive_data_callback(const lidar_message_t *message, void *user_data)
{
    int err = k_msgq_put(&obs_man_obj.lidar_msgq, message, K_NO_WAIT);
    if (err) {
        LOG_WRN("No space in lidar buffer");
    }

    const obstacle_manager_message_t msg = {.type = OM_MESSAGE_LIDAR_DATA_RECEIVED};
    obstacle_manager_send_message(&msg);
}

static int obstacle_manager_task(void)
{
    int err;
    obstacle_manager_message_t msg;
    while (true) {
        k_msgq_get(&obstacle_manager_msgq, &msg, K_FOREVER);
        LOG_DBG("Received message");
        if (msg.type == OM_MESSAGE_LIDAR_DATA_RECEIVED) {
            LOG_DBG("Received event");

            lidar_message_t lidar_message;
            k_sem_take(&obsacle_holder_lock, K_FOREVER);
            while (!k_msgq_get(&obs_man_obj.lidar_msgq, &lidar_message, K_NO_WAIT)) {
                err = process_lidar_message(&obs_man_obj, &lidar_message);
                if (err) {
                    LOG_ERR("obstacle_manager_task error when calling "
                            "process_lidar_message %d",
                            err);
                }
            }
            k_sem_give(&obsacle_holder_lock);
        } else if (msg.type == OM_MESSAGE_QUIT) {
            return 0;
        }
    }
}

K_THREAD_DEFINE(obstacle_manager_thread, CONFIG_OBSTACLE_MANAGER_THREAD_STACK,
                obstacle_manager_task, NULL, NULL, NULL, CONFIG_OBSTACLE_MANAGER_THREAD_PRIORITY, 0,
                K_TICKS_FOREVER);

void obstacle_manager_init(obstacle_manager_collision_clbk fun)
{
    obs_man_obj.collision_callback = fun;
    k_msgq_init(&obs_man_obj.lidar_msgq, obs_man_obj.lidar_msgq_buffer, sizeof(lidar_message_t),
                MAX_LIDAR_MESSAGE);
    camsense_x1_init(lidar_receive_data_callback, NULL);
    k_thread_start(obstacle_manager_thread);
    LOG_INF("Init done");
}

void obstacle_manager_kill(void)
{
    camsense_x1_kill();
    const obstacle_manager_message_t msg = {.type = OM_MESSAGE_QUIT};
    k_msgq_put(&obstacle_manager_msgq, &msg, K_FOREVER);
    k_thread_join(obstacle_manager_thread, K_FOREVER);
}
