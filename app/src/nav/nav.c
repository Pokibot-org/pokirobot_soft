#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include "nav/obstacle_manager.h"
#include "nav/path_manager.h"
#include "strat/strat_interface.h"
#include "nav.h"

LOG_MODULE_REGISTER(nav);

#define MAX_PATH_SIZE 256
point2_t path[MAX_PATH_SIZE];
pos2_t path_pos[MAX_PATH_SIZE];
uint16_t path_size = 0;
K_SEM_DEFINE(path_found_sem, 0, 1);

void path_found_clbk(const path_node_t *node, void *user_data)
{
    if (node) {
        path_size = path_manager_retrieve_path(path, MAX_PATH_SIZE, NULL, node);
        if (path_size == 0) {
            LOG_ERR("Path not correctly retrived");
        }
    } else {
        path_size = 0;
        LOG_ERR("End node is NULL path not found");
    }

    k_sem_give(&path_found_sem);
}

void revert_path_and_add_angle(point2_t *path, pos2_t *path_pos, int path_len, float a)
{
    for (size_t i = 0; i < path_len; i++) {
        path_pos[i].x = path[path_len - i - 1].x;
        path_pos[i].y = path[path_len - i - 1].y;
        path_pos[i].a = a;
    }
}

int nav_go_to_with_pathfinding(pos2_t end_pos, obstacle_t *obstacle_list, uint8_t obstacle_list_len)
{
    LOG_INF("Lauching pathfinding");
    path_manager_config_t path_config;
    path_config.found_path_clbk = path_found_clbk;
    path_config.nb_node_optimisation = 50;
    path_size = 0;
    pos2_t start_pos;
    strat_get_robot_pos(&start_pos);
    point2_t start;
    start.x = start_pos.x;
    start.y = start_pos.y;

    point2_t end;
    end.x = end_pos.x;
    end.y = end_pos.y;

    uint64_t previous = k_uptime_get();
    while (k_uptime_get() - previous < 30 * 1000) {

        k_sem_take(&path_found_sem, K_NO_WAIT);
        uint8_t err =
            path_manager_find_path(start, end, obstacle_list, obstacle_list_len, path_config);
        if (err) {
            LOG_ERR("problem when launching pathfinding %u", err);
            return -1;
        }
        LOG_INF("Waiting for path to be found");
        if (k_sem_take(&path_found_sem, K_SECONDS(4))) {
            LOG_WRN("No path found, timeout");
            return -2;
        }

        if (path_size == 0) {
            LOG_WRN("No path found");
            return -2;
        }

        LOG_INF("Path found of size %u", path_size);
        revert_path_and_add_angle(path, path_pos, path_size, end_pos.a);

        strat_set_waypoints(path_pos, path_size);
        int ret = strat_wait_target(STRAT_PLANAR_TARGET_SENSITIVITY_DEFAULT,
                                    STRAT_ANGULAR_TARGET_SENSITIVITY_DEFAULT, 15000, 3000);
        switch (ret) {
            case STRAT_WAIT_OK:
                LOG_INF("Go to with pf ok");
                return 0;
            case STRAT_WAIT_TIMEOUT_BRAKE:
                LOG_WRN("Timeout with brake");
                continue;
            case STRAT_WAIT_TIMEOUT_TARGET:
                LOG_ERR("Timeout");
                return -1;
            default:
                return -255;
        }
    }

    LOG_ERR("Timeout");

    return -1;
}

void collision_callback(bool collision)
{
    static int cnt = 0;
    if (collision) {
        cnt = MIN(cnt+1, LIDAR_FILTER);
        LOG_DBG("Collision detected");
    } else {
        cnt = MAX(cnt-1, 0);
    }
    if (cnt >= LIDAR_FILTER) {
        strat_set_robot_brake(true);
    } else if (cnt <= 0) {
        strat_set_robot_brake(false);
    }
}

void nav_init(void)
{
    obstacle_manager_init(collision_callback);
}

void nav_stop(void)
{
    obstacle_manager_kill();
}
