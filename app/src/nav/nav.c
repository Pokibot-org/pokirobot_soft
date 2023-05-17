#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include "nav/obstacle_manager.h"
#include "nav/path_manager.h"
#include "strat/strat_interface.h"
#include "nav.h"

LOG_MODULE_REGISTER(nav);

#define MAX_PATH_SIZE 1000
point2_t path[MAX_PATH_SIZE];
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

int nav_go_to_with_pathfinding(pos2_t end_pos)
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

    LOG_INF("From x:%f, y:%f |To x:%f, y:%f", start.x, start.y, end.x, end.y);
    k_sem_take(&path_found_sem, K_NO_WAIT);
    uint8_t err = path_manager_find_path(start, end, path_config);
    if (err) {
        LOG_ERR("problem when launching pathfinding %u", err);
        return -1;
    }
    LOG_INF("Waiting for path to be found");
    if (k_sem_take(&path_found_sem, K_SECONDS(5))) {
        LOG_WRN("No path found");
        return -2;
    }

    LOG_INF("Path found of size %u", path_size);
    for (int16_t i = path_size - 1; i >= 0; i--) {
        pos2_t next_pos;
        next_pos.a = end_pos.a;
        next_pos.x = path[i].x;
        next_pos.y = path[i].y;
        LOG_INF("Go to point %d, x: %f, y: %f", i, next_pos.x, next_pos.y);
        strat_set_target(next_pos);
        strat_wait_target(100.0f, M_PI / 4.0f, 5000);
    }
    strat_wait_target(STRAT_PLANAR_TARGET_SENSITIVITY_DEFAULT,
                      STRAT_ANGULAR_TARGET_SENSITIVITY_DEFAULT, 5000);
    return 0;
}

void collision_callback(bool collision)
{
    if (collision) {
        LOG_DBG("Collision detected");
    }
    strat_set_robot_brake(collision);
}

void nav_init(void)
{
    obstacle_manager_init(collision_callback);
}

void nav_stop(void)
{
    obstacle_manager_kill();
}