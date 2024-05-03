#include "path_manager.h"

#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>
#include "obstacle_manager.h"
#include "pathfinding/pathfinding.h"
#include "string.h"

LOG_MODULE_REGISTER(path_manager, 4);

typedef struct path_manager_object {
    point2_t start;
    point2_t end;
    path_manager_config_t conf;
    pathfinding_object_t pathfinding_obj;
    obstacle_holder_t obstacle_hold;
} path_manager_object_t;

// PRIVATE VAR
K_THREAD_STACK_DEFINE(path_manager_stack_area, CONFIG_PATH_MANAGER_THREAD_STACK);
struct k_thread path_manager_thread_data;
k_tid_t path_manager_tid = {0};
static path_manager_object_t pm_obj = {0};

// TO REMOVE ---------------------------------------------------------------

#define DEBUG_TAB_SIZE_Y 40
#define DEBUG_TAB_SIZE_X 80
void pbd(pathfinding_object_t *obj)
{
    char tab[DEBUG_TAB_SIZE_Y][DEBUG_TAB_SIZE_X + 1] = {0};

    // printf("FB: %d %d\n", obj->config.field_boundaries.x,
    // obj->config.field_boundaries.y);
    for (size_t y = 0; y < DEBUG_TAB_SIZE_Y; y++) {
        for (size_t x = 0; x < DEBUG_TAB_SIZE_X; x++) {
            tab[y][x] = '.';
        }
        tab[y][DEBUG_TAB_SIZE_X] = '\0';
    }

    for (size_t i = 0; i < PATHFINDING_MAX_NUM_OF_NODES; i++) {
        if (obj->nodes[i].is_used) {
            uint16_t y =
                obj->nodes[i].coordinate.y * DEBUG_TAB_SIZE_Y / obj->config.field_boundaries.max_y;
            uint16_t x =
                obj->nodes[i].coordinate.x * DEBUG_TAB_SIZE_X / obj->config.field_boundaries.max_x;
            tab[y][x] = 'X';
        }
    }

    for (size_t y = 0; y < DEBUG_TAB_SIZE_Y; y++) {
        LOG_DBG("%s", tab[y]);
    }
};

// TO REMOVE ---------------------------------------------------------------

// PRIVATE FUN
static void path_manager_task(void *p0, void *p1, void *p2)
{

    path_manager_object_t *pm_obj = (path_manager_object_t *)p0;
    path_node_t *pn_end = NULL;

    LOG_INF("path_manager_task start");
    int err = pathfinding_find_path(&pm_obj->pathfinding_obj, &pm_obj->obstacle_hold, pm_obj->start,
                                    pm_obj->end, &pn_end);
    if (err) {
        LOG_WRN("Path not found err %d", err);
    } else {
        LOG_INF("Path found");
        if (pm_obj->conf.nb_node_optimisation != 0) {
            err = pathfinding_optimize_path(&pm_obj->pathfinding_obj, &pm_obj->obstacle_hold,
                                            pn_end, pm_obj->conf.nb_node_optimisation);
            if (err) {
                LOG_WRN("Err in path optimize %d", err);
            }
        }
    }

    if (pm_obj->conf.found_path_clbk != NULL) {
        // LOG_INF("Calling found_path_clbk %d", pm_obj->conf.found_path_clbk);
        pm_obj->conf.found_path_clbk(pn_end, &pm_obj->conf.user_config);
    } else {
        LOG_ERR("found_path_clbk is null");
    }
}

// PUBLIC FUN
// #define TEST
uint8_t path_manager_find_path(point2_t start, point2_t end, obstacle_t *obstacle_list,
                               uint8_t obstacle_list_len, path_manager_config_t config)
{

    if (config.found_path_clbk == NULL) {
        return -2;
    }

    if (path_manager_tid != NULL) {
        k_thread_abort(path_manager_tid);
        path_manager_tid = NULL;
    }
    pm_obj.start = start;
    pm_obj.end = end;
    pm_obj.conf = config;

    memset(&pm_obj.pathfinding_obj, 0, sizeof(pm_obj.pathfinding_obj));

#ifdef TEST
#warning COMMENT TEST define in path_manager
    memset(&pm_obj.obstacle_hold, 0,
           sizeof(pm_obj.obstacle_hold)); // TODO: Reactivate get snapshot obstacle
                                          // manager
    obstacle_t obs = {.type = obstacle_type_circle,
                      .data.circle = {.coordinates = {.x = 1500, .y = 10}, .radius = 0}};
    for (size_t i = 0; i < 120; i++) {
        obstacle_holder_push(&pm_obj.obstacle_hold, &obs);
        obs.data.circle.coordinates.y += 10;
    }

    obs.data.circle.coordinates.x = 2000;
    obs.data.circle.coordinates.y = 2000;
    for (size_t i = 0; i < 120; i++) {
        obstacle_holder_push(&pm_obj.obstacle_hold, &obs);
        obs.data.circle.coordinates.y -= 10;
    }
#else
    obstacle_manager_get_obstacle_snapshot(&pm_obj.obstacle_hold);
#endif

    if (obstacle_list) {
        for (size_t i = 0; i < obstacle_list_len; i++) {
            int ret = obstacle_holder_push(&pm_obj.obstacle_hold, &obstacle_list[i]);
            if (ret) {
                LOG_WRN("No space");
            }
        }
    }

    const int32_t offset = 5;
    pathfinding_configuration_t pathfinding_config;
    pathfinding_config.field_boundaries.min_x = BOARD_MIN_X + offset;
    pathfinding_config.field_boundaries.min_y = BOARD_MIN_Y + offset;
    pathfinding_config.field_boundaries.max_x = BOARD_MAX_X - offset;
    pathfinding_config.field_boundaries.max_y = BOARD_MAX_Y - offset;
    pathfinding_config.delta_distance = 500.0f; // jump of Xmm
    pathfinding_config.radius_of_security = ROBOT_MAX_RADIUS_MM + 10;
    pathfinding_object_configure(&pm_obj.pathfinding_obj, &pathfinding_config);

    path_manager_tid =
        k_thread_create(&path_manager_thread_data, path_manager_stack_area,
                        K_THREAD_STACK_SIZEOF(path_manager_stack_area), path_manager_task, &pm_obj,
                        NULL, NULL, CONFIG_PATH_MANAGER_THREAD_PRIORITY, 0, K_NO_WAIT);
    if (path_manager_tid == NULL) {
        LOG_ERR("Cant start path manager thread");
        return -1;
    }
    return 0;
}

/**
 * @brief To be used by the user when path is found
 * Permit to retrieve all the path nodes in an array
 *
 * @param array array provided by the user where the final path will be stored
 * @param array_size size of the provided array
 * @param ptr_array_start return a ptr to the beginning of the path
 * @param end_node end node found by the pathfinding
 * @return int16_t return the copied path length
 */
uint16_t path_manager_retrieve_path(point2_t *array, uint32_t array_size,
                                    point2_t **ptr_array_start, const path_node_t *end_node)
{
    const path_node_t *current_node = end_node;
    for (uint16_t i = 0; i < array_size; i++) {
        array[i] = (point2_t){.x = current_node->coordinate.x, .y = current_node->coordinate.y};
        if (current_node->parent_node == NULL) {
            if (ptr_array_start) {
                *ptr_array_start = &array[i];
            }
            return i + 1;
        }
        current_node = current_node->parent_node;
    }
    return 0;
}
