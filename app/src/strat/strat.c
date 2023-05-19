#include <zephyr/logging/log.h>
#include "pokibrain/pokibrain.h"
#include "strat.h"
#include "strat_interface.h"
#include "pokutils.h"
#include <stdint.h>
#include "nav/nav.h"
#include "pokpush/pokpush.h"

LOG_MODULE_REGISTER(strategy);

#define CONVERT_POS2_TO_POINT2(pos)                                                                \
    (point2_t)                                                                                     \
    {                                                                                              \
        .x = (pos).x, .y = (pos).y                                                                 \
    }
#define CONVERT_POINT2_TO_POS2(point, angle)                                                       \
    (pos2_t)                                                                                       \
    {                                                                                              \
        .x = (point).x, .y = (point).y, .a = (angle)                                               \
    }

#define OFFSET_SCORE(x) (x << 16)

#define BOARD_SIZE_X   2000
#define BOARD_CENTER_X 1000
#define BOARD_SIZE_Y   3000
#define BOARD_CENTER_Y 1500
#define PLATE_SIZE     450

#define CAKE_MAX_NB_LAYERS 3
#define CAKE_LAYER_RADIUS  60
#define CAKE_LAYER_HEIGHT  20

#define NB_CHERRY_PER_DISPENSER  10
#define CHERRY_DISPENSER_WIDTH   300
#define CHERRY_DISPENSER_DEPTH   30
#define CHERRY_DISPENSER_WIDTH_2 150
#define CHERRY_DISPENSER_DEPTH_2 15

enum layer_color {
    LAYER_COLOR_NONE,
    LAYER_COLOR_YELLOW,
    LAYER_COLOR_PINK,
    LAYER_COLOR_BROWN,
};

struct cake_layer {
    enum layer_color color;
    point2_t point;
    uint8_t in_plate;
};

const static struct cake_layer layer_list[] = {
    (struct cake_layer){.color = LAYER_COLOR_BROWN,
                        .point = {.x = BOARD_CENTER_X - 275, .y = BOARD_CENTER_Y - 375}},
    (struct cake_layer){.color = LAYER_COLOR_BROWN,
                        .point = {.x = BOARD_CENTER_X + 275, .y = BOARD_CENTER_Y - 375}},
    (struct cake_layer){.color = LAYER_COLOR_BROWN,
                        .point = {.x = BOARD_CENTER_X - 275, .y = BOARD_CENTER_Y + 375}},
    (struct cake_layer){.color = LAYER_COLOR_BROWN,
                        .point = {.x = BOARD_CENTER_X + 275, .y = BOARD_CENTER_Y + 375}},
    (struct cake_layer){.color = LAYER_COLOR_PINK, .point = {.x = 225, .y = 575}},
    (struct cake_layer){.color = LAYER_COLOR_PINK, .point = {.x = BOARD_SIZE_X - 225, .y = 575}},
    (struct cake_layer){.color = LAYER_COLOR_PINK, .point = {.x = 225, .y = BOARD_SIZE_Y - 575}},
    (struct cake_layer){.color = LAYER_COLOR_PINK,
                        .point = {.x = BOARD_SIZE_X - 225, .y = BOARD_SIZE_Y - 575}},
    (struct cake_layer){.color = LAYER_COLOR_YELLOW, .point = {.x = 225, .y = 775}},
    (struct cake_layer){.color = LAYER_COLOR_YELLOW, .point = {.x = BOARD_SIZE_X - 225, .y = 775}},
    (struct cake_layer){.color = LAYER_COLOR_YELLOW, .point = {.x = 225, .y = BOARD_SIZE_Y - 775}},
    (struct cake_layer){.color = LAYER_COLOR_YELLOW,
                        .point = {.x = BOARD_SIZE_X - 225, .y = BOARD_SIZE_Y - 775}},

};

enum plate_color {
    PLATE_COLOR_NONE,
    PLATE_COLOR_BLUE,
    PLATE_COLOR_GREEN,
};

struct plate {
    enum plate_color color;
    point2_t point;
    uint8_t cake_size;
};

const static struct plate plate_list[] = {
    // GREEN
    (struct plate){.color = PLATE_COLOR_GREEN, .point = {.x = 225, .y = 225}},
    (struct plate){.color = PLATE_COLOR_GREEN, .point = {.x = BOARD_CENTER_X + 275, .y = 225}},
    (struct plate){.color = PLATE_COLOR_GREEN,
                   .point = {.x = BOARD_SIZE_X - 225, .y = BOARD_CENTER_Y - 375}},
    (struct plate){.color = PLATE_COLOR_GREEN, .point = {.x = 225, .y = BOARD_CENTER_Y + 375}},
    (struct plate){.color = PLATE_COLOR_GREEN,
                   .point = {.x = BOARD_SIZE_X - 225, .y = BOARD_SIZE_Y - 225}},
    // BLUE
    (struct plate){.color = PLATE_COLOR_BLUE, .point = {.x = BOARD_CENTER_X - 275, .y = 225}},
    (struct plate){.color = PLATE_COLOR_BLUE, .point = {.x = BOARD_SIZE_X - 225, .y = 225}},
    (struct plate){.color = PLATE_COLOR_BLUE, .point = {.x = 225, .y = BOARD_CENTER_Y - 375}},
    (struct plate){.color = PLATE_COLOR_BLUE,
                   .point = {.x = BOARD_SIZE_X - 225, .y = BOARD_CENTER_Y + 375}},
    (struct plate){.color = PLATE_COLOR_BLUE, .point = {.x = 225, .y = BOARD_SIZE_Y - 225}},
};

#define ID_END_PLATE_GREEN 4
#define ID_END_PLATE_BLUE  9

struct cherry_dispenser {
    pos2_t pos;
};

const struct cherry_dispenser dispenser_list[] = {
    (struct cherry_dispenser){.pos = {.x = BOARD_CENTER_X, .y = CHERRY_DISPENSER_WIDTH_2, .a = 0}},
    (struct cherry_dispenser){
        .pos = {.x = BOARD_CENTER_X, .y = BOARD_SIZE_Y - CHERRY_DISPENSER_WIDTH_2, .a = 0}},
    (struct cherry_dispenser){.pos = {.x = CHERRY_DISPENSER_DEPTH_2, .y = BOARD_CENTER_Y, .a = 0}},
    (struct cherry_dispenser){
        .pos = {.x = BOARD_SIZE_X - CHERRY_DISPENSER_DEPTH_2, .y = BOARD_CENTER_Y, .a = 0}},
};

struct grab_layer_precompute_storage {
    uint8_t layer_index;
    pos2_t dock_pos;
};

struct put_layer_precompute_storage {
    uint8_t plate_index;
    pos2_t dock_pos;
};

struct push_layer_precompute_storage {
    uint8_t plate_index;
    uint8_t layer_index;
    pos2_t push_dock_pos;
    pos2_t deposit_pos;
    uint8_t pushed_layer_index[ARRAY_SIZE(layer_list)];
    uint32_t pushed_layers;
};

struct pokibrain_user_context {
    pos2_t robot_pos;
    enum plate_color team_color;
    uint8_t nb_cake_layer_grabbed;
    uint8_t nb_cherry_grabbed;
    uint8_t index_held_layer;
    struct cake_layer layer_list[ARRAY_SIZE(layer_list)];
    struct plate plate_list[ARRAY_SIZE(plate_list)];
    struct cherry_dispenser dispenser_list[ARRAY_SIZE(dispenser_list)];
    union {
        struct grab_layer_precompute_storage grab;
        struct put_layer_precompute_storage put;
        struct push_layer_precompute_storage push;
    } precompute;
};

// ------------------------------ HELPERS ------------------------------

int get_closest_available_cake_layer_index(struct pokibrain_user_context *ctx,
                                           enum layer_color color, uint8_t *index)
{
    float best_dist = MAXFLOAT;
    int ret = -1;

    for (uint8_t i = 0; i < ARRAY_SIZE(ctx->layer_list); i++) {
        struct cake_layer *layer = &ctx->layer_list[i];
        if (layer->color != color) {
            continue;
        }
        if (layer->in_plate) {
            continue;
        }
        float dist = vec2_distance(CONVERT_POS2_TO_POINT2(ctx->robot_pos), layer->point);
        if (dist <= best_dist) {
            ret = 0;
            best_dist = dist;
            *index = i;
        }
    }
    return ret;
};

int get_closest_available_plate_index(struct pokibrain_user_context *ctx, point2_t point,
                                      uint8_t *index)
{
    float best_dist = MAXFLOAT;
    int ret = -1;

    for (uint8_t i = 0; i < ARRAY_SIZE(ctx->plate_list); i++) {
        struct plate *plate = &ctx->plate_list[i];
        if (plate->color != ctx->team_color) {
            continue;
        }
        if (plate->cake_size >= 2) {
            continue;
        }
        float dist = vec2_distance(point, plate->point);
        if (dist <= best_dist) {
            ret = 0;
            best_dist = dist;
            *index = i;
        }
    }
    return ret;
};

int get_closest_in_build_plate_index(struct pokibrain_user_context *ctx, uint8_t *index)
{
    float best_dist = MAXFLOAT;
    int ret = -1;

    for (uint8_t i = 0; i < ARRAY_SIZE(ctx->plate_list); i++) {
        struct plate *plate = &ctx->plate_list[i];
        if (plate->color != ctx->team_color) {
            continue;
        }
        if (plate->cake_size == 0) {
            continue;
        }
        float dist = vec2_distance(CONVERT_POS2_TO_POINT2(ctx->robot_pos), plate->point);
        if (dist <= best_dist) {
            ret = 0;
            best_dist = dist;
            *index = i;
        }
    }
    return ret;
};

// ---------------------------- STRAT TASKS ----------------------------

uint32_t calculate_score(struct pokibrain_user_context *ctx)
{
    uint32_t score = 0;
    for (uint8_t i = 0; i < ARRAY_SIZE(ctx->plate_list); i++) {
        struct plate *plate = &ctx->plate_list[i];
        if (plate->color != ctx->team_color) {
            continue;
        }
        if (plate->cake_size == 0) {
            continue;
        }
        score += MIN(plate->cake_size, 3);
    }
    return score;
};

#define PUSH_TOOL_OFFSET (M_PI + M_PI / 6)
#define DOCKING_DIST     (ROBOT_RADIUS + CAKE_LAYER_RADIUS + 20)

int get_home_docking_pos(point2_t robot_point, point2_t layer_point, pos2_t *dock_pos)
{
    float segment_len = vec2_distance(robot_point, layer_point);
    float frac = ROBOT_RADIUS * 2 / segment_len;
    vec2_t diff = point2_diff(layer_point, robot_point);

    dock_pos->x = layer_point.x - diff.dx * frac;
    dock_pos->y = layer_point.y - diff.dy * frac;
    dock_pos->a = angle_modulo(atan2f(diff.dy, diff.dx) + PUSH_TOOL_OFFSET);
    return 0;
}

int get_layer_docking_pos(point2_t robot_point, point2_t layer_point, pos2_t *dock_pos)
{
    float segment_len = vec2_distance(robot_point, layer_point);
    float frac = DOCKING_DIST / segment_len;
    vec2_t diff = point2_diff(layer_point, robot_point);

    dock_pos->x = layer_point.x - diff.dx * frac;
    dock_pos->y = layer_point.y - diff.dy * frac;
    dock_pos->a = angle_modulo(atan2f(diff.dy, diff.dx) + PUSH_TOOL_OFFSET);
    return 0;
}

int get_aligned_plate_layer_docking_pos(point2_t plate_point, point2_t layer_point,
                                        pos2_t *dock_pos)
{
    vec2_t diff = vec2_normalize(point2_diff(plate_point, layer_point));

    dock_pos->x = layer_point.x - diff.dx * DOCKING_DIST;
    dock_pos->y = layer_point.y - diff.dy * DOCKING_DIST;
    dock_pos->a = angle_modulo(atan2f(diff.dy, diff.dx) + PUSH_TOOL_OFFSET);
    LOG_DBG("plate x,y %f,%f | layer x,y %f,%f | dock_pos x,y %f,%f", plate_point.x, plate_point.y,
            layer_point.x, layer_point.y, dock_pos->x, dock_pos->y);
    if (dock_pos->x - ROBOT_RADIUS < 0 || dock_pos->y - ROBOT_RADIUS < 0 ||
        dock_pos->x + ROBOT_RADIUS > BOARD_SIZE_X || dock_pos->y + ROBOT_RADIUS > BOARD_SIZE_Y) {
        return -1;
    }

    return 0;
}

void add_layer_to_plate(struct plate *target_plate, struct cake_layer *layer)
{
    target_plate->cake_size += 1;
    layer->in_plate = true;
    layer->point = target_plate->point;
}

int get_static_components_as_obstacle(struct pokibrain_user_context *ctx, obstacle_t *obstacle_list)
{
    for (size_t i = 0; i < ARRAY_SIZE(layer_list); i++) {
        obstacle_list[i] = (obstacle_t){.type = obstacle_type_circle,
                                        .data.circle.coordinates = ctx->layer_list[i].point,
                                        .data.circle.radius = CAKE_LAYER_RADIUS};
    }

    for (int index_dispenser = 0; index_dispenser < ARRAY_SIZE(dispenser_list); index_dispenser++) {
        obstacle_list[ARRAY_SIZE(layer_list) + index_dispenser] = (obstacle_t){
            .type = obstacle_type_rectangle,
            .data.rectangle = {.coordinates = {.x = dispenser_list[index_dispenser].pos.x,
                                               .y = dispenser_list[index_dispenser].pos.y},
                               .height = CHERRY_DISPENSER_WIDTH,
                               .width = CHERRY_DISPENSER_DEPTH}};
    }

    obstacle_list[ARRAY_SIZE(layer_list) + ARRAY_SIZE(dispenser_list)] =
        (obstacle_t){.type = obstacle_type_circle,
                     .data.circle.coordinates =
                         ctx->plate_list[ctx->team_color == PLATE_COLOR_BLUE ? ID_END_PLATE_GREEN
                                                                             : ID_END_PLATE_BLUE]
                             .point,
                     .data.circle.radius = CAKE_LAYER_RADIUS * 2};

    // for (size_t i = 0; i < ARRAY_SIZE(plate_list); i++) {
    //     if (ctx->plate_list[i].color == ctx->team_color || i == ID_END_PLATE_BLUE ||
    //         i == ID_END_PLATE_GREEN) {
    //         obstacle_list[ARRAY_SIZE(layer_list) + ARRAY_SIZE(dispenser_list) + i] =
    //             (obstacle_t){.type = obstacle_type_none};
    //     } else {

    //     }
    // }

    return ARRAY_SIZE(layer_list) + ARRAY_SIZE(dispenser_list) + 1;
}

// GO HOME

int pokibrain_task_go_home(struct pokibrain_callback_params *params)
{
    LOG_INF("RUNNING %s", __func__);
    struct pokibrain_user_context *ctx = params->world_context;

    point2_t end_point = ctx->team_color == PLATE_COLOR_BLUE ? plate_list[ID_END_PLATE_BLUE].point
                                                             : plate_list[ID_END_PLATE_GREEN].point;
    pos2_t docking_pos;
    get_home_docking_pos((point2_t){.x = BOARD_CENTER_X, .y = BOARD_CENTER_Y}, end_point,
                         &docking_pos);
    obstacle_t
        obstacles[ARRAY_SIZE(layer_list) + ARRAY_SIZE(dispenser_list) + ARRAY_SIZE(plate_list)];
    int nb_obstacles = get_static_components_as_obstacle(ctx, obstacles);
    nav_go_to_with_pathfinding(docking_pos, obstacles, nb_obstacles);
    k_sleep(K_SECONDS(1));
    return 0;
}

int32_t pokibrain_reward_calculation_go_home(struct pokibrain_callback_params *params)
{
    LOG_DBG("REWARD CALC %s", __func__);
    if (params->time_in_match_ms < 85 * 1000) {
        return INT32_MIN + 1;
    }
    return OFFSET_SCORE(105);
}

int pokibrain_completion_go_home(struct pokibrain_callback_params *params)
{
    LOG_DBG("COMPLETION %s", __func__);

    return 0;
}

// PUSH CAKE

int pokibrain_precompute_push_cake_layer_in_plate(struct pokibrain_callback_params *params)
{
    LOG_DBG("PRECOMPUTE %s", __func__);
    struct pokibrain_user_context *ctx = params->world_context;

    ctx->precompute.push.layer_index = params->self->id;
    if (ctx->layer_list[ctx->precompute.push.layer_index].in_plate) {
        return INT32_MIN;
    }

    if (ctx->team_color == PLATE_COLOR_BLUE) {
        if (params->self->id == 6 || params->self->id == 10) {
            return INT32_MIN;
        }
    } else {
        if (params->self->id == 7 || params->self->id == 11) {
            return INT32_MIN;
        }
    }

    if (get_closest_available_plate_index(ctx,
                                          ctx->layer_list[ctx->precompute.push.layer_index].point,
                                          &ctx->precompute.push.plate_index)) {
        return INT32_MIN;
    }

    if (get_aligned_plate_layer_docking_pos(ctx->plate_list[ctx->precompute.push.plate_index].point,
                                            ctx->layer_list[ctx->precompute.push.layer_index].point,
                                            &ctx->precompute.push.push_dock_pos)) {
        return INT32_MIN;
    }

    if (get_layer_docking_pos(ctx->layer_list[ctx->precompute.push.layer_index].point,
                              ctx->plate_list[ctx->precompute.push.plate_index].point,
                              &ctx->precompute.push.deposit_pos)) {
        return INT32_MIN;
    }

    ctx->precompute.push.pushed_layers = 0;
    for (size_t i = 0; i < ARRAY_SIZE(layer_list); i++) {
        point2_t coll_pos;
        obstacle_t layer_obstacle = {
            .type = obstacle_type_circle,
            .data.circle = {.radius = CAKE_LAYER_RADIUS, .coordinates = ctx->layer_list[i].point}};

        if (obstacle_get_point_of_collision_with_segment(
                ctx->layer_list[ctx->precompute.push.layer_index].point,
                ctx->plate_list[ctx->precompute.push.plate_index].point, &layer_obstacle, 20,
                &coll_pos) == OBSTACLE_COLLISION_DETECTED) {
            ctx->precompute.push.pushed_layer_index[ctx->precompute.push.pushed_layers] = i;
            ctx->precompute.push.pushed_layers += 1;
        }
    }

    return 0;
}

int pokibrain_task_push_cake_layer_in_plate(struct pokibrain_callback_params *params)
{
    int ret = 0;
    LOG_INF("RUNNING %s", __func__);
    struct pokibrain_user_context *ctx = params->world_context;
    // struct plate *plate = &ctx->plate_list[ctx->precompute.push.layer_index];

    obstacle_t
        obstacles[ARRAY_SIZE(layer_list) + ARRAY_SIZE(dispenser_list) + ARRAY_SIZE(plate_list)];
    int nb_obstacles = get_static_components_as_obstacle(ctx, obstacles);
    if (nav_go_to_with_pathfinding(ctx->precompute.push.push_dock_pos, obstacles, nb_obstacles)) {
        // hack
        // strat_set_target((pos2_t){.x = BOARD_CENTER_X, .y = BOARD_CENTER_Y, .a = 0});
        ctx->layer_list[ctx->precompute.push.layer_index].in_plate = true;
        ret = -1;
        goto exit;
    }

    pokpush_deploy();

    if (strat_move_robot_to(ctx->precompute.push.deposit_pos, K_SECONDS(8))) {
        ret = -1;
        goto exit;
    }

exit:
    pokpush_retract();
    return ret;
}

int32_t
pokibrain_reward_calculation_push_cake_layer_in_plate(struct pokibrain_callback_params *params)
{
    LOG_DBG("REWARD CALC %s", __func__);
    struct pokibrain_user_context *ctx = params->world_context;
    struct plate target_plate = ctx->plate_list[ctx->precompute.push.plate_index];
    for (size_t i = 0; i < ctx->precompute.push.pushed_layers; i++) {
        add_layer_to_plate(&target_plate,
                           &ctx->layer_list[ctx->precompute.push.pushed_layer_index[i]]);
    }
    // add_layer_to_plate(&target_plate, &ctx->layer_list[ctx->precompute.push.layer_index]);
    int32_t score = MIN(target_plate.cake_size, CAKE_MAX_NB_LAYERS);
    return OFFSET_SCORE(score) -
           (int)vec2_distance(CONVERT_POS2_TO_POINT2(ctx->robot_pos),
                              ctx->layer_list[ctx->precompute.push.layer_index].point);
}

int pokibrain_completion_push_cake_layer_in_plate(struct pokibrain_callback_params *params)
{
    LOG_DBG("COMPLETION %s", __func__);
    struct pokibrain_user_context *ctx = params->world_context;
    struct plate *target_plate = &ctx->plate_list[ctx->precompute.push.plate_index];
    for (size_t i = 0; i < ctx->precompute.push.pushed_layers; i++) {
        add_layer_to_plate(target_plate,
                           &ctx->layer_list[ctx->precompute.push.pushed_layer_index[i]]);
    }

    return 0;
}

/**
 * We need :
 * - grab a slice
 * - deposit of a slice in a plate
    1 pt per slice + 4 for the legendary recipe
    pink
    yellow
    brown
 * - grab cherry
 * - put cherry on the cake 3pt
 * - deposit cherry in basket
 * - go back in start area 15 pt
 * - funny action
 */

void strat_pre_think(void *world_context)
{
    struct pokibrain_user_context *ctx = world_context;
    strat_get_robot_pos(&ctx->robot_pos);

    LOG_DBG("SCORE %d", calculate_score(ctx));
}

static void strat_end_game_clbk(void *world_context)
{
    struct pokibrain_user_context *ctx = world_context;
    LOG_INF("GAME IS OVER");
    LOG_INF("SCORE %d", calculate_score(ctx));
    nav_stop();
    strat_set_robot_brake(true);
    while (1) {
        strat_force_motor_stop();
        k_sched_lock();
    }
}

const char *get_side_name(enum team_color color)
{
    switch (color) {
        case TEAM_COLOR_BLUE:
            return "BLUE";
        case TEAM_COLOR_GREEN:
            return "GREEN";
        default:
        case TEAM_COLOR_NONE:
            return "UNKNOWN";
    }
}

// -------------------------- PUBLIC FUNCTIONS ---------------------------

#define DECLARE_PUSH_TASK(_id)                                                                     \
    {                                                                                              \
        .name = "push " STRINGIFY(_id), .task_precompute = pokibrain_precompute_push_cake_layer_in_plate,                \
                  .reward_calculation = pokibrain_reward_calculation_push_cake_layer_in_plate,     \
                  .task_process = pokibrain_task_push_cake_layer_in_plate,                         \
                  .completion_callback = pokibrain_completion_push_cake_layer_in_plate,            \
                  .id = (uint32_t)_id,                                                             \
    }

void strat_init(enum team_color color)
{
    LOG_INF("Strat init with team side %s", get_side_name(color));

    static struct pokibrain_user_context world_context = {
        .nb_cherry_grabbed = 0,
        .nb_cake_layer_grabbed = 0,
    };
    world_context.team_color = (enum plate_color)color;
    pos2_t start_pos = world_context.team_color == PLATE_COLOR_BLUE
                           ? CONVERT_POINT2_TO_POS2(plate_list[8].point, M_PI_2)
                           : CONVERT_POINT2_TO_POS2(plate_list[4].point, -M_PI_2);
    strat_set_robot_pos(start_pos);
    strat_set_target(start_pos);

    memcpy(world_context.layer_list, layer_list, sizeof(layer_list));
    memcpy(world_context.plate_list, plate_list, sizeof(plate_list));
    memcpy(world_context.dispenser_list, dispenser_list, sizeof(dispenser_list));

    static struct pokibrain_task tasks[] = {
        DECLARE_PUSH_TASK(0),
        DECLARE_PUSH_TASK(1),
        DECLARE_PUSH_TASK(2),
        DECLARE_PUSH_TASK(3),
        DECLARE_PUSH_TASK(4),
        DECLARE_PUSH_TASK(5),
        DECLARE_PUSH_TASK(6),
        DECLARE_PUSH_TASK(7),
        DECLARE_PUSH_TASK(8),
        DECLARE_PUSH_TASK(9),
        DECLARE_PUSH_TASK(10),
        DECLARE_PUSH_TASK(11),
        {.name = "go home",
         .task_precompute = NULL,
         .completion_callback = pokibrain_completion_go_home,
         .task_process = pokibrain_task_go_home,
         .reward_calculation = pokibrain_reward_calculation_go_home}
        /*
{
.name = "grab brown",
.task_precompute = pokibrain_precompute_grab_cake_layer,
.reward_calculation = pokibrain_reward_calculation_grab_cake_layer,
.task_process = pokibrain_task_grab_cake_layer,
.completion_callback = pokibrain_completion_grab_cake_layer,
.id = (uint32_t)LAYER_COLOR_BROWN,
},
{
.name = "grab pink",
.task_precompute = pokibrain_precompute_grab_cake_layer,
.reward_calculation = pokibrain_reward_calculation_grab_cake_layer,
.task_process = pokibrain_task_grab_cake_layer,
.completion_callback = pokibrain_completion_grab_cake_layer,
.id = (uint32_t)LAYER_COLOR_PINK,
},
{
.name = "grab yellow",
.task_precompute = pokibrain_precompute_grab_cake_layer,
.reward_calculation = pokibrain_reward_calculation_grab_cake_layer,
.task_process = pokibrain_task_grab_cake_layer,
.completion_callback = pokibrain_completion_grab_cake_layer,
.id = (uint32_t)LAYER_COLOR_YELLOW,
},
{.name = "put empty plate",
.task_precompute = pokibrain_precompute_put_cake_layer_in_plate,
.reward_calculation = pokibrain_reward_calculation_put_cake_layer_in_plate,
.task_process = pokibrain_task_put_cake_layer_in_plate,
.completion_callback = pokibrain_completion_put_cake_layer_in_plate,
.id = 0},
{.name = "put in build plate",
.task_precompute = pokibrain_precompute_put_cake_layer_in_plate,
.reward_calculation = pokibrain_reward_calculation_put_cake_layer_in_plate,
.task_process = pokibrain_task_put_cake_layer_in_plate,
.completion_callback = pokibrain_completion_put_cake_layer_in_plate,
.id = 1}*/
    };
    pokibrain_init(tasks, sizeof(tasks) / sizeof(tasks[0]), &world_context,
                   sizeof(struct pokibrain_user_context), strat_pre_think, strat_end_game_clbk);

    nav_init();
}

void strat_run(void)
{
    pokibrain_start();
}
