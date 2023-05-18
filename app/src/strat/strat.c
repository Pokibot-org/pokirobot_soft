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
    (struct cake_layer){.color = LAYER_COLOR_PINK, .point = {.x = 225, .y = 775}},
    (struct cake_layer){.color = LAYER_COLOR_PINK, .point = {.x = BOARD_SIZE_X - 225, .y = 775}},
    (struct cake_layer){.color = LAYER_COLOR_PINK, .point = {.x = 225, .y = BOARD_SIZE_Y - 775}},
    (struct cake_layer){.color = LAYER_COLOR_PINK,
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
    enum layer_color cake_layer_colors[CAKE_MAX_NB_LAYERS];
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

int get_closest_available_plate_index(struct pokibrain_user_context *ctx, uint8_t *index)
{
    float best_dist = MAXFLOAT;
    int ret = -1;

    for (uint8_t i = 0; i < ARRAY_SIZE(ctx->plate_list); i++) {
        struct plate *plate = &ctx->plate_list[i];
        if (plate->color != ctx->team_color) {
            continue;
        }
        if (plate->cake_size != 0) {
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

bool is_golden_recipe(const struct plate *plate)
{
    if (plate->cake_size != 3) {
        return false;
    }
    return plate->cake_layer_colors[0] == LAYER_COLOR_BROWN &&
           plate->cake_layer_colors[1] == LAYER_COLOR_YELLOW &&
           plate->cake_layer_colors[2] == LAYER_COLOR_PINK;
}

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
        score += plate->cake_size + is_golden_recipe(plate) * 4;
    }
    return score;
};

    // ---------------------------------------- GRAB CAKE

#define PUSH_TOOL_OFFSET (M_PI + M_PI / 6)

int get_layer_docking_pos(point2_t robot_point, point2_t layer_point, pos2_t *dock_pos)
{
    const float docking_dist = ROBOT_RADIUS + CAKE_LAYER_RADIUS;
    float segment_len = vec2_distance(robot_point, layer_point);
    float frac = docking_dist / segment_len;
    vec2_t diff = point2_diff(layer_point, robot_point);

    dock_pos->x = layer_point.x - diff.dx * frac;
    dock_pos->y = layer_point.y - diff.dy * frac;
    dock_pos->a = angle_modulo(atan2f(diff.dy, diff.dx) + PUSH_TOOL_OFFSET);
    return 0;
}

int get_aligned_plate_layer_docking_pos(point2_t plate_point, point2_t layer_point,
                                        pos2_t *dock_pos)
{
    const float docking_dist = ROBOT_RADIUS + CAKE_LAYER_RADIUS + 20;
    vec2_t diff = vec2_normalize(point2_diff(plate_point, layer_point));

    dock_pos->x = layer_point.x - diff.dx * docking_dist;
    dock_pos->y = layer_point.y - diff.dy * docking_dist;
    dock_pos->a = angle_modulo(atan2f(diff.dy, diff.dx) + PUSH_TOOL_OFFSET);
    LOG_DBG("plate x,y %f,%f | layer x,y %f,%f | dock_pos x,y %f,%f", plate_point.x, plate_point.y,
            layer_point.x, layer_point.y, dock_pos->x, dock_pos->y);
    if (dock_pos->x - ROBOT_RADIUS < 0 || dock_pos->y - ROBOT_RADIUS < 0 ||
        dock_pos->x + ROBOT_RADIUS > BOARD_SIZE_X || dock_pos->y + ROBOT_RADIUS > BOARD_SIZE_Y) {
        return -1;
    }

    return 0;
}

int pokibrain_precompute_grab_cake_layer(struct pokibrain_callback_params *params)
{
    LOG_DBG("PRECOMPUTE %s", __func__);
    struct pokibrain_user_context *ctx = params->world_context;
    if (ctx->nb_cake_layer_grabbed != 0) {
        return INT32_MIN;
    }
    enum layer_color color = (enum layer_color)params->self->id;
    uint8_t index = 255;
    if (get_closest_available_cake_layer_index(ctx, color, &index)) {
        return INT32_MIN;
    }

    if (get_layer_docking_pos(CONVERT_POS2_TO_POINT2(ctx->robot_pos), ctx->layer_list[index].point,
                              &ctx->precompute.grab.dock_pos)) {
        return INT32_MIN;
    }
    ctx->precompute.grab.layer_index = index;

    return 0;
}

int pokibrain_task_grab_cake_layer(struct pokibrain_callback_params *params)
{
    LOG_INF("RUNNING %s", __func__);
    struct pokibrain_user_context *ctx = params->world_context;

    pos2_t layer_pos = ctx->precompute.grab.dock_pos;
    if (nav_go_to_with_pathfinding(layer_pos, NULL, 0)) {
        return -1;
    }

    if (strat_grab_layer(layer_pos, K_SECONDS(6))) {
        return -1;
    }
    return 0;
}

int32_t pokibrain_reward_calculation_grab_cake_layer(struct pokibrain_callback_params *params)
{
    LOG_DBG("REWARD CALC %s", __func__);
    return 0;
}

int pokibrain_completion_grab_cake_layer(struct pokibrain_callback_params *params)
{
    LOG_DBG("COMPLETION %s", __func__);

    struct pokibrain_user_context *ctx = params->world_context;

    ctx->index_held_layer = ctx->precompute.grab.layer_index;
    ctx->nb_cake_layer_grabbed = 1;
    return 0;
}

// ---------------------------------------- PUT CAKE
int pokibrain_precompute_put_cake_layer_in_plate(struct pokibrain_callback_params *params)
{
    LOG_DBG("PRECOMPUTE %s", __func__);
    struct pokibrain_user_context *ctx = params->world_context;
    if (ctx->nb_cake_layer_grabbed == 0) {
        return INT32_MIN;
    }

    uint8_t index = 255;
    if (params->self->id) {
        if (get_closest_available_plate_index(ctx, &index)) {
            return INT32_MIN;
        }
    } else {
        if (get_closest_in_build_plate_index(ctx, &index)) {
            return INT32_MIN;
        }
    }

    if (ctx->plate_list[index].cake_size >= CAKE_MAX_NB_LAYERS) {
        return INT32_MIN;
    }

    if (get_layer_docking_pos(CONVERT_POS2_TO_POINT2(ctx->robot_pos), ctx->plate_list[index].point,
                              &ctx->precompute.put.dock_pos)) {
        return INT32_MIN;
    }

    ctx->precompute.put.plate_index = index;
    return 0;
}

int pokibrain_task_put_cake_layer_in_plate(struct pokibrain_callback_params *params)
{
    LOG_INF("RUNNING %s", __func__);
    struct pokibrain_user_context *ctx = params->world_context;
    struct plate *plate = &ctx->plate_list[ctx->precompute.put.plate_index];

    pos2_t plate_pos = ctx->precompute.put.dock_pos;
    if (nav_go_to_with_pathfinding(plate_pos, NULL, 0)) {
        return -1;
    }

    if (strat_put_layer(plate_pos, plate->cake_size, K_SECONDS(6))) {
        return -1;
    }

    return 0;
}

void add_layer_to_plate(struct plate *target_plate, struct cake_layer *layer)
{
    target_plate->cake_layer_colors[target_plate->cake_size] = layer->color;
    target_plate->cake_size += 1;
    layer->in_plate = true;
    layer->point = target_plate->point;
}

int32_t
pokibrain_reward_calculation_put_cake_layer_in_plate(struct pokibrain_callback_params *params)
{
    LOG_DBG("REWARD CALC %s", __func__);
    struct pokibrain_user_context *ctx = params->world_context;
    struct plate target_plate = ctx->plate_list[ctx->precompute.put.plate_index];

    add_layer_to_plate(&target_plate, &ctx->layer_list[ctx->index_held_layer]);
    int32_t score = 1 * target_plate.cake_size + is_golden_recipe(&target_plate) * 4 +
                    (target_plate.cake_layer_colors[0] == LAYER_COLOR_BROWN) +
                    (target_plate.cake_layer_colors[1] == LAYER_COLOR_YELLOW) +
                    (target_plate.cake_layer_colors[2] == LAYER_COLOR_PINK);
    return OFFSET_SCORE(score);
}

int pokibrain_completion_put_cake_layer_in_plate(struct pokibrain_callback_params *params)
{
    LOG_DBG("COMPLETION %s", __func__);
    struct pokibrain_user_context *ctx = params->world_context;
    struct plate *target_plate = &ctx->plate_list[ctx->precompute.put.plate_index];
    add_layer_to_plate(target_plate, &ctx->layer_list[ctx->index_held_layer]);
    ctx->nb_cake_layer_grabbed = 0;
    ctx->index_held_layer = -1;
    return 0;
}

// PUSH CAKE

int pokibrain_precompute_push_cake_layer_in_plate(struct pokibrain_callback_params *params)
{
    LOG_DBG("PRECOMPUTE %s", __func__);
    struct pokibrain_user_context *ctx = params->world_context;

    if (get_closest_available_plate_index(ctx, &ctx->precompute.push.plate_index)) {
        return INT32_MIN;
    }
    enum layer_color color = (enum layer_color)params->self->id;

    if (get_closest_available_cake_layer_index(ctx, color, &ctx->precompute.push.layer_index)) {
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

    return 0;
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

    return ARRAY_SIZE(layer_list) + ARRAY_SIZE(dispenser_list);
}

int pokibrain_task_push_cake_layer_in_plate(struct pokibrain_callback_params *params)
{
    int ret = 0;
    LOG_INF("RUNNING %s", __func__);
    struct pokibrain_user_context *ctx = params->world_context;
    // struct plate *plate = &ctx->plate_list[ctx->precompute.push.layer_index];

    obstacle_t obstacles[ARRAY_SIZE(layer_list) + ARRAY_SIZE(dispenser_list)];
    int nb_obstacles = get_static_components_as_obstacle(ctx, obstacles);
    if (nav_go_to_with_pathfinding(ctx->precompute.push.push_dock_pos, obstacles, nb_obstacles)) {
        // hack
        strat_set_target((pos2_t){.x = BOARD_CENTER_X, .y = BOARD_CENTER_Y, .a = 0});
        // ctx->layer_list[ctx->precompute.push.layer_index].in_plate = true;
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

    add_layer_to_plate(&target_plate, &ctx->layer_list[ctx->precompute.push.layer_index]);
    int32_t score = 1 * target_plate.cake_size + is_golden_recipe(&target_plate) * 4 +
                    (target_plate.cake_layer_colors[0] == LAYER_COLOR_BROWN) +
                    (target_plate.cake_layer_colors[1] == LAYER_COLOR_YELLOW) +
                    (target_plate.cake_layer_colors[2] == LAYER_COLOR_PINK);
    return OFFSET_SCORE(score);
}

int pokibrain_completion_push_cake_layer_in_plate(struct pokibrain_callback_params *params)
{
    LOG_DBG("COMPLETION %s", __func__);
    struct pokibrain_user_context *ctx = params->world_context;
    struct plate *target_plate = &ctx->plate_list[ctx->precompute.push.plate_index];
    add_layer_to_plate(target_plate, &ctx->layer_list[ctx->precompute.push.layer_index]);
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
    strat_force_motor_stop();
    k_sched_lock();
    while (1) {
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

void strat_init(enum team_color color)
{
    LOG_INF("Strat init with team side %s", get_side_name(color));

    static struct pokibrain_user_context world_context = {
        .nb_cherry_grabbed = 0,
        .nb_cake_layer_grabbed = 0,
    };
    world_context.team_color = (enum plate_color)color;
    pos2_t start_pos = world_context.team_color == PLATE_COLOR_BLUE
                           ? CONVERT_POINT2_TO_POS2(plate_list[7].point, -M_PI_2)
                           : CONVERT_POINT2_TO_POS2(plate_list[3].point, M_PI_2);
    strat_set_robot_pos(start_pos);
    strat_set_target(start_pos);

    memcpy(world_context.layer_list, layer_list, sizeof(layer_list));
    memcpy(world_context.plate_list, plate_list, sizeof(plate_list));
    memcpy(world_context.dispenser_list, dispenser_list, sizeof(dispenser_list));

    static struct pokibrain_task tasks[] = {
        {
            .name = "push brown",
            .task_precompute = pokibrain_precompute_push_cake_layer_in_plate,
            .reward_calculation = pokibrain_reward_calculation_push_cake_layer_in_plate,
            .task_process = pokibrain_task_push_cake_layer_in_plate,
            .completion_callback = pokibrain_completion_push_cake_layer_in_plate,
            .id = (uint32_t)LAYER_COLOR_BROWN,
        },
        {
            .name = "push yellow",
            .task_precompute = pokibrain_precompute_push_cake_layer_in_plate,
            .reward_calculation = pokibrain_reward_calculation_push_cake_layer_in_plate,
            .task_process = pokibrain_task_push_cake_layer_in_plate,
            .completion_callback = pokibrain_completion_push_cake_layer_in_plate,
            .id = (uint32_t)LAYER_COLOR_YELLOW,
        },
        {
            .name = "push pink",
            .task_precompute = pokibrain_precompute_push_cake_layer_in_plate,
            .reward_calculation = pokibrain_reward_calculation_push_cake_layer_in_plate,
            .task_process = pokibrain_task_push_cake_layer_in_plate,
            .completion_callback = pokibrain_completion_push_cake_layer_in_plate,
            .id = (uint32_t)LAYER_COLOR_PINK,
        },
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
