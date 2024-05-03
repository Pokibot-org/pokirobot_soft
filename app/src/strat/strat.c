#include <zephyr/logging/log.h>
#include "pokibrain/pokibrain.h"
#include "strat.h"
#include "strat_interface.h"
#include "pokutils.h"
#include <math.h>
#include <stdint.h>
#include "nav/nav.h"
#include "pokpush/pokpush.h"
#include "global_def.h"

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

#define PUSH_TOOL_OFFSET (M_PI + M_PI / 6)

#define DROP_ZONE_SIDE_LEN 450

struct drop_zone {
    point2_t point;
};

struct pokibrain_user_context {
    pos2_t robot_pos;
    enum team_color team_color;
    uint32_t plants_pushed;
};

struct drop_zone drop_zones[] = {
    (struct drop_zone){.point = {.x = (float)DROP_ZONE_SIDE_LEN / 2 + BOARD_MIN_X,
                                 .y = BOARD_MIN_Y + (float)DROP_ZONE_SIDE_LEN / 2}},
    (struct drop_zone){.point = {.x = (float)DROP_ZONE_SIDE_LEN / 2 + BOARD_MIN_X,
                                 .y = BOARD_MAX_Y - (float)DROP_ZONE_SIDE_LEN / 2}},
    (struct drop_zone){
        .point = {.x = BOARD_MAX_X - (float)DROP_ZONE_SIDE_LEN / 2, .y = BOARD_CENTER_Y}},
};

struct point2 convert_point_for_team(enum team_color color, struct point2 point)
{
    if (color == TEAM_COLOR_BLUE) {
        return point;
    }

    point.x = -point.x;
    return point;
}

struct pos2 convert_pos_for_team(enum team_color color, struct pos2 pos)
{
    if (color == TEAM_COLOR_BLUE) {
        return pos;
    }

    pos.x = -pos.x;
    pos.a = pos.a + 180.0f;
    if (pos.a > 360.0f) {
        pos.a -= 360.0f;
    }
    return pos;
}

int calculate_score(struct pokibrain_user_context *ctx)
{
    return ctx->plants_pushed;
}

int get_home_docking_pos(point2_t robot_point, point2_t end_point, pos2_t *dock_pos)
{
    float segment_len = vec2_distance(robot_point, end_point);
    float frac = ROBOT_RADIUS * 2 / segment_len;
    vec2_t diff = point2_diff(end_point, robot_point);

    dock_pos->x = end_point.x - diff.dx * frac;
    dock_pos->y = end_point.y - diff.dy * frac;
    dock_pos->a = angle_modulo(atan2f(diff.dy, diff.dx) + PUSH_TOOL_OFFSET);
    return 0;
}

int pokibrain_task_go_home(struct pokibrain_callback_params *params)
{
    LOG_INF("RUNNING %s", __func__);
    struct pokibrain_user_context *ctx = params->world_context;

    point2_t end_point = convert_point_for_team(ctx->team_color, drop_zones[1].point);
    LOG_ERR("color: %d", ctx->team_color);
    // pos2_t docking_pos = CONVERT_POINT2_TO_POS2(end_point, 0);
    pos2_t docking_pos;
    get_home_docking_pos((point2_t){.x = BOARD_CENTER_X, .y = BOARD_CENTER_Y}, end_point,
                         &docking_pos);
    nav_go_to_with_pathfinding(docking_pos, NULL, 0);
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
        case TEAM_COLOR_YELLOW:
            return "YELLOW";
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
        .plants_pushed = 0,
    };
    world_context.team_color = color;
    pos2_t start_pos =
        convert_pos_for_team(color, CONVERT_POINT2_TO_POS2(drop_zones[0].point, M_PI_2));
    strat_set_robot_pos(start_pos);
    strat_set_target(start_pos);

    // memcpy(world_context.layer_list, layer_list, sizeof(layer_list));
    // memcpy(world_context.plate_list, plate_list, sizeof(plate_list));
    // memcpy(world_context.dispenser_list, dispenser_list, sizeof(dispenser_list));

    static struct pokibrain_task tasks[] = {
        {.name = "go home",
         .task_precompute = NULL,
         .completion_callback = pokibrain_completion_go_home,
         .task_process = pokibrain_task_go_home,
         .reward_calculation = pokibrain_reward_calculation_go_home}};

    pokibrain_init(tasks, sizeof(tasks) / sizeof(tasks[0]), &world_context,
                   sizeof(struct pokibrain_user_context), strat_pre_think, strat_end_game_clbk);

    nav_init();
}

void strat_run(void)
{
    pokibrain_start();
}
