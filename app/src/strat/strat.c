#include <zephyr/logging/log.h>
#include "pokibrain/pokibrain.h"
#include "strat.h"
#include "strat_interface.h"
#include "pokutils.h"
#include <math.h>
#include <stdint.h>
#include "nav/nav.h"
#include "pokpush/pokpush.h"
#include "pokstick/pokstick.h"
#include "pokuicom/pokuicom.h"
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

struct plant_zone {
    point2_t point;
};

struct solar_pannel {
    point2_t point;
};

struct pokibrain_user_context {
    pos2_t robot_pos;
    enum strat_team_color team_color;
    uint32_t plants_pushed;
    bool in_end_zone;
};

struct drop_zone drop_zones[] = {
    (struct drop_zone){.point = {.x = (float)DROP_ZONE_SIDE_LEN / 2 + BOARD_MIN_X,
                                 .y = BOARD_MIN_Y + (float)DROP_ZONE_SIDE_LEN / 2}},
    (struct drop_zone){.point = {.x = (float)DROP_ZONE_SIDE_LEN / 2 + BOARD_MIN_X,
                                 .y = BOARD_MAX_Y - (float)DROP_ZONE_SIDE_LEN / 2}},
    (struct drop_zone){
        .point = {.x = BOARD_MAX_X - (float)DROP_ZONE_SIDE_LEN / 2, .y = BOARD_CENTER_Y}},
};

struct plant_zone plant_zones[] = {
    (struct plant_zone){.point = {.x = BOARD_CENTER_X,
                                 .y = BOARD_CENTER_Y + 500}},
    (struct plant_zone){.point = {.x = BOARD_CENTER_X,
                                 .y = BOARD_CENTER_Y - 500}},
    (struct plant_zone){.point = {.x = BOARD_CENTER_X - 500,
                                 .y = BOARD_CENTER_Y + 300}},
    (struct plant_zone){.point = {.x = BOARD_CENTER_X - 500,
                                 .y = BOARD_CENTER_Y - 300}},
};

struct solar_pannel solar_pannels[] = {
    (struct solar_pannel){.point = {.x = BOARD_MIN_X + 275, .y = 0}},
    (struct solar_pannel){.point = {.x = BOARD_MIN_X + 275 + 225, .y = 0}},
    (struct solar_pannel){.point = {.x = BOARD_MIN_X + 275 + 225*2, .y = 0}},
    (struct solar_pannel){.point = {.x = BOARD_MIN_X + 275 + 225*2 + 550, .y = 0}},
    (struct solar_pannel){.point = {.x = BOARD_MIN_X + 275 + 225*2 + 550 + 225, .y = 0}},
    (struct solar_pannel){.point = {.x = BOARD_MIN_X + 275 + 225*2 + 550 + 255*2, .y = 0}},
    (struct solar_pannel){.point = {.x = BOARD_MAX_X - 275, .y = 0}},
    (struct solar_pannel){.point = {.x = BOARD_MAX_X - (275 + 225), .y = 0}},
    (struct solar_pannel){.point = {.x = BOARD_MAX_X - (275 + 225*2), .y = 0}},
};

struct point2 convert_point_for_team(enum strat_team_color color, struct point2 point)
{
    if (color == STRAT_TEAM_COLOR_BLUE) {
        return point;
    }

    point.x = -point.x;
    return point;
}

struct pos2 convert_pos_for_team(enum strat_team_color color, struct pos2 pos)
{
    if (color == STRAT_TEAM_COLOR_BLUE) {
        return pos;
    }

    pos.x = -pos.x;
    pos.a = -pos.a;

    return pos;
}

/**
REMPOTER LES PLANTES ET LES METTRE EN CULTURE
• 3 points par plante valide dans une zone adaptée
• 1 points supplémentaires si la plante valide est dans un pot
• 1 points supplémentaires si la plante valide est dans une jardinière

I.4.b. ORIENTER LES PANNEAUX SOLAIRES
• 5 points pour chaque panneau valide pour l’équipe;

I.4.c. ASSURER LA POLLINISATION DES PLANTES
• 5 points par zone de dépose de l’équipe occupée par au moins une coccinelle à la fin du match.
• 5 points supplémentaires par zone de dépose de l’équipe dans laquelle au moins une coccinelle est
en contact avec une plante ou un pot contenant une plante en fin de match.

• Attention : si une coccinelle réalise ses actions dans une zone adverse, alors les points ainsi fait vont à
l’équipe adverse.

I.4.d. RETOURNER SE RECHARGER LES BATTERIES
• 10 points si le robot de l’équipe est dans l’aire valide.
*/

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

int get_closest_push_point_for_solar_panels(point2_t robot_point, point2_t *point)
{
    point2_t closest_pt; 
    float closest_dist = MAXFLOAT;
    for (size_t i = 0; i < ARRAY_SIZE(solar_pannels); i++)
    {
        point2_t cur_point = solar_pannels[i].point;
        float dist = vec2_distance(robot_point, cur_point);
        if (dist < closest_dist) 
        {
            closest_dist = dist;
            closest_pt = cur_point;
        }
    }
    closest_pt.y = ROBOT_RADIUS + ROBOT_POKSTICK_LEN;
    *point = closest_pt;
    return 0;
}

int pokibrain_task_go_home(struct pokibrain_callback_params *params)
{
    LOG_INF("RUNNING %s", __func__);
    struct pokibrain_user_context *ctx = params->world_context;
    int err = 0;
    uint8_t score = 0;
    // point2_t point;
    // get_closest_push_point_for_solar_panels(ctx->robot_pos, &point);
    // nav_go_to_with_pathfinding(CONVERT_POINT2_TO_POS2(point, M_PI_2)); 

    const float offset_pokstick = M_PI / 6;
    const pos2_t pos_offset_pokstick = {
        .x = 0,
        .y = 0,
        .a = offset_pokstick
    };

    const float offset_pokpush = -M_PI / 3;
    const pos2_t pos_offset_pokpush = {
        .x = 0,
        .y = 0,
        .a = offset_pokpush
    };
    const float our_last_solar_x = BOARD_MIN_X + 275 + 225*2;
    const float push_y = (float)DROP_ZONE_SIDE_LEN / 2 - 40;
    pos2_t pos_to_push_solar_setup_0  = (pos2_t){
            .x = our_last_solar_x + ROBOT_RADIUS / 2,
            .y = push_y,
            .a = -M_PI
    };

    pos2_t pos_to_push_solar_midle_setup_0  = (pos2_t){
            .x = BOARD_MIN_X + 275 + 225*2 + 550,
            .y = 450,
            .a = -M_PI
    };

    pos2_t pos_to_push_solar_midle_setup_1  = (pos2_t){
            .x = BOARD_MIN_X + 275 + 225*2 + 550 + 255*2 + ROBOT_RADIUS / 2,
            .y = push_y,
            .a = -M_PI
    };

    pos2_t pos_to_push_solar_midle_end  = (pos2_t){
            .x = BOARD_MIN_X + 275 + 225*2 + 550 - ROBOT_RADIUS / 2,
            .y = push_y,
            .a = -M_PI
    };

    pos2_t pos_to_push_solar_after = (pos2_t){
            .x = BOARD_MIN_X + 450.0f / 2,
            .y = push_y,
            .a = -M_PI
    };


    // PUSH HOME PANELS
    strat_set_target(
        pos2_add(convert_pos_for_team(ctx->team_color, pos_to_push_solar_setup_0), pos_offset_pokstick)
    );
    err = strat_wait_target(STRAT_PLANAR_TARGET_SENSITIVITY_DEFAULT,
                      STRAT_ANGULAR_TARGET_SENSITIVITY_DEFAULT, 15000, 6000);

    if (!err) {
        pokstick_deploy();

        k_sleep(K_MSEC(500));

        strat_set_target(
            pos2_add(convert_pos_for_team(ctx->team_color, pos_to_push_solar_after), pos_offset_pokstick)
        );

        strat_wait_target(STRAT_PLANAR_TARGET_SENSITIVITY_DEFAULT,
                                    STRAT_ANGULAR_TARGET_SENSITIVITY_DEFAULT, 20000, 15000);

        pokstick_retract();

        score += 3 * 5;
        pokuicom_send_score(score);
    }

    // PUSH MIDDLE PANNELS
    strat_set_target(
        pos2_add(convert_pos_for_team(ctx->team_color, pos_to_push_solar_midle_setup_0), pos_offset_pokstick)
    );
    err = strat_wait_target(STRAT_PLANAR_TARGET_SENSITIVITY_DEFAULT,
                      STRAT_ANGULAR_TARGET_SENSITIVITY_DEFAULT, 15000, 1000);

    strat_set_target(
        pos2_add(convert_pos_for_team(ctx->team_color, pos_to_push_solar_midle_setup_1), pos_offset_pokstick)
    );
    err = strat_wait_target(STRAT_PLANAR_TARGET_SENSITIVITY_DEFAULT,
                      STRAT_ANGULAR_TARGET_SENSITIVITY_DEFAULT, 15000, 3000);

    if (!err) {
        pokstick_deploy();

        k_sleep(K_MSEC(500));

        strat_set_target(
            pos2_add(convert_pos_for_team(ctx->team_color, pos_to_push_solar_midle_end), pos_offset_pokstick)
        );

        strat_wait_target(STRAT_PLANAR_TARGET_SENSITIVITY_DEFAULT,
                                    STRAT_ANGULAR_TARGET_SENSITIVITY_DEFAULT, 20000, 15000);

        pokstick_retract();

        score += 3 * 5;
        pokuicom_send_score(score);
    }

    // PUSH PLANTS
    if(pokibrain_get_time_remaining_in_match_ms() / 1000 < 40)
    {
        goto go_home;
    }

    float a_push = 2*M_PI/3;
    strat_set_target(pos2_add(
        convert_pos_for_team(ctx->team_color,
                             (pos2_t){.x = BOARD_CENTER_X, .y = BOARD_CENTER_Y, .a = a_push}),
        pos_offset_pokpush));
    err = strat_wait_target(STRAT_PLANAR_TARGET_SENSITIVITY_DEFAULT,
                            STRAT_ANGULAR_TARGET_SENSITIVITY_DEFAULT, 15000, 6000);

    if (!err) {
        pokpush_deploy();

        k_sleep(K_MSEC(500));

        pos2_t dock_pos = {
            .x = drop_zones[0].point.x + ROBOT_RADIUS/2,
            .y = drop_zones[0].point.y + ROBOT_RADIUS/2,
            .a = a_push
        };
        strat_set_target(pos2_add( convert_pos_for_team(ctx->team_color, dock_pos), pos_offset_pokpush));

        strat_wait_target(STRAT_PLANAR_TARGET_SENSITIVITY_DEFAULT,
                                    STRAT_ANGULAR_TARGET_SENSITIVITY_DEFAULT, 20000, 15000);

        pokpush_retract();

        score += 2 * 3; // Let's hope there is is juste 2 plantes
        pokuicom_send_score(score);
    }

go_home:
    // GO HOME
    {
        pos2_t docking_pos = CONVERT_POINT2_TO_POS2(drop_zones[1].point, 0);
        strat_set_target(convert_pos_for_team(ctx->team_color, docking_pos));
        err = strat_wait_target(STRAT_PLANAR_TARGET_SENSITIVITY_DEFAULT,
                        STRAT_ANGULAR_TARGET_SENSITIVITY_DEFAULT, 20000, 10000);
    }

    // IF WE FAIL TO GO IN THE END ZONE
    if (err)
    {
        pos2_t docking_pos = CONVERT_POINT2_TO_POS2(drop_zones[2].point, 0);
        strat_set_target(convert_pos_for_team(ctx->team_color, docking_pos));
        err = strat_wait_target(STRAT_PLANAR_TARGET_SENSITIVITY_DEFAULT,
                        STRAT_ANGULAR_TARGET_SENSITIVITY_DEFAULT, 50000, 40000);
    }

    score += 10;
    pokuicom_send_score(score);
    k_sleep(K_SECONDS(1));
    pokuicom_send_score(score);
    k_sleep(K_FOREVER);
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
}

static void strat_end_game_clbk(void *world_context)
{
    // struct pokibrain_user_context *ctx = world_context;
    LOG_INF("GAME IS OVER");
    nav_stop();
    strat_force_motor_stop();
}

const char *get_side_name(enum strat_team_color color)
{
    switch (color) {
        case STRAT_TEAM_COLOR_BLUE:
            return "BLUE";
        case STRAT_TEAM_COLOR_YELLOW:
            return "YELLOW";
        default:
        case STRAT_TEAM_COLOR_NONE:
            return "UNKNOWN";
    }
}

// -------------------------- PUBLIC FUNCTIONS ---------------------------

void strat_init(enum strat_team_color color)
{
    LOG_INF("Strat init with team side %s", get_side_name(color));

    static struct pokibrain_user_context world_context = {
        .plants_pushed = 0,
    };
    world_context.team_color = color;

    pos2_t start_pos_blue = {
        .x = BOARD_MIN_X + ROBOT_RADIUS,
        .y = BOARD_MIN_Y + 332/2 + 120,
        .a = -M_PI_2
    };
    pos2_t start_pos = convert_pos_for_team(color, start_pos_blue);
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
