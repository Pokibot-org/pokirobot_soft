#include <zephyr/logging/log.h>
#include "pokibrain/pokibrain.h"
#include "strat.h"
#include "pokutils.h"
#include <stdint.h>

LOG_MODULE_REGISTER(strategy);

#define BOARD_SIZE_X   2000
#define BOARD_CENTER_X 1000
#define BOARD_SIZE_Y   3000
#define BOARD_CENTER_Y 1500
#define PLATE_SIZE	   450

#define NB_CHERRY_PER_DISPENSER	 10
#define CHERRY_DISPENSER_WIDTH	 300
#define CHERRY_DISPENSER_DEPTH	 30
#define CHERRY_DISPENSER_WIDTH_2 150
#define CHERRY_DISPENSER_DEPTH_2 15
enum layer_color {
	LAYER_COLOR_YELLOW,
	LAYER_COLOR_PINK,
	LAYER_COLOR_BROWN,
};

struct cake_layer {
	enum layer_color color;
	point2_t pos;
};

static struct cake_layer layer_list[] = {
	(struct cake_layer){.color = LAYER_COLOR_BROWN,
						.pos = {.x = BOARD_CENTER_X - 275, .y = BOARD_CENTER_Y - 375}},
	(struct cake_layer){.color = LAYER_COLOR_BROWN,
						.pos = {.x = BOARD_CENTER_X + 275, .y = BOARD_CENTER_Y - 375}},
	(struct cake_layer){.color = LAYER_COLOR_BROWN,
						.pos = {.x = BOARD_CENTER_X - 275, .y = BOARD_CENTER_Y + 375}},
	(struct cake_layer){.color = LAYER_COLOR_BROWN,
						.pos = {.x = BOARD_CENTER_X + 275, .y = BOARD_CENTER_Y + 375}},
	(struct cake_layer){.color = LAYER_COLOR_PINK, .pos = {.x = 225, .y = 575}},
	(struct cake_layer){.color = LAYER_COLOR_PINK, .pos = {.x = BOARD_SIZE_X - 225, .y = 575}},
	(struct cake_layer){.color = LAYER_COLOR_PINK, .pos = {.x = 225, .y = BOARD_SIZE_Y - 575}},
	(struct cake_layer){.color = LAYER_COLOR_PINK,
						.pos = {.x = BOARD_SIZE_X - 225, .y = BOARD_SIZE_Y - 575}},
	(struct cake_layer){.color = LAYER_COLOR_PINK, .pos = {.x = 225, .y = 775}},
	(struct cake_layer){.color = LAYER_COLOR_PINK, .pos = {.x = BOARD_SIZE_X - 225, .y = 775}},
	(struct cake_layer){.color = LAYER_COLOR_PINK, .pos = {.x = 225, .y = BOARD_SIZE_Y - 775}},
	(struct cake_layer){.color = LAYER_COLOR_PINK,
						.pos = {.x = BOARD_SIZE_X - 225, .y = BOARD_SIZE_Y - 775}},

};

enum plate_color {
	PLATE_COLOR_BLUE,
	PLATE_COLOR_GREEN,
};

struct plate {
	enum plate_color color;
	point2_t pos;
	uint8_t cake_size;
};

static struct plate plate_list[] = {
	// GREEN
	(struct plate){.color = PLATE_COLOR_GREEN, .pos = {.x = 225, .y = 225}},
	(struct plate){.color = PLATE_COLOR_GREEN, .pos = {.x = BOARD_CENTER_X + 275, .y = 225}},
	(struct plate){.color = PLATE_COLOR_GREEN,
				   .pos = {.x = BOARD_SIZE_X - 225, .y = BOARD_CENTER_Y - 375}},
	(struct plate){.color = PLATE_COLOR_GREEN, .pos = {.x = 225, .y = BOARD_CENTER_Y + 375}},
	(struct plate){.color = PLATE_COLOR_GREEN,
				   .pos = {.x = BOARD_SIZE_X - 225, .y = BOARD_SIZE_Y - 225}},
	// BLUE
	(struct plate){.color = PLATE_COLOR_BLUE, .pos = {.x = BOARD_CENTER_X - 275, .y = 225}},
	(struct plate){.color = PLATE_COLOR_BLUE, .pos = {.x = BOARD_SIZE_X - 225, .y = 225}},
	(struct plate){.color = PLATE_COLOR_BLUE, .pos = {.x = 225, .y = BOARD_CENTER_Y - 375}},
	(struct plate){.color = PLATE_COLOR_BLUE,
				   .pos = {.x = BOARD_SIZE_X - 225, .y = BOARD_CENTER_Y + 375}},
	(struct plate){.color = PLATE_COLOR_BLUE, .pos = {.x = 225, .y = BOARD_SIZE_Y - 225}},
};

struct cherry_dispenser {
	pos2_t pos;
};

struct cherry_dispenser dispenser_list[] = {
	(struct cherry_dispenser){.pos = {.x = BOARD_CENTER_X, .y = CHERRY_DISPENSER_WIDTH_2, .a = 0}},
	(struct cherry_dispenser){
		.pos = {.x = BOARD_CENTER_X, .y = BOARD_SIZE_Y - CHERRY_DISPENSER_WIDTH_2, .a = 0}},
	(struct cherry_dispenser){.pos = {.x = CHERRY_DISPENSER_DEPTH_2, .y = BOARD_CENTER_Y, .a = 0}},
	(struct cherry_dispenser){
		.pos = {.x = BOARD_SIZE_X - CHERRY_DISPENSER_DEPTH_2, .y = BOARD_CENTER_Y, .a = 0}},
};

struct pokibrain_user_context {
	uint8_t nb_cake_layer_grabbed;
	uint8_t nb_cherry_grabbed;
};

uint8_t pokibrain_task_grab_cake_layer(struct pokibrain_callback_params *params)
{
	LOG_INF("RUNNING %s", __func__);
	return 0;
}

int32_t pokibrain_reward_calculation_grab_cake_layer(struct pokibrain_callback_params *params)
{
	LOG_DBG("REWARD CALC %s", __func__);
	return 0;
}

uint8_t pokibrain_completion_grab_cake_layer(struct pokibrain_callback_params *params)
{
	LOG_DBG("COMPLETION %s", __func__);

	struct pokibrain_user_context *ctx = params->world_context;

	ctx->nb_cake_layer_grabbed = MIN(3, ctx->nb_cake_layer_grabbed + 1);
	return 0;
}

uint8_t pokibrain_task_put_cake_layer_in_plate(struct pokibrain_callback_params *params)
{
	LOG_INF("RUNNING %s", __func__);
	return 0;
}

int32_t
pokibrain_reward_calculation_put_cake_layer_in_plate(struct pokibrain_callback_params *params)
{
	LOG_DBG("REWARD CALC %s", __func__);
	struct pokibrain_user_context *ctx = params->world_context;
	return 10 * ctx->nb_cake_layer_grabbed;
}

uint8_t pokibrain_completion_put_cake_layer_in_plate(struct pokibrain_callback_params *params)
{
	LOG_DBG("COMPLETION %s", __func__);
	struct pokibrain_user_context *ctx = params->world_context;
	ctx->nb_cake_layer_grabbed = 0;
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

static void strat_end_game_clbk(void)
{
	LOG_INF("GAME IS OVER");
}

void strat_init(void)
{
	static struct pokibrain_user_context world_context;
	static struct pokibrain_task tasks[] = {
		{.name = "grab",
		 .reward_calculation = pokibrain_reward_calculation_grab_cake_layer,
		 .task_process = pokibrain_task_grab_cake_layer,
		 .completion_callback = pokibrain_completion_grab_cake_layer},
		{.name = "put",
		 .reward_calculation = pokibrain_reward_calculation_put_cake_layer_in_plate,
		 .task_process = pokibrain_task_put_cake_layer_in_plate,
		 .completion_callback = pokibrain_completion_put_cake_layer_in_plate}};
	pokibrain_init(tasks, sizeof(tasks) / sizeof(tasks[0]), &world_context,
				   sizeof(struct pokibrain_user_context), strat_end_game_clbk);
}

void strat_run(void)
{
	pokibrain_start();
}