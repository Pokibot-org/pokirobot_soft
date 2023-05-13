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

#define CAKE_MAX_NB_LAYERS 3

#define NB_CHERRY_PER_DISPENSER	 10
#define CHERRY_DISPENSER_WIDTH	 300
#define CHERRY_DISPENSER_DEPTH	 30
#define CHERRY_DISPENSER_WIDTH_2 150
#define CHERRY_DISPENSER_DEPTH_2 15

#define ROBOT_MAX_LAYER_GRABBED 1

enum layer_color {
	LAYER_COLOR_YELLOW,
	LAYER_COLOR_PINK,
	LAYER_COLOR_BROWN,
};

struct cake_layer {
	enum layer_color color;
	point2_t pos;
	uint8_t in_plate;
};

const static struct cake_layer layer_list[] = {
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
	enum layer_color cake_layer_colors[CAKE_MAX_NB_LAYERS];
};

const static struct plate plate_list[] = {
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
};

struct put_layer_precompute_storage {
	uint8_t plate_index;
};

struct pokibrain_user_context {
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
	} precompute;
};

// ------------------------------ HELPERS ------------------------------

int get_closest_available_cake_layer_index(struct pokibrain_user_context *ctx,
										   enum layer_color color, uint8_t *index)
{
	return 0;
};

int get_closest_available_plate_index(struct pokibrain_user_context *ctx, uint8_t *index)
{
	return 0;
};

int get_closest_in_build_plate_index(struct pokibrain_user_context *ctx, uint8_t *index)
{
	return 0;
};

// ---------------------------- STRAT TASKS ----------------------------

// ---------------------------------------- GRAB CAKE

int pokibrain_precompute_grab_cake_layer(struct pokibrain_callback_params *params)
{
	LOG_DBG("REWARD CALC %s", __func__);
	struct pokibrain_user_context *ctx = params->world_context;
	if (ctx->nb_cake_layer_grabbed >= ROBOT_MAX_LAYER_GRABBED) {
		return INT32_MIN;
	}
	enum layer_color color = (enum layer_color)params->self->id;
	uint8_t index = 255;
	if (get_closest_available_cake_layer_index(ctx, color, &index)) {
		return INT32_MIN;
	}

	if (index == 255) {
		LOG_ERR("Should not happens");
		return INT32_MIN;
	}

	ctx->precompute.grab.layer_index = index;
	return 0;
}

int pokibrain_task_grab_cake_layer(struct pokibrain_callback_params *params)
{
	LOG_INF("RUNNING %s", __func__);
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
	ctx->nb_cake_layer_grabbed = MIN(ROBOT_MAX_LAYER_GRABBED, ctx->nb_cake_layer_grabbed + 1);
	return 0;
}

// ---------------------------------------- PUT CAKE
int pokibrain_precompute_put_cake_layer_in_plate(struct pokibrain_callback_params *params)
{
	LOG_DBG("REWARD CALC %s", __func__);
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
	if (index == 255) {
		LOG_ERR("Should not happens");
		return INT32_MIN;
	}

	if (ctx->plate_list[index].cake_size >= CAKE_MAX_NB_LAYERS) {
		return INT32_MIN;
	}

	ctx->precompute.put.plate_index = index;
	return 0;
}

int pokibrain_task_put_cake_layer_in_plate(struct pokibrain_callback_params *params)
{
	LOG_INF("RUNNING %s", __func__);
	return 0;
}

void add_layer_to_plate(struct plate *target_plate, struct cake_layer *layer)
{
	target_plate->cake_layer_colors[target_plate->cake_size] = layer->color;
	target_plate->cake_size += 1;
	layer->in_plate = true;
	layer->pos = target_plate->pos;
}

int32_t
pokibrain_reward_calculation_put_cake_layer_in_plate(struct pokibrain_callback_params *params)
{
	LOG_DBG("REWARD CALC %s", __func__);
	struct pokibrain_user_context *ctx = params->world_context;
	struct plate target_plate = ctx->plate_list[ctx->precompute.put.plate_index];

	add_layer_to_plate(&target_plate, &ctx->layer_list[ctx->index_held_layer]);
	// TODO: is golden receipe ?
	return 1 * target_plate.cake_size;
}

int pokibrain_completion_put_cake_layer_in_plate(struct pokibrain_callback_params *params)
{
	LOG_DBG("COMPLETION %s", __func__);
	struct pokibrain_user_context *ctx = params->world_context;
	struct plate *target_plate = &ctx->plate_list[ctx->precompute.put.plate_index];
	add_layer_to_plate(target_plate, &ctx->layer_list[ctx->index_held_layer]);
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
	// TODO: CALCULATE SCORE
}

// -------------------------- PUBLIC FUNCTIONS ---------------------------

void strat_init(void)
{
	static struct pokibrain_user_context world_context = {
		.team_color = PLATE_COLOR_BLUE,
		.nb_cherry_grabbed = 0,
		.nb_cake_layer_grabbed = 0,
	};

	memcpy(world_context.layer_list, layer_list, ARRAY_SIZE(layer_list));
	memcpy(world_context.plate_list, plate_list, ARRAY_SIZE(plate_list));
	memcpy(world_context.dispenser_list, dispenser_list, ARRAY_SIZE(dispenser_list));

	static struct pokibrain_task tasks[] = {
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
		 .id = 1}};
	pokibrain_init(tasks, sizeof(tasks) / sizeof(tasks[0]), &world_context,
				   sizeof(struct pokibrain_user_context), strat_end_game_clbk);
}

void strat_run(void)
{
	pokibrain_start();
}