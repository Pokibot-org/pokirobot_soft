#ifndef POKIBRAIN_H
#define POKIBRAIN_H
#include <stdint.h>
#include <zephyr/kernel.h>

#include "pokutils.h"

#define POKIBRAIN_LOG_LEVEL		  3
#define POKIBRAIN_STACK_SIZE	  2048
#define POKIBRAIN_TASK_PRIORITY	  4
#define POKIBRAIN_PERIOD_MS		  2000
#define POKIBRAIN_CHILD_TASK_SIZE 2048
#define POKIBRAIN_DEPTH			  3

struct pokibrain_callback_params {
	uint32_t time_in_match_ms;
	void *world_context;
	struct pokibrain_task *self;
};

typedef int (*pokibrain_task_precompute_t)(struct pokibrain_callback_params *params);
typedef int (*pokibrain_task_function_t)(struct pokibrain_callback_params *params);
typedef int32_t (*pokibrain_reward_calculation_t)(struct pokibrain_callback_params *params);
typedef int (*pokibrain_completion_callback_t)(struct pokibrain_callback_params *params);

typedef void (*pokibrain_pre_think_callback_t)(void *world_context);
typedef void (*pokibrain_end_of_game_callback_t)(void *world_context);

struct pokibrain_task {
	pokibrain_task_precompute_t task_precompute;
	pokibrain_task_function_t task_process;
	pokibrain_reward_calculation_t reward_calculation;
	pokibrain_completion_callback_t completion_callback;
	const char *name;
	uint32_t id;
};

int pokibrain_init(struct pokibrain_task *tasks, uint32_t number_of_tasks, void *world_context,
				   uint32_t world_context_size, pokibrain_pre_think_callback_t pre_think_clbk,
				   pokibrain_end_of_game_callback_t end_clbk);
void pokibrain_start(void);
void pokibrain_think_now(void);
#endif
