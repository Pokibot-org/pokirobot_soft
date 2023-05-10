#ifndef POKIBRAIN_H
#define POKIBRAIN_H
#include <stdint.h>
#include <zephyr/kernel.h>

#include "pokutils.h"

#define POKIBRAIN_LOG_LEVEL		  3
#define POKIBRAIN_STACK_SIZE	  2048
#define POKIBRAIN_TASK_PRIORITY	  4
#define POKIBRAIN_PERIOD_MS		  500
#define POKIBRAIN_CHILD_TASK_SIZE 2048
#define POKIBRAIN_DEPTH			  3

struct pokibrain_callback_params {
	uint32_t time_in_match_ms;
	void *world_context;
};

typedef uint8_t (*pokibrain_task_function_t)(struct pokibrain_callback_params *params);
typedef int32_t (*pokibrain_reward_calculation_t)(struct pokibrain_callback_params *params);
typedef uint8_t (*pokibrain_completion_callback_t)(struct pokibrain_callback_params *params);

typedef void (*pokibrain_end_of_game_callback_t)(void);

struct pokibrain_task {
	pokibrain_task_function_t task_process;
	pokibrain_reward_calculation_t reward_calculation;
	pokibrain_completion_callback_t completion_callback;
	const char *name;
};

int pokibrain_init(struct pokibrain_task *tasks, uint32_t number_of_tasks, void *world_context,
				   uint32_t world_context_size, pokibrain_end_of_game_callback_t end_clbk);
void pokibrain_start(void);
void pokibrain_think_now(void);
#endif
