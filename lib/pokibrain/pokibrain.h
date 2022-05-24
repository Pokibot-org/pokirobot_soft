#ifndef POKIBRAIN_H
#define POKIBRAIN_H
#include <stdint.h>
#include <zephyr.h>

#include "user_context.h"
#include "utils.h"

#define POKIBRAIN_LOG_LEVEL 4
#define POKIBRAIN_STACK_SIZE 2048
#define POKIBRAIN_TASK_PRIORITY 4
#define POKIBRAIN_PERIOD_MS 500
#define POKIBRAIN_CHILD_TASK_SIZE 2048
#define POKIBRAIN_DEEPTH 2

typedef struct {
    bool is_done;
} pokibrain_process_data_t;

typedef struct {
    pos2_t task_position;
    pos2_t robot_position;
    uint32_t time;
    pokibrain_user_context_t* world_context;
} pokibrain_callback_params_t;

typedef uint8_t (*pokibrain_task_function_t)(
    pokibrain_callback_params_t* params);
typedef int32_t (*pokibrain_reward_calulation_t)(
    pokibrain_callback_params_t* params);
typedef uint8_t (*pokibrain_completion_callback_t)(
    pokibrain_callback_params_t* params);

typedef struct {
    pos2_t task_position;
    pokibrain_task_function_t task_process;
    pokibrain_reward_calulation_t reward_calculation;
    pokibrain_completion_callback_t completiton_callback;
    pokibrain_process_data_t _process_data;
} pokibrain_task_t;

void pokibrain_init(pokibrain_task_t* tasks, uint32_t number_of_tasks,
    pokibrain_user_context_t* world_context);
void pokibrain_start(void);
void pokibrain_think_now(void);
#endif
