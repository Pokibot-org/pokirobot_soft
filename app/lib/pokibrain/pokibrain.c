#include "pokibrain.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdint.h>

LOG_MODULE_REGISTER(pokibrain, POKIBRAIN_LOG_LEVEL);

// DEFINES ---------------------------------------------------------------
#define GAME_ROUND_TIME_S 99

// TYPES -----------------------------------------------------------------
typedef enum {
	POKIBRAIN_THINK,
	POKIBRAIN_TASK_DONE,
	POKIBRAIN_DIE,
} pokibrain_events_t;

typedef struct {
	uint32_t number_of_tasks;
	struct pokibrain_task *tasks;
	struct k_thread child_thread;
	struct k_thread thread;
	k_tid_t thread_id;
	k_tid_t child_thread_id;
	uint32_t current_score;
	void *world_context;
	void *world_context_save;
	pokibrain_end_of_game_callback_t end_clbk;
	uint32_t start_time_ms;
	bool running;
} pokibrain_t;

// FUNC ------------------------------------------------------------------
static uint32_t pokibrain_get_time_in_match_ms(void);
static void pokibrain_periodic_timer(struct k_timer *timer_id);
static void pokibrain_match_timer(struct k_timer *timer_id);

// DECLARATIONS ----------------------------------------------------------
static pokibrain_t brain;
K_THREAD_STACK_DEFINE(child_stack, POKIBRAIN_CHILD_TASK_SIZE);
K_THREAD_STACK_DEFINE(stack, POKIBRAIN_STACK_SIZE);
K_MSGQ_DEFINE(event_queue, sizeof(pokibrain_events_t), 8, 4);
K_TIMER_DEFINE(timer, pokibrain_periodic_timer, NULL);
K_TIMER_DEFINE(match_end_timer, pokibrain_match_timer, NULL);

void pokibrain_run_task_work_handler(struct k_work *work);
K_WORK_DEFINE(run_task, pokibrain_run_task_work_handler);
K_MUTEX_DEFINE(run_task_mutex);
struct pokibrain_task *tasks_to_run[POKIBRAIN_DEPTH];

// IMPLEMENTATION --------------------------------------------------------
void pokibrain_run_task_work_handler(struct k_work *work)
{
	struct pokibrain_task task_to_run;
	struct pokibrain_callback_params params = {.robot_position = {.x = 0, .y = 0, .a = 0},
											   .time_in_match_ms = pokibrain_get_time_in_match_ms(),
											   .world_context = brain.world_context};
	if (!tasks_to_run[0]) {
		return;
	}
	k_mutex_lock(&run_task_mutex, K_FOREVER);
	task_to_run = *tasks_to_run[0];
	k_mutex_unlock(&run_task_mutex);

	if (!task_to_run.task_process(&params)) {
		// TODO:  update pos
		task_to_run.completion_callback(&params);
	}

	pokibrain_events_t ev = POKIBRAIN_TASK_DONE;
	k_msgq_put(&event_queue, &ev, K_FOREVER);
}

static uint32_t pokibrain_get_time_in_match_ms(void)
{
	return k_uptime_get_32() - brain.start_time_ms;
}

static void pokibrain_match_timer(struct k_timer *timer_id)
{
	pokibrain_events_t ev = POKIBRAIN_DIE;
	k_msgq_put(&event_queue, &ev, K_NO_WAIT);
}

static void pokibrain_periodic_timer(struct k_timer *timer_id)
{
	pokibrain_events_t ev = POKIBRAIN_THINK;
	k_msgq_put(&event_queue, &ev, K_NO_WAIT);
}

void pokibrain_get_best_task_recursive(uint8_t depth, void *world_context,
									   struct pokibrain_task *best_task)
{
	if (depth == 0) {
		return;
	}
}

void pokibrain_get_best_task(uint8_t depth, struct pokibrain_task *best_task,
							 uint32_t *best_task_score)
{
}

void pokibrain_think(void)
{
	LOG_DBG("Think");
	uint32_t best_score = 0;
	struct pokibrain_task *best_task = NULL;
	struct pokibrain_callback_params params = {.robot_position = {.x = 0, .y = 0, .a = 0},
											   .time_in_match_ms = pokibrain_get_time_in_match_ms(),
											   .world_context = brain.world_context};
	for (size_t i = 0; i < brain.number_of_tasks; i++) {
		uint32_t score;
		params.task_position = brain.tasks[i].task_position;
		score = brain.tasks[i].reward_calculation(&params);
		if (score >= best_score) {
			best_score = score;
			best_task = &brain.tasks[i];
		}
	}

	// run task and save next task
	if (!k_work_busy_get(&run_task)) {
		tasks_to_run[0] = best_task;
		k_work_submit(&run_task);
	}
}

void pokibrain_run_next_task(void)
{
}

void pokibrain_task(void *arg1, void *arg2, void *arg3)
{
	int64_t start_time, end_time;
	uint64_t total;

	brain.running = true;

	while (brain.running) {
		pokibrain_events_t ev;
		k_msgq_get(&event_queue, &ev, K_FOREVER);

		switch (ev) {
			case POKIBRAIN_THINK:
				if (brain.tasks == NULL || brain.number_of_tasks == 0) {
					break;
				}
				start_time = k_uptime_get();
				pokibrain_think();
				end_time = k_uptime_get();
				total = end_time - start_time;
				LOG_INF("Thinking took %llu ms", total);
				break;
			case POKIBRAIN_TASK_DONE:
				pokibrain_run_next_task();
				break;
			case POKIBRAIN_DIE:
				LOG_DBG("Die");
				k_timer_stop(&timer);
				k_msgq_purge(&event_queue);
				brain.running = false;
				if (brain.end_clbk) {
					brain.end_clbk();
				}
				break;
			default:
				break;
		}
	}

	LOG_INF("Brain dead");
}

int pokibrain_init(struct pokibrain_task *tasks, uint32_t number_of_tasks, void *world_context,
				   uint32_t world_context_size, pokibrain_end_of_game_callback_t end_clbk)
{
	brain.tasks = tasks;
	brain.number_of_tasks = number_of_tasks;
	brain.world_context = world_context;
	brain.world_context_save = k_malloc(POKIBRAIN_DEPTH * world_context_size);
	if (!brain.world_context_save) {
		LOG_ERR("k_malloc err");
		return -ENOMEM;
	}
	brain.end_clbk = end_clbk;
	brain.thread_id = k_thread_create(&brain.thread, stack, POKIBRAIN_STACK_SIZE, pokibrain_task,
									  NULL, NULL, NULL, POKIBRAIN_TASK_PRIORITY, 0, K_NO_WAIT);
	LOG_INF("Brain initialized");
	return 0;
}

void pokibrain_start(void)
{
	LOG_INF("Brain started");
	k_timer_start(&timer, K_MSEC(POKIBRAIN_PERIOD_MS), K_MSEC(POKIBRAIN_PERIOD_MS));
	k_timer_start(&match_end_timer, K_SECONDS(GAME_ROUND_TIME_S), K_NO_WAIT);
	brain.start_time_ms = k_uptime_get_32();
}

void pokibrain_think_now(void)
{
	LOG_INF("Think now requested");
	pokibrain_events_t ev = POKIBRAIN_THINK;
	k_msgq_put(&event_queue, &ev, K_NO_WAIT);
}
