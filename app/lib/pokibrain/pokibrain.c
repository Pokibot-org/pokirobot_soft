#include "pokibrain.h"
#include "zephyr/kernel/thread_stack.h"

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
    uint32_t world_context_size;
    pokibrain_pre_think_callback_t pre_think_clbk;
    pokibrain_end_of_game_callback_t end_clbk;
    uint32_t start_time_ms;
    bool running;
} pokibrain_t;

// FUNC ------------------------------------------------------------------
static void pokibrain_periodic_timer(struct k_timer *timer_id);
static void pokibrain_match_timer(struct k_timer *timer_id);

// DECLARATIONS ----------------------------------------------------------
static pokibrain_t brain;
K_THREAD_STACK_DEFINE(stack, POKIBRAIN_STACK_SIZE);
K_MSGQ_DEFINE(event_queue, sizeof(pokibrain_events_t), 8, 4);
K_TIMER_DEFINE(timer, pokibrain_periodic_timer, NULL);
K_TIMER_DEFINE(match_end_timer, pokibrain_match_timer, NULL);

const struct k_work_queue_config task_runner_cfg = {.name = "Pokibrain WQ", .no_yield = false};
struct k_work_q task_runner_work_queue;
K_THREAD_STACK_DEFINE(task_runner_stack, POKIBRAIN_CHILD_TASK_SIZE);
void pokibrain_run_task_work_handler(struct k_work *work);
K_WORK_DEFINE(run_task, pokibrain_run_task_work_handler);
K_MSGQ_DEFINE(task_to_run_msgq, sizeof(struct pokibrain_task), POKIBRAIN_DEPTH, 4);

static void end_game_work_handler(struct k_work *work);
K_WORK_DEFINE(end_game_work, end_game_work_handler);
// IMPLEMENTATION --------------------------------------------------------

int run_task_precompute(struct pokibrain_task *task_to_run,
                        struct pokibrain_callback_params *params)
{
    if (!task_to_run->task_precompute) {
        return 0;
    }
    return task_to_run->task_precompute(params);
}

void pokibrain_run_task_work_handler(struct k_work *work)
{
    struct pokibrain_task task_to_run;
    if (k_msgq_get(&task_to_run_msgq, &task_to_run, K_MSEC(100))) {
        LOG_ERR("No task to run found");
        return;
    }

    struct pokibrain_callback_params params = {.time_in_match_ms = pokibrain_get_time_in_match_ms(),
                                               .world_context = brain.world_context,
                                               .self = &task_to_run};

    if (run_task_precompute(&task_to_run, &params)) {
        LOG_ERR("Error in precompute");
        goto exit;
    }

    if (task_to_run.task_process(&params)) {
        LOG_ERR("Error in task run");
        goto exit;
    }
    LOG_INF("Task %s run ok", task_to_run.name);
    task_to_run.completion_callback(&params);
    // LOG_WRN("Problems during task process, flushing task to run queue");
    // k_msgq_purge(&task_to_run_msgq);
exit:
    pokibrain_events_t ev = POKIBRAIN_TASK_DONE;
    k_msgq_put(&event_queue, &ev, K_FOREVER);
}

uint32_t pokibrain_get_time_in_match_ms(void)
{
    return k_uptime_get_32() - brain.start_time_ms;
}

uint32_t pokibrain_get_time_remaining_in_match_ms(void)
{
    return MAX((int)GAME_ROUND_TIME_S * 1000 - (int)pokibrain_get_time_in_match_ms(), 0);
}


static void end_game_work_handler(struct k_work *work)
{
    LOG_DBG("Die");
    k_timer_stop(&timer);
    k_msgq_purge(&event_queue);
    brain.running = false;
    if (brain.end_clbk) {
        brain.end_clbk(brain.world_context);
    }
}

static void pokibrain_match_timer(struct k_timer *timer_id)
{
    k_work_submit(&end_game_work);
}

static void pokibrain_periodic_timer(struct k_timer *timer_id)
{
    pokibrain_events_t ev = POKIBRAIN_THINK;
    k_msgq_put(&event_queue, &ev, K_NO_WAIT);
}

int32_t pokibrain_get_best_task_recursive(uint8_t depth, struct pokibrain_task *current_task,
                                          void *world_context)
{
    LOG_DBG("Depth %d", depth);
    void *current_world_context =
        (uint8_t *)brain.world_context_save + depth * brain.world_context_size;
    memcpy(current_world_context, world_context, brain.world_context_size);
    struct pokibrain_callback_params params = {.time_in_match_ms = pokibrain_get_time_in_match_ms(),
                                               .world_context = current_world_context,
                                               .self = current_task};
    if (run_task_precompute(current_task, &params)) {
        return INT32_MIN;
    }
    // Evaluate the nb of point that the task will give before considering it done
    int32_t current_task_score = current_task->reward_calculation(&params);
    if (current_task_score == INT32_MIN) {
        return INT32_MIN;
    }
    current_task->completion_callback(&params);
    if (depth == 0) {
        // FIXME: should be the current total of points
        LOG_DBG("Depth 0, current task reward %d", current_task_score);
        return current_task_score;
    }

    int32_t best_score = INT32_MIN;
    for (size_t i = 0; i < brain.number_of_tasks; i++) {
        int32_t score =
            pokibrain_get_best_task_recursive(depth - 1, &brain.tasks[i], current_world_context);
        if (score >= best_score) {
            best_score = score;
        }
    }

    return best_score + current_task_score;
}

int pokibrain_get_best_task(struct pokibrain_task **best_task)
{
    LOG_DBG("Searching for the best task to do");

    if (brain.pre_think_clbk) {
        brain.pre_think_clbk(brain.world_context);
    }

    int32_t best_score = INT32_MIN;
    for (size_t i = 0; i < brain.number_of_tasks; i++) {
        int32_t score;
        score = pokibrain_get_best_task_recursive(POKIBRAIN_DEPTH - 1, &brain.tasks[i],
                                                  brain.world_context);
        if (score >= best_score) {
            best_score = score;
            *best_task = &brain.tasks[i];
            LOG_DBG("New best task name: %s, score %d", (*best_task)->name, best_score);
        }
    }
    if (best_score == INT32_MIN) {
        return -1;
    }
    LOG_INF("Best task name : %s, score %d", (*best_task)->name, best_score);
    return 0;
}

void pokibrain_think(void)
{
    LOG_DBG("Think");
    struct pokibrain_task *best_task;
    if (pokibrain_get_best_task(&best_task)) {
        LOG_WRN("No task to do, waiting");
        return;
    }

    // run task and save next task
    if (!k_work_busy_get(&run_task)) {
        k_msgq_put(&task_to_run_msgq, best_task, K_MSEC(100));
        k_work_submit_to_queue(&task_runner_work_queue, &run_task);
    } else {
        LOG_WRN("One task is already running");
    }
}

void pokibrain_run_next_task(void)
{
    pokibrain_think_now();
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
                LOG_DBG("Thinking took %llu ms", total);
                break;
            case POKIBRAIN_TASK_DONE:
                pokibrain_run_next_task();
                break;
            default:
                break;
        }
    }

    LOG_INF("Brain dead");
}

int pokibrain_init(struct pokibrain_task *tasks, uint32_t number_of_tasks, void *world_context,
                   uint32_t world_context_size, pokibrain_pre_think_callback_t pre_think_clbk,
                   pokibrain_end_of_game_callback_t end_clbk)
{
    brain.tasks = tasks;
    brain.number_of_tasks = number_of_tasks;
    brain.world_context = world_context;
    brain.world_context_size = world_context_size;
    brain.world_context_save = k_malloc(POKIBRAIN_DEPTH * world_context_size);
    if (!brain.world_context_save) {
        LOG_ERR("k_malloc err");
        return -ENOMEM;
    }
    brain.pre_think_clbk = pre_think_clbk;
    brain.end_clbk = end_clbk;
    brain.thread_id = k_thread_create(&brain.thread, stack, POKIBRAIN_STACK_SIZE, pokibrain_task,
                                      NULL, NULL, NULL, POKIBRAIN_TASK_PRIORITY, 0, K_NO_WAIT);
    LOG_INF("Brain initialized");
    return 0;
}

void pokibrain_start(void)
{
    LOG_INF("Brain started");
    k_work_queue_init(&task_runner_work_queue);
    k_work_queue_start(&task_runner_work_queue, task_runner_stack,
                       K_THREAD_STACK_SIZEOF(task_runner_stack), (POKIBRAIN_TASK_PRIORITY - 1),
                       &task_runner_cfg);
    pokibrain_think_now();
    k_timer_start(&timer, K_MSEC(POKIBRAIN_PERIOD_MS), K_MSEC(POKIBRAIN_PERIOD_MS));
    k_timer_start(&match_end_timer, K_SECONDS(GAME_ROUND_TIME_S), K_NO_WAIT);
    brain.start_time_ms = k_uptime_get_32();
}

void pokibrain_think_now(void)
{
    LOG_INF("Think now requested --------------------\n");
    pokibrain_events_t ev = POKIBRAIN_THINK;
    k_msgq_put(&event_queue, &ev, K_NO_WAIT);
}
