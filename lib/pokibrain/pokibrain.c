#include "pokibrain.h"

#include <zephyr.h>

#include <logging/log.h>
#include <timing/timing.h>

LOG_MODULE_REGISTER(pokibrain, POKIBRAIN_LOG_LEVEL);

// DEFINES ---------------------------------------------------------------
#define GAME_ROUND_TIME_S 10

// TYPES -----------------------------------------------------------------
typedef enum { POKIBRAIN_THINK, POKIBRAIN_DIE } pokibrain_events_t;

typedef struct {
    uint32_t number_of_tasks;
    pokibrain_task_t* tasks;
    struct k_thread child_thread;
    struct k_thread thread;
    k_tid_t thread_id;
    k_tid_t child_thread_id;
    uint32_t current_score;
    pokibrain_user_context_t* world_context;
    pokibrain_user_context_t world_context_save[POKIBRAIN_DEEPTH];
} pokibrain_t;

// FUNC ------------------------------------------------------------------
static void pokibrain_periodic_timer(struct k_timer* timer_id);
static void pokibrain_match_timer(struct k_timer* timer_id);

// DECLARATIONS ----------------------------------------------------------
static pokibrain_t brain;
K_THREAD_STACK_DEFINE(child_stack, POKIBRAIN_CHILD_TASK_SIZE);
K_THREAD_STACK_DEFINE(stack, POKIBRAIN_STACK_SIZE);
K_MSGQ_DEFINE(event_queue, sizeof(pokibrain_events_t), 4, 4);
K_TIMER_DEFINE(timer, pokibrain_periodic_timer, NULL);
K_TIMER_DEFINE(match_end_timer, pokibrain_match_timer, NULL);

// IMPLEMENTATION --------------------------------------------------------
static void pokibrain_match_timer(struct k_timer* timer_id) {
    pokibrain_events_t ev = POKIBRAIN_DIE;
    k_msgq_put(&event_queue, &ev, K_NO_WAIT);
}

static void pokibrain_periodic_timer(struct k_timer* timer_id) {
    pokibrain_events_t ev = POKIBRAIN_THINK;
    k_msgq_put(&event_queue, &ev, K_NO_WAIT);
}

void pokibrain_get_best_task(uint8_t deepth, pokibrain_task_t* best_task, uint32_t* best_task_score) {
}

void pokibrain_think(void) {
    LOG_DBG("Think");
    uint32_t best_score = 0;
    pokibrain_task_t* best_task = NULL;
    pokibrain_callback_params_t params = {.robot_position = {.x = 0, .y = 0, .a = 0},
                                          .time = 0, // TODO: TIMER
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

    params.task_position = best_task->task_position;
    best_task->completiton_callback(&params);
}

void pokibrain_task(void* arg1, void* arg2, void* arg3) {
    soc_timing_init();
    timing_t start_time, end_time;
    uint64_t total;
    while (1) {
        pokibrain_events_t ev;
        k_msgq_get(&event_queue, &ev, K_FOREVER);

        switch (ev) {
        case POKIBRAIN_THINK:
            soc_timing_start();
            start_time = soc_timing_counter_get();
            pokibrain_think();
            end_time = soc_timing_counter_get();
            soc_timing_stop();
            total = soc_timing_cycles_get(&start_time, &end_time);
            total = soc_timing_cycles_to_ns(total);
            LOG_INF("Thinking took %llu ms/%llu us", total / 1000, total);
            break;
        case POKIBRAIN_DIE:
            LOG_DBG("Die");
            k_timer_stop(&timer);
            k_msgq_purge(&event_queue);
            break;
        default:
            break;
        }
    }
}

void pokibrain_init(pokibrain_task_t* tasks, uint32_t number_of_tasks, pokibrain_user_context_t* world_context) {
    brain.tasks = tasks;
    brain.number_of_tasks = number_of_tasks;
    brain.world_context = world_context;
    brain.thread_id = k_thread_create(&brain.thread, stack, POKIBRAIN_STACK_SIZE, pokibrain_task, NULL, NULL, NULL,
                                      POKIBRAIN_TASK_PRIORITY, 0, K_NO_WAIT);
    LOG_INF("Brain initialized");
}

void pokibrain_start(void) {
    LOG_INF("Brain started");
    k_timer_start(&timer, K_MSEC(POKIBRAIN_PERIOD_MS), K_MSEC(POKIBRAIN_PERIOD_MS));
    k_timer_start(&match_end_timer, K_SECONDS(GAME_ROUND_TIME_S), K_NO_WAIT);
}

void pokibrain_think_now(void) {
    LOG_INF("Think now requested");
    pokibrain_events_t ev = POKIBRAIN_THINK;
    k_msgq_put(&event_queue, &ev, K_NO_WAIT);
}
