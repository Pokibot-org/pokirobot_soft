#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <logging/log.h>
#include "pokibrain.h"

LOG_MODULE_REGISTER(main);

uint8_t pokibrain_task_grab_smth(pokibrain_callback_params_t *params)
{
    LOG_INF("%s",__func__);
    return 0;
}

int32_t pokibrain_reward_calulation_grab_smth(pokibrain_callback_params_t *params)
{
    LOG_INF("%s",__func__);
    return 0;
}

uint8_t pokibrain_completion_grab_smth(pokibrain_callback_params_t *params)
{
    params->world_context->nb_thing_grabbed = MIN(3, params->world_context->nb_thing_grabbed + 1);
    return 0;
}

uint8_t pokibrain_task_put_smth(pokibrain_callback_params_t *params)
{
    LOG_INF("%s",__func__);
    return 0;
}

int32_t pokibrain_reward_calulation_put_smth(pokibrain_callback_params_t *params)
{
    LOG_INF("%s",__func__);
    return 10 * params->world_context->nb_thing_grabbed;
}

uint8_t pokibrain_completion_put_smth(pokibrain_callback_params_t *params)
{
    params->world_context->nb_thing_grabbed = 0;
    return 0;
}

void main(void)
{

    LOG_INF("BOOTING !\n");

    static pokibrain_user_context_t world_context;
    static pokibrain_task_t tasks[] = {
        {
            .task_position = {.angle = 90.0f, .coordinates = {.x = 100, .y = 100}},
            .reward_calculation = pokibrain_reward_calulation_grab_smth,
            .task_process = pokibrain_task_grab_smth,
            .completiton_callback = pokibrain_completion_grab_smth
        },
                {
            .task_position = {.angle = 90.0f, .coordinates = {.x = 100, .y = 100}},
            .reward_calculation = pokibrain_reward_calulation_put_smth,
            .task_process = pokibrain_task_put_smth,
            .completiton_callback = pokibrain_completion_put_smth
        }
        };
    pokibrain_init(tasks, sizeof(tasks)/sizeof(tasks[0]), &world_context);
    pokibrain_start();
    while (1)
    {
        k_sleep(K_MSEC(1000));
    }
}
