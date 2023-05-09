#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include "pokibrain/pokibrain.h"

LOG_MODULE_REGISTER(main);

struct pokibrain_user_context {
	uint8_t nb_thing_grabbed;
};

uint8_t pokibrain_task_grab_smth(struct pokibrain_callback_params *params)
{
	LOG_INF("RUNNING %s", __func__);
	return 0;
}

int32_t pokibrain_reward_calculation_grab_smth(struct pokibrain_callback_params *params)
{
	LOG_INF("REWARD CALC %s", __func__);
	return 0;
}

uint8_t pokibrain_completion_grab_smth(struct pokibrain_callback_params *params)
{
	LOG_INF("COMPLETION %s", __func__);

	struct pokibrain_user_context *ctx = params->world_context;

	ctx->nb_thing_grabbed = MIN(3, ctx->nb_thing_grabbed + 1);
	return 0;
}

uint8_t pokibrain_task_put_smth(struct pokibrain_callback_params *params)
{
	LOG_INF("RUNNING %s", __func__);
	return 0;
}

int32_t pokibrain_reward_calculation_put_smth(struct pokibrain_callback_params *params)
{
	LOG_INF("REWARD CALC %s", __func__);
	struct pokibrain_user_context *ctx = params->world_context;
	return 10 * ctx->nb_thing_grabbed;
}

uint8_t pokibrain_completion_put_smth(struct pokibrain_callback_params *params)
{
	LOG_INF("COMPLETION %s", __func__);
	struct pokibrain_user_context *ctx = params->world_context;
	ctx->nb_thing_grabbed = 0;
	return 0;
}

void main(void)
{

	LOG_INF("BOOTING !\n");

	static struct pokibrain_user_context world_context;
	static struct pokibrain_task tasks[] = {
		{.task_position = {.a = 90.0f, .x = 100, .y = 100},
		 .reward_calculation = pokibrain_reward_calculation_grab_smth,
		 .task_process = pokibrain_task_grab_smth,
		 .completion_callback = pokibrain_completion_grab_smth},
		{.task_position = {.a = 90.0f, .x = 100, .y = 100},
		 .reward_calculation = pokibrain_reward_calculation_put_smth,
		 .task_process = pokibrain_task_put_smth,
		 .completion_callback = pokibrain_completion_put_smth}};
	pokibrain_init(tasks, sizeof(tasks) / sizeof(tasks[0]), &world_context,
				   sizeof(struct pokibrain_user_context), NULL);
	pokibrain_start();
	while (1) {
		k_sleep(K_MSEC(1000));
	}
}
