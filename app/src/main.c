#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

#include "control/control.h"
#include "coords.h"
#include "hmi/hmi_led.h"
#include <zephyr/kernel.h>
#include "lidar/camsense_x1/camsense_x1.h"
#include "nav/obstacle_manager.h"
#include "nav/path_manager.h"
#include "pokibrain/pokibrain.h"
#include "shared.h"
#include "tirette/tirette.h"
#include "tmc2209/tmc2209.h"
#include "pokutils.h"
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main);

// #error on callback before decimation check collisions

void collision_callback(bool collision)
{
	if (collision) {
		LOG_INF("Collision detected");
	}
	shared_ctrl.brake = collision;
}

void end_game_callback(void)
{
	LOG_INF("MATCH IS OVER");
	camsense_x1_kill();
	obstacle_manager_kill();
	shared_ctrl.brake = true;
	pokarm_up();
	control_force_motor_stop();
	k_sched_lock();
	while (1) {
	}
}

#define MAX_PATH_SIZE 1000
point2_t path[MAX_PATH_SIZE];
uint16_t path_size = 0;
void path_found_clbk(const path_node_t *node, void *user_data)
{
	path_size = path_manager_retrieve_path(path, MAX_PATH_SIZE, NULL, node);
	if (path_size == 0) {
		LOG_ERR("Path not correctly retrived");
	}
}

int go_to_with_pathfinding(pos2_t end_pos)
{
	LOG_INF("Lauching pathfinding");
	path_manager_config_t path_config;
	path_config.found_path_clbk = path_found_clbk;
	path_config.nb_node_optimisation = 50;
	path_size = 0;
	pos2_t start_pos;
	control_get_pos(&shared_ctrl, &start_pos);
	point2_t start;
	start.x = start_pos.x;
	start.y = start_pos.y;

	point2_t end;
	end.x = end_pos.x;
	end.y = end_pos.y;

	LOG_INF("From x:%f, y:%f |To x:%f, y:%f", start.x, start.y, end.x, end.y);

	uint8_t err = path_manager_find_path(start, end, path_config);
	if (err) {
		LOG_ERR("problem when launching pathfinding %u", err);
		return -1;
	}
	LOG_INF("Waiting for path to be found");
	for (size_t i = 0; i < 20000; i++) {
		k_sleep(K_MSEC(1));
		if (path_size != 0) {
			break;
		}
	}
	if (path_size == 0) {
		LOG_WRN("No path found");
		return -2;
	}

	LOG_INF("Path found of size %u", path_size);
	for (int16_t i = path_size - 1; i >= 0; i--) {
		pos2_t next_pos;
		next_pos.a = start_pos.a;
		next_pos.x = path[i].x;
		next_pos.y = path[i].y;
		LOG_INF("Go to point %d, x: %f, y: %f", i, next_pos.x, next_pos.y);
		control_set_target(&shared_ctrl, next_pos);
		control_task_wait_target(100.0f, M_PI / 4.0f, 10000);
	}
	control_task_wait_target(CONTROL_PLANAR_TARGET_SENSITIVITY_DEFAULT,
							 CONTROL_ANGULAR_TARGET_SENSITIVITY_DEFAULT, 10000);
	return 0;
}

void match_init()
{
	LOG_INF("MATCH INIT");
	// hmi_led_init();
	// hmi_led_error();
	static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
	static const struct gpio_dt_spec sw_side = GPIO_DT_SPEC_GET(DT_ALIAS(sw_side), gpios);
	static const struct gpio_dt_spec sw_power = GPIO_DT_SPEC_GET(DT_ALIAS(sw_power), gpios);

	obstacle_manager_init(collision_callback);
	int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("failed to init led");
		goto exit;
	}
	ret = gpio_pin_configure_dt(&sw_power, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("failed to init sw_poser");
		goto exit;
	}
	if (tirette_init()) {
		LOG_ERR("failed to init tirette");
		ret = -1;
		goto exit;
	}
	if (!device_is_ready(led.port)) {
		LOG_ERR("failed to init led");
		goto exit;
	}

	// init strat

	LOG_INF("MATCH INIT DONE");
exit:
	return;
}

void match_wait_start()
{
	LOG_INF("MATCH WAIT FOR TASKS AND POWER");
	static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
	static const struct gpio_dt_spec sw_side = GPIO_DT_SPEC_GET(DT_ALIAS(sw_side), gpios);
	static const struct gpio_dt_spec sw_power = GPIO_DT_SPEC_GET(DT_ALIAS(sw_power), gpios);
	while (!gpio_pin_get_dt(&sw_power)) {
		k_sleep(K_MSEC(1));
	}
	LOG_INF("POWER IS UP!");
	shared_ctrl.start_init = true;

	LOG_INF("MATCH WAIT FOR TASKS");
	control_task_wait_ready();

	k_sleep(K_MSEC(500));
	hmi_led_success();

	LOG_INF("MATCH WAIT FOR STARTER KEY");
	tirette_wait_until_released();
}

void match_1()
{
	int ret = 0;
	static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
	static const struct gpio_dt_spec sw_side = GPIO_DT_SPEC_GET(DT_ALIAS(sw_side), gpios);
	static const struct gpio_dt_spec sw_power = GPIO_DT_SPEC_GET(DT_ALIAS(sw_power), gpios);

	match_init();
	match_wait_start();

	LOG_INF("MATCH START");
	pokibrain_start();
	int side = gpio_pin_get_dt(&sw_side);
	LOG_INF("side= %d", side);
	pos2_t start_pos = TRANSFORM_SIDE(side, COORDS_START);
	pos2_t target = start_pos;
	bool at_target = false;
	control_set_pos(&shared_ctrl, start_pos);
	control_set_target(&shared_ctrl, start_pos);

	k_sleep(K_MSEC(500)); // delay before going away not to let the key halfway in
	shared_ctrl.start = true;

	LOG_INF("go to carre de fouille");
	target = TRANSFORM_SIDE(side, COORDS_CARREFOUILLE_B1);
	control_set_target(&shared_ctrl, target);
	at_target = control_task_wait_target(30.0f, DEG_TO_RAD(10.0f), 20000);
	if (!at_target) {
		LOG_INF("abort carrefouille b1");
		goto end_carrefouille;
	}
	target = TRANSFORM_SIDE(side, COORDS_CARREFOUILLE_B2);
	control_set_target(&shared_ctrl, target);
	at_target = control_task_wait_target(30.0f, DEG_TO_RAD(10.0f), 10000);
	if (!at_target) {
		LOG_INF("abort carrefouille b2");
		goto end_carrefouille;
	}
	target = TRANSFORM_SIDE(side, COORDS_CARREFOUILLE_B3);
	control_set_target(&shared_ctrl, target);
	at_target = control_task_wait_target(30.0f, DEG_TO_RAD(10.0f), 10000);
	if (!at_target) {
		LOG_INF("abort carrefouille b3");
		goto end_carrefouille;
	}
	pokarm_pos_put_haxagone_display();
	k_sleep(K_MSEC(500));
	target = TRANSFORM_SIDE(side, COORDS_CARREFOUILLE);
	control_set_target(&shared_ctrl, target);
	at_target = control_task_wait_target_default(10000);
	if (!at_target) {
		LOG_INF("abort carrefouille target");
		goto end_carrefouille;
	}
end_carrefouille:
	pokarm_up();
	k_sleep(K_MSEC(500));
	target = TRANSFORM_SIDE(side, COORDS_CARREFOUILLE_A1);
	control_set_target(&shared_ctrl, target);
	at_target = control_task_wait_target_default(10000);
	if (!at_target) {
		LOG_INF("abort carrefouille a1");
	}

	LOG_INF("go to statuette");
	target = TRANSFORM_SIDE(side, COORDS_STATUETTE);
	if (side == SIDE_YELLOW) {
		target.a -= 2.0f / 3.0f * M_PI;
	}
	control_set_target(&shared_ctrl, target);
	at_target = control_task_wait_target_default(20000);
	if (!at_target) {
		LOG_INF("abort statuette target");
		goto end_statuette;
	}
	figurine_lifter_grab();
	k_sleep(K_MSEC(500));
	LOG_INF("go to vitrine");
	target = TRANSFORM_SIDE(side, COORDS_VITRINE_B1);
	if (side == SIDE_YELLOW) {
		target.a -= 2.0f / 3.0f * M_PI;
	}
	control_set_target(&shared_ctrl, target);
	at_target = control_task_wait_target_default(30000);
	if (!at_target) {
		LOG_INF("abort statuette target");
		goto end_statuette;
	}
	target = TRANSFORM_SIDE(side, COORDS_VITRINE);
	if (side == SIDE_YELLOW) {
		target.a -= 2.0f / 3.0f * M_PI;
	}
	control_set_target(&shared_ctrl, target);
	at_target = control_task_wait_target_default(10000);
	if (!at_target) {
		LOG_INF("abort statuette target");
		goto end_statuette;
	}
end_statuette:
	figurine_lifter_put();
	k_sleep(K_MSEC(500));
	figurine_lifter_up_inside();
	k_sleep(K_MSEC(500));

	LOG_INF("go to home");
	target = TRANSFORM_SIDE(side, COORDS_HOME);
	control_set_target(&shared_ctrl, target);
	at_target = control_task_wait_target(50.0f, DEG_TO_RAD(20.0f), 20000);

exit:
	end_game_callback();
	LOG_INF("MATCH DONE (ret: %d)", ret);
	return;
}

int main(void)
{
	LOG_INF("BOOTING");
	int ret = 0;

	// obstacle_manager_init(collision_callback);

	// pokarm_test();
	// wait for init

	// main thread

	// _test_gconf();
	// _test_motor_cmd();
	// _test_target();
	// _test_calibration_distance();
	// _test_calibration_angle();
	// _test_calibration_mix();
	// _test_connerie();
	// _test_pathfinding();

	match_1();

	// test_match();

exit:
	return ret;
}
