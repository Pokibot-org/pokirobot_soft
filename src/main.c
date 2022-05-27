#include <device.h>
#include <devicetree.h>
#include <zephyr.h>

#include "control/control.h"
#include "figurine_lifter/figurine_lifter.h"
#include "hmi/hmi_led.h"
#include "kernel.h"
#include "nav/obstacle_manager.h"
#include "nav/path_manager.h"
#include "pokarm/pokarm.h"
#include "pokibrain/pokibrain.h"
#include "shared.h"
#include "tirette/tirette.h"
#include "tmc2209/tmc2209.h"
#include "utils.h"
#include "coords.h"
#include <drivers/gpio.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(main);

// #error on callback before decimation check collisions

void collision_callback(bool collision) {
    if (collision) {
        LOG_INF("Collision detected");
    }
    shared_ctrl.brake = collision;
}

void end_game_callback(void) {
    LOG_INF("MATCH IS OVER");
    shared_ctrl.brake = true;
    pokarm_up();
    k_sleep(K_MSEC(100));
    k_sched_lock();
    while (1) {
    }
}

void match_1() {
    LOG_INF("MATCH INIT");
    // hmi_led_init();
    // hmi_led_error();
    static const struct gpio_dt_spec led =
        GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
    static const struct gpio_dt_spec sw_side =
        GPIO_DT_SPEC_GET(DT_ALIAS(sw_side), gpios);
    static const struct gpio_dt_spec sw_power =
        GPIO_DT_SPEC_GET(DT_ALIAS(sw_power), gpios);

    obstacle_manager_init(collision_callback);
    int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        LOG_ERR("failed to init led");
        goto exit;
    }
    ret = gpio_pin_configure_dt(&sw_power, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("failed to init led");
        goto exit;
    }
    if (shared_init()) {
        LOG_ERR("failed to init shared objects");
        ret = -1;
        goto exit;
    }
    if (tirette_init()) {
        LOG_ERR("failed to init tirette");
        ret = -1;
        goto exit;
    }
    if (pokarm_init()) {
        LOG_ERR("failed to init pokarm");
        ret = -1;
        goto exit;
    }
    if (figurine_lifter_init()) {
        LOG_ERR("failed to init figurine_lifter");
        ret = -1;
        goto exit;
    }
    if (!device_is_ready(led.port)) {
        LOG_ERR("failed to init led");
        goto exit;
    }
    pokarm_up();
    figurine_lifter_up_inside();
    pokibrain_init(NULL, 0, NULL, end_game_callback);
    LOG_INF("MATCH INIT DONE");
    LOG_INF("MATCH WAIT FOR TASKS AND POWER");

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
    LOG_INF("MATCH START");
    pokibrain_start();
    k_sleep(
        K_MSEC(1000)); // delay before going away not to let the key halfway in
    shared_ctrl.start = true;
    int side = gpio_pin_get_dt(&sw_side);
    LOG_DBG("side= %d", side);

    LOG_DBG("go to target 1");
    pos2_t dst_1 = {770.0f, 350.0f, 0.5f * M_PI};
    if (side == SIDE_YELLOW) {
        dst_1.x = -dst_1.x;
        dst_1.a = -dst_1.a;
    }
    control_set_target(&shared_ctrl, dst_1);
    for (int i = 0; i < 200; i++) {
        gpio_pin_toggle(led.port, led.pin);
        k_sleep(K_MSEC(100));
    }

    LOG_DBG("go to target 2");
    pos2_t dst_2 = {770.0f, 640.0f, 0.5f * M_PI};
    if (side == SIDE_YELLOW) {
        dst_2.x = -dst_2.x;
        dst_2.a = -dst_2.a;
    }
    control_set_target(&shared_ctrl, dst_2);
    for (int i = 0; i < 40; i++) {
        gpio_pin_toggle(led.port, led.pin);
        k_sleep(K_MSEC(100));
    }
    LOG_DBG("sending pokarm out");
    pokarm_pos_put_haxagone_display();
    k_sleep(K_MSEC(1000));

    LOG_DBG("go to target 3");
    pos2_t dst_3 = {950.0f, 700.0f, 0.5f * M_PI};
    // pos2_t dst_3 = {0.0f, 0.0f, 100.0f * M_PI};
    if (side == SIDE_YELLOW) {
        dst_3.x = -dst_3.x;
        dst_3.a = -dst_3.a;
    }
    control_set_target(&shared_ctrl, dst_3);
    for (int i = 0; i < 200; i++) {
        gpio_pin_toggle(led.port, led.pin);
        k_sleep(K_MSEC(100));
    }
exit:
    LOG_INF("MATCH DONE (ret: %d)", ret);
    return;
}

void match_2() {
    LOG_INF("MATCH INIT");
    // hmi_led_init();
    // hmi_led_error();
    static const struct gpio_dt_spec led =
        GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
    static const struct gpio_dt_spec sw_side =
        GPIO_DT_SPEC_GET(DT_ALIAS(sw_side), gpios);
    static const struct gpio_dt_spec sw_power =
        GPIO_DT_SPEC_GET(DT_ALIAS(sw_power), gpios);

    obstacle_manager_init(collision_callback);
    int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        LOG_ERR("failed to init led");
        goto exit;
    }
    ret = gpio_pin_configure_dt(&sw_power, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("failed to init led");
        goto exit;
    }
    if (shared_init()) {
        LOG_ERR("failed to init shared objects");
        ret = -1;
        goto exit;
    }
    if (tirette_init()) {
        LOG_ERR("failed to init tirette");
        ret = -1;
        goto exit;
    }
    if (pokarm_init()) {
        LOG_ERR("failed to init pokarm");
        ret = -1;
        goto exit;
    }
    if (figurine_lifter_init()) {
        LOG_ERR("failed to init figurine_lifter");
        ret = -1;
        goto exit;
    }
    if (!device_is_ready(led.port)) {
        LOG_ERR("failed to init led");
        goto exit;
    }
    pokarm_up();
    figurine_lifter_up_inside();
    pokibrain_init(NULL, 0, NULL, end_game_callback);
    LOG_INF("MATCH INIT DONE");
    LOG_INF("MATCH WAIT FOR TASKS AND POWER");

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
    LOG_INF("MATCH START");
    pokibrain_start();
    k_sleep(
        K_MSEC(1000)); // delay before going away not to let the key halfway in
    shared_ctrl.start = true;
    int side = gpio_pin_get_dt(&sw_side);
    LOG_DBG("side= %d", side);

    LOG_DBG("go to target 1");
    pos2_t dst_1 = {770.0f, 350.0f, 0.5f * M_PI};
    if (side == SIDE_YELLOW) {
        dst_1.x = -dst_1.x;
        dst_1.a = -dst_1.a;
    }
    control_set_target(&shared_ctrl, dst_1);
    for (int i = 0; i < 100; i++) {
        gpio_pin_toggle(led.port, led.pin);
        k_sleep(K_MSEC(100));
    }

    LOG_DBG("go to target 2");
    pos2_t dst_2 = {770.0f, 640.0f, 0.5f * M_PI};
    if (side == SIDE_YELLOW) {
        dst_2.x = -dst_2.x;
        dst_2.a = -dst_2.a;
    }
    control_set_target(&shared_ctrl, dst_2);
    for (int i = 0; i < 40; i++) {
        gpio_pin_toggle(led.port, led.pin);
        k_sleep(K_MSEC(100));
    }
    LOG_DBG("sending pokarm out");
    pokarm_pos_put_haxagone_display();
    k_sleep(K_MSEC(1000));

    LOG_DBG("go to target 3");
    pos2_t dst_3 = {955.0f, 700.0f, 0.5f * M_PI};
    if (side == SIDE_YELLOW) {
        dst_3.x = -dst_3.x;
        dst_3.a = -dst_3.a;
    }
    control_set_target(&shared_ctrl, dst_3);
    for (int i = 0; i < 100; i++) {
        gpio_pin_toggle(led.port, led.pin);
        k_sleep(K_MSEC(100));
    }

    LOG_DBG("go back to target 2");
    pokarm_up();
    control_set_target(&shared_ctrl, dst_2);
    for (int i = 0; i < 100; i++) {
        gpio_pin_toggle(led.port, led.pin);
        k_sleep(K_MSEC(100));
    }

    LOG_DBG("go back to target 1");
    control_set_target(&shared_ctrl, dst_1);
    for (int i = 0; i < 100; i++) {
        gpio_pin_toggle(led.port, led.pin);
        k_sleep(K_MSEC(100));
    }

    LOG_DBG("go back to home");
    pos2_t dst_home = {-60.0f, 0.0f, 0.0f * M_PI};
    if (side == SIDE_YELLOW) {
        dst_home.x = -dst_home.x;
        dst_home.a = -dst_home.a;
    }
    control_set_target(&shared_ctrl, dst_home);
    for (int i = 0; i < 100; i++) {
        gpio_pin_toggle(led.port, led.pin);
        k_sleep(K_MSEC(100));
    }
    shared_ctrl.brake = true;
    k_sleep(K_MSEC(100));
    k_sched_lock();
    while (1) {
    }
exit:
    LOG_INF("MATCH DONE (ret: %d)", ret);
    return;
}

void match_3() {
    LOG_INF("MATCH INIT");
    // hmi_led_init();
    // hmi_led_error();
    static const struct gpio_dt_spec led =
        GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
    static const struct gpio_dt_spec sw_side =
        GPIO_DT_SPEC_GET(DT_ALIAS(sw_side), gpios);
    static const struct gpio_dt_spec sw_power =
        GPIO_DT_SPEC_GET(DT_ALIAS(sw_power), gpios);

    obstacle_manager_init(collision_callback);
    int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        LOG_ERR("failed to init led");
        goto exit;
    }
    ret = gpio_pin_configure_dt(&sw_power, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("failed to init led");
        goto exit;
    }
    if (shared_init()) {
        LOG_ERR("failed to init shared objects");
        ret = -1;
        goto exit;
    }
    if (tirette_init()) {
        LOG_ERR("failed to init tirette");
        ret = -1;
        goto exit;
    }
    if (pokarm_init()) {
        LOG_ERR("failed to init pokarm");
        ret = -1;
        goto exit;
    }
    if (figurine_lifter_init()) {
        LOG_ERR("failed to init figurine_lifter");
        ret = -1;
        goto exit;
    }
    if (!device_is_ready(led.port)) {
        LOG_ERR("failed to init led");
        goto exit;
    }
    pokarm_up();
    figurine_lifter_up_inside();
    pokibrain_init(NULL, 0, NULL, end_game_callback);
    LOG_INF("MATCH INIT DONE");
    LOG_INF("MATCH WAIT FOR TASKS AND POWER");

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
    LOG_INF("MATCH START");
    pokibrain_start();
    k_sleep(
        K_MSEC(1000)); // delay before going away not to let the key halfway in
    shared_ctrl.start = true;
    int side = gpio_pin_get_dt(&sw_side);
    LOG_DBG("side= %d", side);

    pos2_t start_pos = {1000.0f, 1500.0f - 155.0f, M_PI / 2};
    if (side == SIDE_YELLOW) {
        start_pos.y = -start_pos.y;
        start_pos.a = -start_pos.a;
    }
    control_set_pos(&shared_ctrl, start_pos);

    LOG_DBG("go to target 1");
    pos2_t dst_1 = {770.0f, 350.0f, 0.5f * M_PI};
    if (side == SIDE_YELLOW) {
        dst_1.x = -dst_1.x;
        dst_1.a = -dst_1.a;
    }
    control_set_target(&shared_ctrl, dst_1);
    for (int i = 0; i < 100; i++) {
        gpio_pin_toggle(led.port, led.pin);
        k_sleep(K_MSEC(100));
    }

    LOG_DBG("go to target 2");
    pos2_t dst_2 = {770.0f, 640.0f, 0.5f * M_PI};
    if (side == SIDE_YELLOW) {
        dst_2.x = -dst_2.x;
        dst_2.a = -dst_2.a;
    }
    control_set_target(&shared_ctrl, dst_2);
    for (int i = 0; i < 40; i++) {
        gpio_pin_toggle(led.port, led.pin);
        k_sleep(K_MSEC(100));
    }
    LOG_DBG("sending pokarm out");
    pokarm_pos_put_haxagone_display();
    k_sleep(K_MSEC(1000));

    LOG_DBG("go to target 3");
    pos2_t dst_3 = {955.0f, 700.0f, 0.5f * M_PI};
    if (side == SIDE_YELLOW) {
        dst_3.x = -dst_3.x;
        dst_3.a = -dst_3.a;
    }
    control_set_target(&shared_ctrl, dst_3);
    for (int i = 0; i < 100; i++) {
        gpio_pin_toggle(led.port, led.pin);
        k_sleep(K_MSEC(100));
    }

    LOG_DBG("go back to target 2");
    pokarm_up();
    control_set_target(&shared_ctrl, dst_2);
    for (int i = 0; i < 100; i++) {
        gpio_pin_toggle(led.port, led.pin);
        k_sleep(K_MSEC(100));
    }

    LOG_DBG("go back to target 1");
    control_set_target(&shared_ctrl, dst_1);
    for (int i = 0; i < 100; i++) {
        gpio_pin_toggle(led.port, led.pin);
        k_sleep(K_MSEC(100));
    }

    LOG_DBG("go back to home");
    pos2_t dst_home = {-60.0f, 0.0f, 0.0f * M_PI};
    if (side == SIDE_YELLOW) {
        dst_home.x = -dst_home.x;
        dst_home.a = -dst_home.a;
    }
    control_set_target(&shared_ctrl, dst_home);
    for (int i = 0; i < 100; i++) {
        gpio_pin_toggle(led.port, led.pin);
        k_sleep(K_MSEC(100));
    }
    shared_ctrl.brake = true;
    k_sleep(K_MSEC(100));
    k_sched_lock();
    while (1) {
    }
exit:
    LOG_INF("MATCH DONE (ret: %d)", ret);
    return;
}


#define MAX_PATH_SIZE 1000
point2_t path[MAX_PATH_SIZE];
uint16_t path_size = 0;
void path_found_clbk(const path_node_t* node, void* user_data) {
    path_size = path_manager_retrieve_path(path, MAX_PATH_SIZE, NULL, node);
    if (path_size == 0)
    {
        LOG_ERR("Path not correctly retrived");
    }
}

void _test_pathfinding() {
    LOG_INF("MATCH INIT");
    // hmi_led_init();
    // hmi_led_error();
    static const struct gpio_dt_spec led =
        GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
    static const struct gpio_dt_spec sw_side =
        GPIO_DT_SPEC_GET(DT_ALIAS(sw_side), gpios);
    static const struct gpio_dt_spec sw_power =
        GPIO_DT_SPEC_GET(DT_ALIAS(sw_power), gpios);

    obstacle_manager_init(collision_callback);
    int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        LOG_ERR("failed to init led");
        goto exit;
    }
    ret = gpio_pin_configure_dt(&sw_power, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("failed to init led");
        goto exit;
    }
    if (shared_init()) {
        LOG_ERR("failed to init shared objects");
        ret = -1;
        goto exit;
    }
    if (tirette_init()) {
        LOG_ERR("failed to init tirette");
        ret = -1;
        goto exit;
    }
    if (pokarm_init()) {
        LOG_ERR("failed to init pokarm");
        ret = -1;
        goto exit;
    }
    if (figurine_lifter_init()) {
        LOG_ERR("failed to init figurine_lifter");
        ret = -1;
        goto exit;
    }
    if (!device_is_ready(led.port)) {
        LOG_ERR("failed to init led");
        goto exit;
    }
    pokarm_up();
    figurine_lifter_up_inside();
    pokibrain_init(NULL, 0, NULL, end_game_callback);
    LOG_INF("MATCH INIT DONE");
    LOG_INF("MATCH WAIT FOR TASKS AND POWER");

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
    LOG_INF("MATCH START");
    pokibrain_start();
    k_sleep(K_MSEC(1000)); // delay before going away not to let the key halfway in
    int side = gpio_pin_get_dt(&sw_side);
    LOG_DBG("side= %d", side);

    pos2_t start_pos = {1000.0f, 1500.0f - 155.0f, M_PI / 2};
    pos2_t end_pos;
    end_pos.y = -start_pos.y;
    end_pos.x = start_pos.x;
    end_pos.a = -start_pos.a;

    control_set_pos(&shared_ctrl, start_pos);
    shared_ctrl.start = true;

    point2_t start;
    start.x = start_pos.x;
    start.y = start_pos.y;

    point2_t end;
    end.x = end_pos.x;
    end.y = end_pos.y;

    path_manager_config_t path_config;
    path_config.found_path_clbk = path_found_clbk;
    LOG_INF("Lauching pathfinding");
    uint8_t err = path_manager_find_path(start, end, path_config);
    if (err)
    {
        LOG_ERR("problem when launching pathfinding %u", err);
        goto exit;
    }
    LOG_INF("Waiting for path to be found");
    while (path_size == 0)
    {
        k_sleep(K_MSEC(100));
    }
    LOG_INF("Path found of size %u", path_size);

    for (int16_t i = path_size - 1; i >= 0; i--)
    {
        pos2_t next_pos = start_pos;
        next_pos.x = path[i].x;
        next_pos.y = path[i].y;
        LOG_INF("Go to point %d, x: %f, y: %f", i, next_pos.x, next_pos.y);
        control_set_target(&shared_ctrl, next_pos);
        k_sleep(K_MSEC(5000));
    }
    
exit:
    LOG_INF("MATCH DONE (ret: %d)", ret);
    return;
}


int main(void) {
    LOG_INF("BOOTING");
    int ret = 0;

    // obstacle_manager_init(collision_callback);

    // pokarm_test();
    // wait for init

    // main thread

    // _test_gconf();
    // _test_motor_cmd();
    // _test_target();
    // _test_calibration();
    // _test_connerie();
    // match_1();
    // match_2();
    // match_3();

    _test_pathfinding();
exit:
    return ret;
}
