#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "control/control.h"
#include "hmi/hmi_led.h"
#include "tirette/tirette.h"
#include "pokutils.h"
#include "strat/strat.h"

LOG_MODULE_REGISTER(main);

void match_init()
{
    LOG_INF("MATCH INIT");
    // hmi_led_init();
    // hmi_led_error();
    static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
    // static const struct gpio_dt_spec sw_side = GPIO_DT_SPEC_GET(DT_ALIAS(sw_side), gpios);
    static const struct gpio_dt_spec sw_power = GPIO_DT_SPEC_GET(DT_ALIAS(sw_power), gpios);

    int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        LOG_ERR("failed to init led");
        goto exit;
    }
    ret = gpio_pin_configure_dt(&sw_power, GPIO_INPUT);
    if (ret < 0) {
        LOG_ERR("failed to init sw_power");
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
    // static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
    // static const struct gpio_dt_spec sw_side = GPIO_DT_SPEC_GET(DT_ALIAS(sw_side), gpios);
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
    // static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
    static const struct gpio_dt_spec sw_side = GPIO_DT_SPEC_GET(DT_ALIAS(sw_side), gpios);
    // static const struct gpio_dt_spec sw_power = GPIO_DT_SPEC_GET(DT_ALIAS(sw_power), gpios);

    match_init();
    match_wait_start();
    LOG_INF("MATCH START");
    int side = gpio_pin_get_dt(&sw_side);
    LOG_INF("side= %d", side);
    enum team_color color = side ? TEAM_COLOR_GREEN : TEAM_COLOR_BLUE;
    strat_init(color);
    k_sleep(K_MSEC(500)); // delay before going away not to let the key halfway in
    shared_ctrl.start = true;

    strat_run();

    k_sleep(K_FOREVER);
    return;
}

#include "strat/strat_interface.h"

int main(void)
{
    LOG_INF("BOOTING");

    // _test_gconf();
    // _test_motor_cmd();
    // _test_target();
    // _test_calibration_distance();
    // _test_calibration_angle();
    // _test_calibration_mix();
    // _test_connerie();
    // _test_pathfinding();

    // match_1();
    shared_ctrl.start_init = true;

    control_task_wait_ready();

    shared_ctrl.start = true;

    pos2_t path_pos[] = {
        {.x = 0, .y = 1000, .a = 0},
        {.x = 1000, .y = 1000, .a = 0},
        {.x = 1000, .y = 0, .a = 0},
        {.x = 0, .y = 0, .a = 0},
    };
    strat_set_waypoints(path_pos, 4);
    strat_wait_target(STRAT_PLANAR_TARGET_SENSITIVITY_DEFAULT,
                      STRAT_ANGULAR_TARGET_SENSITIVITY_DEFAULT, 50000);
    LOG_ERR("HAAAAAAAAAAAAAAAAAAAAAA");
    return 0;
}
