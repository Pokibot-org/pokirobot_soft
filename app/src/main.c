#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "pokarm/pokarm.h"
#include "control/control.h"
#include "hmi/hmi_led.h"
#include "tirette/tirette.h"
#include "pokutils.h"
#include "strat/strat.h"
#include "pokstick/pokstick.h"
#include "pokpush/pokpush.h"
#include "pokuicom/pokuicom.h"

LOG_MODULE_REGISTER(main);

void hardware_init(void)
{
    LOG_INF("MATCH INIT");
    static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
    // static const struct gpio_dt_spec sw_side = GPIO_DT_SPEC_GET(DT_ALIAS(sw_side), gpios);
    static const struct gpio_dt_spec sw_power = GPIO_DT_SPEC_GET(DT_ALIAS(sw_power), gpios);
    
    pokstick_retract();
    pokpush_retract();

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

    LOG_INF("WAIT FOR TASKS AND POWER");
    while (!gpio_pin_get_dt(&sw_power)) {
        k_sleep(K_MSEC(1));
    }

    LOG_INF("POWER IS UP!");
    shared_ctrl.start_init = true;

    LOG_INF("MATCH WAIT FOR TASKS");
    control_task_wait_ready();

    k_sleep(K_MSEC(500));
    hmi_led_success();
exit:
    return;
}

void match(void)
{
    hardware_init();

    LOG_INF("MATCH WAIT START");

    strat_run();

    k_sleep(K_FOREVER);
    return;
}

#include "strat/strat_interface.h"
#include "nav/nav.h"

int main(void)
{
    LOG_INF("BOOTING");

    // _test_gconf();
    // _test_motor_cmd();
    // _test_target();
    // _test_calibration_distance();
    // _test_calibration_angle();
    // _test_calibration_mix();
    // _test_drawing();
    // _test_connerie();
    // _test_pathfinding();
    // _test_pathfinding();
    // _test_pokarm_stepper();

    match();
    // nav_init();
    // shared_ctrl.start_init = true;

    // control_task_wait_ready();

    // control_set_pos(&shared_ctrl, (pos2_t){.x = 200, .y = 200, .a = -M_PI_2});
    // shared_ctrl.start = true;

    // nav_go_to_with_pathfinding((pos2_t){.x = 2000 - 200, .y = 200, .a = -M_PI_2}, NULL, 0);

    // while (1) {
    //     k_sleep(K_SECONDS(1));
    // }

    // LOG_ERR("HAAAAAAAAAAAAAAAAAAAAAA");
    return 0;
}
