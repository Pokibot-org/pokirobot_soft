#include <device.h>
#include <devicetree.h>
#include <zephyr.h>

#include "control/control.h"
#include "figurine_lifter/figurine_lifter.h"
#include "hmi/hmi_led.h"
#include "kernel.h"
#include "nav/obstacle_manager.h"
#include "pokarm/pokarm.h"
#include "shared.h"
#include "tirette/tirette.h"
#include "tmc2209/tmc2209.h"
#include "utils.h"
#include <drivers/gpio.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(main);

// #error on callback before decimation check collisions

void collision_callback(bool collision) {
    if (collision)
        LOG_INF("Collision detected");
    shared_ctrl.brake = collision;
}


void match() {
    LOG_INF("MATCH INIT");
    // hmi_led_init();
    // hmi_led_error();
    static const struct gpio_dt_spec led =
        GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
    static const struct gpio_dt_spec sw_side =
        GPIO_DT_SPEC_GET(DT_ALIAS(sw_side), gpios);
    obstacle_manager_init(collision_callback);
    int ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
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
    LOG_INF("MATCH INIT DONE");
    LOG_INF("MATCH WAIT FOR TASKS");
    while (1) {
        if (!shared_ctrl.ready) {
            k_sleep(K_MSEC(100));
            continue;
        }
        break;
    }
    k_sleep(K_MSEC(1000));
    LOG_INF("MATCH WAIT FOR STARTER KEY");
    tirette_wait_until_released();
    hmi_led_success();
    LOG_INF("MATCH START");
    k_sleep(K_MSEC(1000));
    shared_ctrl.start = true;
    int side = gpio_pin_get_dt(&sw_side);
    LOG_DBG("side= %d", side);
    pos2_t dst = {200.0f, 900.0f, 0.25f * M_PI};
    if (side == 0) {
        dst.x = -dst.x;
        dst.a = -dst.a;
    }
    control_set_target(&shared_ctrl, dst);
    uint32_t timeout_counter = 99;
    while (1) {
        LOG_DBG("alive");
        gpio_pin_toggle(led.port, led.pin);
        // control_set_target(&shared_ctrl, dst);
        k_sleep(K_MSEC(1000));
        timeout_counter--;
        if (!timeout_counter) {
            tmc2209_set_speed(&train_motor_1, 0);
            tmc2209_set_speed(&train_motor_2, 0);
            tmc2209_set_speed(&train_motor_3, 0);
            k_sleep(K_MSEC(100));
            k_sched_lock();
            while (1) {
            }
        }
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
    match();
exit:
    return ret;
}
