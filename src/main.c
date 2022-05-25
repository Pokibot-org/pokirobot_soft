#include <device.h>
#include <devicetree.h>
#include <zephyr.h>
#include "kernel.h"
#include <drivers/gpio.h>
#include <logging/log.h>

#include "control/control.h"
#include "figurine_lifter/figurine_lifter.h"
#include "hmi/hmi_led.h"
#include "nav/obstacle_manager.h"
#include "pokarm/pokarm.h"
#include "shared.h"
#include "tirette/tirette.h"
#include "tmc2209/tmc2209.h"
#include "utils.h"

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

    LOG_INF("Waiting for tirette to be released !");
    tirette_wait_until_released();
    hmi_led_success();
    LOG_INF("MATCH START");
    shared_ctrl.start = true;
    while (1) {
        LOG_DBG("alive");
        gpio_pin_toggle(led.port, led.pin);
        k_sleep(K_MSEC(1000));
    }
exit:
    LOG_INF("MATCH DONE (ret: %d)", ret);
    return;
}


int main(void) {
    LOG_INF("BOOTING");
    int ret = 0;

    // obstacle_manager_init(collision_callback);

    //pokarm_test();
    // wait for init

    // main thread
    // test_gconf();
    // test_motor_cmd();
    // test_target();
    // test_calibration();
    match();
exit:
    return ret;
}
