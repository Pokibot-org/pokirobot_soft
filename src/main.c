#include <device.h>
#include <devicetree.h>
#include <zephyr.h>

#include "control/control.h"
#include "figurine_lifter/figurine_lifter.h"
#include "hmi/hmi_led.h"
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

void collision_callback(void) {
    LOG_INF("Collision detected");
}

int main(void) {
    LOG_INF("BOOTING!");
    int ret = 0;
    hmi_led_init();
    hmi_led_error();
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
    control_init(&shared_ctrl, &train_motor_1, &train_motor_2, &train_motor_3);
    // obstacle_manager_init(collision_callback);
    static const struct gpio_dt_spec led =
        GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
    if (!device_is_ready(led.port)) {
        LOG_ERR("failed to init led");
        goto exit;
    }
    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        LOG_ERR("failed to init led");
        goto exit;
    }

    LOG_INF("INIT DONE!");
    hmi_led_success();
    //pokarm_test();
    k_sleep(K_MSEC(1000));
    // wait for init
    while (1) {
        if (!shared_ctrl.ready) {
            k_sleep(K_MSEC(100));
            continue;
        }
        break;
    }
    shared_ctrl.start = true;
    // main thread
    while (1) {
        LOG_DBG("alive");
        gpio_pin_toggle(led.port, led.pin);
        control_set_pos(&shared_ctrl, (pos2_t){0.0f, 0.0f, 0.0f});
        control_set_target(&shared_ctrl, (pos2_t){0.0f, 0.0f, 0.0f});
        k_sleep(K_MSEC(2000));
        LOG_DBG("pos: %.2f %.2f %.2f", shared_ctrl.pos.val.x, shared_ctrl.pos.val.y, shared_ctrl.pos.val.a);
        LOG_DBG("target: %.2f %.2f %.2f", shared_ctrl.target.val.x, shared_ctrl.target.val.y, shared_ctrl.target.val.a);
        gpio_pin_toggle(led.port, led.pin);
        control_set_target(&shared_ctrl, (pos2_t){0.0f, 1300.0f, 0.0f * M_PI});
        k_sleep(K_MSEC(15000));
        LOG_DBG("pos: %.2f %.2f %.2f", shared_ctrl.pos.val.x, shared_ctrl.pos.val.y, shared_ctrl.pos.val.a);
        LOG_DBG("target: %.2f %.2f %.2f", shared_ctrl.target.val.x, shared_ctrl.target.val.y, shared_ctrl.target.val.a);
        goto exit;

        // uint32_t gconf;
        // tmc2209_get_gconf(&train_motor_1, &gconf);
        // k_sleep(K_MSEC(8000));


        // gpio_pin_toggle(led.port, led.pin);
        // control_set_target(&shared_ctrl, (pos2_t){1000.0f, 1000.0f, 2.0f*M_PI});
        // k_sleep(K_MSEC(5000));

        // gpio_pin_toggle(led.port, led.pin);
        // tmc2209_set_speed(&train_motor_1, 0);
        // tmc2209_set_speed(&train_motor_2, 0);
        // tmc2209_set_speed(&train_motor_3, 0);
        // gpio_pin_toggle(led.port, led.pin);
        // tmc2209_set_speed(&train_motor_1, 10000);
        // tmc2209_set_speed(&train_motor_2, 20000);
        // tmc2209_set_speed(&train_motor_3, 40000);
        // k_sleep(K_MSEC(1000));
        // gpio_pin_toggle(led.port, led.pin);
        // tmc2209_set_speed(&train_motor_1, 0);
        // tmc2209_set_speed(&train_motor_2, 0);
        // tmc2209_set_speed(&train_motor_3, 0);
        // k_sleep(K_MSEC(1000));
        // gpio_pin_toggle(led.port, led.pin);
        // tmc2209_set_speed(&train_motor_1, 10000);
        // tmc2209_set_speed(&train_motor_2, 0);
        // tmc2209_set_speed(&train_motor_3, 0);
        // k_sleep(K_MSEC(1000));
        // gpio_pin_toggle(led.port, led.pin);
        // tmc2209_set_speed(&train_motor_1, 0);
        // tmc2209_set_speed(&train_motor_2, 10000);
        // tmc2209_set_speed(&train_motor_3, 0);
        // k_sleep(K_MSEC(1000));
        // gpio_pin_toggle(led.port, led.pin);
        // tmc2209_set_speed(&train_motor_1, 0);
        // tmc2209_set_speed(&train_motor_2, 0);
        // tmc2209_set_speed(&train_motor_3, 10000);
        // k_sleep(K_MSEC(1000));
    }
exit:
    return ret;
}
