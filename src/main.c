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

    pokarm_test();

    while (1) {
        LOG_DBG("step 1");
        gpio_pin_toggle(led.port, led.pin);
        for (uint8_t i = 0; i < 4; i++) {
            train_motor_2.addr = i;
            tmc2209_set_speed(&train_motor_2, 2000);
            // k_sleep(K_MSEC(1));
        }
        k_sleep(K_MSEC(1000));
        LOG_DBG("step 2");
        gpio_pin_toggle(led.port, led.pin);
        for (uint8_t i = 0; i < 4; i++) {
            train_motor_2.addr = i;
            tmc2209_set_speed(&train_motor_2, 0);
            // k_sleep(K_MSEC(1));
        }
        k_sleep(K_MSEC(1000));
        LOG_DBG("step 3");
        gpio_pin_toggle(led.port, led.pin);
        for (uint8_t i = 0; i < 4; i++) {
            train_motor_2.addr = i;
            uint32_t ifcnt;
            tmc2209_get_ifcnt(&train_motor_2, &ifcnt);
            LOG_DBG("addr: %x --- ifcnt: %u", i, ifcnt);
            // k_sleep(K_MSEC(1));
        }
        k_sleep(K_MSEC(1000));
        // tmc2209_set_speed(&train_motor_1, 0);
        // tmc2209_set_speed(&train_motor_2, 0);
        // tmc2209_set_speed(&train_motor_3, 0);
        //  gpio_pin_toggle(led.port, led.pin);
        // tmc2209_set_speed(&train_motor_1, 1000);
        // tmc2209_set_speed(&train_motor_2, 2000);
        // tmc2209_set_speed(&train_motor_3, 4000);
        // k_sleep(K_MSEC(1000));
        // gpio_pin_toggle(led.port, led.pin);
        // tmc2209_set_speed(&train_motor_1, 0);
        // tmc2209_set_speed(&train_motor_2, 0);
        // tmc2209_set_speed(&train_motor_3, 0);
        // k_sleep(K_MSEC(1000));
        // gpio_pin_toggle(led.port, led.pin);
        // tmc2209_set_speed(&train_motor_1, 1000);
        // tmc2209_set_speed(&train_motor_2, 0);
        // tmc2209_set_speed(&train_motor_3, 0);
        // k_sleep(K_MSEC(1000));
        // gpio_pin_toggle(led.port, led.pin);
        // tmc2209_set_speed(&train_motor_1, 0);
        // tmc2209_set_speed(&train_motor_2, 1000);
        // tmc2209_set_speed(&train_motor_3, 0);
        // k_sleep(K_MSEC(1000));
        // gpio_pin_toggle(led.port, led.pin);
        // tmc2209_set_speed(&train_motor_1, 0);
        // tmc2209_set_speed(&train_motor_2, 0);
        // tmc2209_set_speed(&train_motor_3, 1000);
        k_sleep(K_MSEC(1000));
    }
exit:
    return ret;
}
