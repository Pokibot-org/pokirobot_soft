#include <device.h>
#include <devicetree.h>
#include <zephyr.h>

#include "figurine_lifter/figurine_lifter.h"
#include "hmi/hmi_led.h"
#include "nav/obstacle_manager.h"
#include "pokarm/pokarm.h"
#include "shared.h"
#include "tirette/tirette.h"
#include "tmc2209/tmc2209.h"
#include "uart_hdb/uart_hdb.h"
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
    obstacle_manager_init(collision_callback);

    static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
    if (!device_is_ready(led.port)) {
        goto exit;
    }
    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        goto exit;
    }

    LOG_INF("INIT DONE!");
    hmi_led_success();
    while (1) {
        gpio_pin_toggle(led.port, led.pin);
        k_sleep(K_MSEC(1000));
    }
exit:
    return ret;
}
