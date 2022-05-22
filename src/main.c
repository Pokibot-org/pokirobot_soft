#include <device.h>
#include <devicetree.h>
#include <zephyr.h>
#include <drivers/gpio.h>
#include <logging/log.h>

#include "shared.h"
#include "hmi/hmi_led.h"
#include "nav/obstacle_manager.h"
#include "tmc2209/tmc2209.h"
#include "uart_hdb/uart_hdb.h"

LOG_MODULE_REGISTER(main);

// #error on callback before decimation check collisions

void collision_callback(void) {
    LOG_INF("Collision detected");
}

int main(void) {
    LOG_INF("BOOTING!");
    int ret;
    hmi_led_init();
    hmi_led_error();
    shared_init();
    if (shared_init()) {
        LOG_ERR("failed to init shared objects");
        ret = -1;
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
