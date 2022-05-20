#include <device.h>
#include <devicetree.h>
#include <zephyr.h>

#include "ihm/led_control.h"
#include "nav/obstacle_manager.h"
#include "tmc2209/tmc2209.h"
#include "uart_hdb/uart_hdb.h"
#include <drivers/gpio.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(main);

// #error on callback before decimation check collisions

void collision_callback(void) {
    LOG_INF("Collision detected");
}

void main(void) {
    LOG_INF("BOOTING!");
    led_control_init();
    rgb_led_set(255, 0 ,0);
    static uart_hdb_t uart_bus;
    uart_hdb_init(&uart_bus, DEVICE_DT_GET(DT_ALIAS(stepper_bus)));
    static tmc2209_t stepper_drv;
    tmc2209_init(&stepper_drv, &uart_bus);
    obstacle_manager_init(collision_callback);
    static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
    int ret;
    if (!device_is_ready(led.port)) {
        return;
    }
    ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    if (ret < 0) {
        return;
    }
    LOG_INF("INIT DONE!");
    rgb_led_set(0, 255, 0);
    while (1) {
        gpio_pin_toggle(led.port, led.pin);
        k_sleep(K_MSEC(1000));
    }
}
