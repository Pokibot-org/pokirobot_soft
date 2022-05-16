#include <device.h>
#include <devicetree.h>
#include <zephyr.h>
#include "lidar/camsense_x1/camsense_x1.h"
#include "obstacles/relative_obstacle_storing.h"
#include "tmc2209/tmc2209.h"
#include "uart_hdb/uart_hdb.h"
#include <drivers/gpio.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(main);

// #error on callback before decimation check collisions

static obstacle_holder_t obstacles_holders[2];
static uint8_t current_obs_holder_index = 0;
void lidar_receive_data_callback(const lidar_message_t* message, void* user_data) { static float angle_sum = 0; }

void main(void) {
    LOG_INF("BOOTING!");
    // #error on callback before decimation check collisions
    camsense_x1_init(lidar_receive_data_callback, NULL);

    static uart_hdb_t uart_bus;
    uart_hdb_init(&uart_bus, DEVICE_DT_GET(DT_ALIAS(stepper_bus)));
    static tmc2209_t stepper_drv;
    tmc2209_init(&stepper_drv, &uart_bus);

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
    while (1) {
        gpio_pin_toggle(led.port, led.pin);
        k_sleep(K_MSEC(1000));
    }
}
