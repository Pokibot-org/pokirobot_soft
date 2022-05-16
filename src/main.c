#include <device.h>
#include <devicetree.h>
#include <zephyr.h>
#include <logging/log.h>
#include "tmc2209/tmc2209.h"
#include "uart_hdb/uart_hdb.h"
#include "lidar/camsense_x1/camsense_x1.h"
#include "obstacles/relative_obstacle_storing.h"
LOG_MODULE_REGISTER(main);

// #error on callback before decimation check collisions


static obstacle_holder_t obstacles_holders[2];
static uint8_t current_obs_holder_index = 0;
void lidar_receive_data_callback(const lidar_message_t* message, void* user_data)
{
    static float angle_sum = 0;
}


void main(void) {
    LOG_INF("BOOTING!\n");

    // #error on callback before decimation check collisions
    camsense_x1_init(lidar_receive_data_callback, NULL);

    static uart_hdb_t uart_bus;
    uart_hdb_init(&uart_bus, DEVICE_DT_GET(DT_ALIAS(stepper_bus)));
    static tmc2209_t stepper_drv;
    tmc2209_init(&stepper_drv, &uart_bus);
    while (1) {
        k_sleep(K_MSEC(1000));
    }
}
