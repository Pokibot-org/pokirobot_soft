#include <device.h>
#include <devicetree.h>
#include <zephyr.h>

#include "tmc2209/tmc2209.h"
#include "uart_hdb/uart_hdb.h"
#include <logging/log.h>

LOG_MODULE_REGISTER(main);

void main(void) {
    LOG_INF("BOOTING!\n");

    static uart_hdb_t uart_bus;
    uart_hdb_init(&uart_bus, DEVICE_DT_GET(DT_ALIAS(stepper_bus)));
    static tmc2209_t stepper_drv;
    tmc2209_init(&stepper_drv, &uart_bus);
    while (1) {
        k_sleep(K_MSEC(1000));
    }
}
