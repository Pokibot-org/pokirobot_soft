#include "shared.h"

#include <zephyr.h>

#include "uart_hdb/uart_hdb.h"
#include "utils.h"
#include <logging/log.h>


LOG_MODULE_REGISTER(shared);

uart_hdb_t steppers_uart_hdb;

int shared_init(void) {
    LOG_INF("shared objects init");
    int ret = 0;
    int tmp = 0;
    tmp =
        uart_hdb_init(&steppers_uart_hdb, DEVICE_DT_GET(DT_ALIAS(stepper_bus)));
    if (tmp) {
        LOG_ERR("failed to init steppers_uart_hdb");
        ret = -10;
    }
    LOG_INF("shared objects init done (ret=%d)", ret);
    return ret;
}
