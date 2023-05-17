#include "shared.h"

#include <zephyr/kernel.h>

#include "uart_hdb/uart_hdb.h"
#include "pokutils.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(shared);

uart_hdb_t steppers_uart_hdb;

int shared_init(const struct device *dev)
{
	ARG_UNUSED(dev);
	LOG_INF("shared objects init");

	if (uart_hdb_init(&steppers_uart_hdb, DEVICE_DT_GET(DT_ALIAS(stepper_bus)))) {
		LOG_ERR("failed to init steppers_uart_hdb");
		return -ENODEV;
	}
	LOG_INF("shared objects init done");
	return 0;
}

SYS_INIT(shared_init, APPLICATION, 90);
