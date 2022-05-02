#include <zephyr.h>
#include <drivers/uart.h>
#include <logging/log.h>
#include "uart_hdb.h"

LOG_MODULE_REGISTER(uart_hdb, LOG_LEVEL_INF);


int uart_hdb_write(const uart_hdb_t* dev, uint8_t* buf, size_t len) {
    int ret = 0;
    for (int i = 0; i < len; i++) {
        uart_poll_out(dev->uart, buf[i]);
    }
    return ret;
}

int uart_hdb_read(const uart_hdb_t* dev, uint8_t* buf, size_t len) {
    int ret = 0;
    for (int i = 0; i < len; i++) {
        uart_poll_in(dev->uart, buf+i);
    }
exit:
    return ret;
}

int uart_hdb_init(uart_hdb_t* dev, const struct device* uart) {
    int ret = 0;
    if (!device_is_ready(uart)) {
		LOG_ERR("UART device not ready");
		return 1;
	}
    dev->uart = uart;

    /* Here as an example, should be done in the devicetree!
    const struct uart_config uart_cfg = {
            .baudrate = 115200,
            .parity = UART_CFG_PARITY_NONE,
            .stop_bits = UART_CFG_STOP_BITS_1,
            .data_bits = UART_CFG_DATA_BITS_8,
            .flow_ctrl = UART_CFG_FLOW_CTRL_NONE
        };
    ret = uart_configure(dev->uart, &uart_cfg);
	if (ret) {
		return 2;
	}
    */
    struct uart_config uart_cfg;
    ret = uart_config_get(dev->uart, &uart_cfg);
    if (ret) {
        LOG_ERR("UART not configured in devicetree");
        return 3;
    }

    return ret;
}