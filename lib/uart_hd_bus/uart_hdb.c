#include <zephyr.h>
#include <drivers/uart.h>
#include "uart_hdb.h"


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


