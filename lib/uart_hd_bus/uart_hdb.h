#ifndef UART_HDB_H
#define UART_HDB_H

#include <zephyr.h>
#include <device.h>


typedef struct uart_hdb {
    const struct device* uart;
} uart_hdb_t;


int uart_hdb_write(const uart_hdb_t* dev, uint8_t* buf, size_t len);


#endif // UART_HDB_H

