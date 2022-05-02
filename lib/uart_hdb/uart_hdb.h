#ifndef UART_HDB_H
#define UART_HDB_H

#include <zephyr.h>
#include <device.h>

#define UART_HDB_STACK_SIZE 1024

typedef struct uart_hdb {
    const struct device* uart;
    K_THREAD_STACK_MEMBER(thread_stack, UART_HDB_STACK_SIZE);
} uart_hdb_t;

int uart_hdb_init(uart_hdb_t* dev, const struct device* uart);
int uart_hdb_write(const uart_hdb_t* dev, uint8_t* buf, size_t len);


#endif // UART_HDB_H

