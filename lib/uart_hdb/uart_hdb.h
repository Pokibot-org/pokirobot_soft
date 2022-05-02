#ifndef UART_HDB_H
#define UART_HDB_H

#include <zephyr.h>
#include <device.h>

#define UART_HDB_STACK_SIZE 1024
#define UART_HDB_THREAD_PRIORITY 1
#define UART_HDB_MESSAE_QUEUE_SIZE 8

typedef struct uart_hdb_message {
    uint32_t toto;
} uart_hdb_message_t;

typedef struct uart_hdb {
    const struct device* uart;
    K_THREAD_STACK_MEMBER(thread_stack, UART_HDB_STACK_SIZE);
    struct k_thread thread;
    k_tid_t thread_id;
    __aligned(4) uint8_t frame_queue_buffer[sizeof(uart_hdb_message_t)*UART_HDB_MESSAE_QUEUE_SIZE];
    struct k_msgq frame_queue;
} uart_hdb_t;

int uart_hdb_init(uart_hdb_t* dev, const struct device* uart);
int uart_hdb_write(const uart_hdb_t* dev, uint8_t* buf, size_t len);

#endif // UART_HDB_H

