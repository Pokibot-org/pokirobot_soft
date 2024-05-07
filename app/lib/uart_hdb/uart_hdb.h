#ifndef UART_HDB_H
#define UART_HDB_H

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

#define UART_HDB_STACK_SIZE         1024
#define UART_HDB_THREAD_PRIORITY    1
#define UART_HDB_MESSAGE_QUEUE_SIZE 6
#define UART_HDB_MSG_DATA_MAX_SIZE  32

typedef enum uart_hdb_msg_event {
    UART_HDB_MSG_EVENT_SEND_NO_ANSWER,
    UART_HDB_MSG_EVENT_SEND_WITH_ANSWER,
} uart_hdb_msg_event_t;

typedef struct uart_hdb_msg {
    uint8_t data[UART_HDB_MSG_DATA_MAX_SIZE];
    uint8_t data_size;
    uint8_t *answer_buffer;
    uint8_t answer_buffer_len;
    bool *answer_received;
} uart_hdb_msg_t;

typedef struct uart_hdb {
    const struct device *uart;
    bool ready;
    K_THREAD_STACK_MEMBER(thread_stack, UART_HDB_STACK_SIZE);
    struct k_thread thread;
    k_tid_t thread_id;
    char __aligned(4) frame_queue_buffer[sizeof(uart_hdb_msg_t) * UART_HDB_MESSAGE_QUEUE_SIZE];
    struct k_msgq frame_queue;
} uart_hdb_t;

int uart_hdb_init(uart_hdb_t *dev, const struct device *uart);
bool uart_hdb_is_ready(uart_hdb_t *dev);
int uart_hdb_write(uart_hdb_t *dev, const uint8_t *buf, size_t len);
int uart_hdb_transceive(uart_hdb_t *dev, const uint8_t *write_buf, size_t write_len,
                        uint8_t *read_buf, size_t read_len);
#endif // UART_HDB_H
