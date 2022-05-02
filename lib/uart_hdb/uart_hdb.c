#include "uart_hdb.h"
#include <zephyr.h>
#include <drivers/uart.h>
#include <logging/log.h>

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
        uart_poll_in(dev->uart, buf + i);
    }
    return ret;
}

void uart_hdb_thread(void* arg1, void* arg2, void* arg3) {
    uart_hdb_t* device = (uart_hdb_t*)arg1;
    LOG_INF("Thread launched");
    while (1) {
        uart_hdb_message_t msg;
        k_msgq_get(&device->frame_queue, &msg, K_FOREVER);
    }
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

    k_msgq_init(&dev->frame_queue, dev->frame_queue_buffer, sizeof(uart_hdb_message_t), UART_HDB_MESSAE_QUEUE_SIZE);

    dev->thread_id = k_thread_create(&dev->thread, dev->thread_stack, UART_HDB_STACK_SIZE, uart_hdb_thread, &dev, NULL,
                                     NULL, UART_HDB_THREAD_PRIORITY, 0, K_NO_WAIT);
    if (!dev->thread_id) {
        LOG_ERR("Error in uart_hdb thread creation");
        return 4;
    }

    return ret;
}