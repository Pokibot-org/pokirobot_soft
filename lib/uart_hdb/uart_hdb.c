#include "uart_hdb.h"

#include <zephyr.h>

#include <drivers/uart.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(uart_hdb);

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
        uart_hdb_msg_t msg;
        k_msgq_get(&device->frame_queue, &msg, K_FOREVER);
        if (!msg.data_size)
            continue;
        for (size_t i = 0; i < msg.data_size; i++) {
            uart_poll_out(device->uart, msg.data[i]);
        }
        const uint8_t min_wait_in_bit_time = 8;
        k_busy_wait(1000000 * (min_wait_in_bit_time + 2) / device->baudrate);
        if (msg.answer_buffer) {
            do {
                uart_poll_in(device->uart, &msg.answer_buffer[0]);
            } while (msg.answer_buffer[0] != msg.data[msg.data_size - 1]);

            for (size_t i = 0; i < msg.answer_buffer_len; i++) {
                uart_poll_in(device->uart, &msg.answer_buffer[i]);
            }
            *msg.answer_received = true;
        }
    }
}

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
int uart_hdb_init(uart_hdb_t* dev, const struct device* uart) {
    int ret = 0;
    if (uart_hdb_is_ready(dev)) {
        LOG_WRN("UART HDB device already initialized");
        ret = 1;
    }
    if (!device_is_ready(uart) && 0) { // FIXME
        LOG_ERR("UART device not ready");
        ret = -1;
        goto exit;
    }
    dev->uart = uart;

    struct uart_config uart_cfg;
    ret = uart_config_get(dev->uart, &uart_cfg);
    if (ret) {
        LOG_ERR("UART not configured in devicetree");
        ret = -3;
        goto exit;
    }
    dev->baudrate = uart_cfg.baudrate;

    k_msgq_init(&dev->frame_queue, dev->frame_queue_buffer, sizeof(uart_hdb_msg_t), UART_HDB_MESSAGE_QUEUE_SIZE);

    dev->thread_id = k_thread_create(&dev->thread, dev->thread_stack, UART_HDB_STACK_SIZE, uart_hdb_thread, dev, NULL,
        NULL, UART_HDB_THREAD_PRIORITY, 0, K_NO_WAIT);
    if (!dev->thread_id) {
        LOG_ERR("Error in uart_hdb thread creation");
        ret = -4;
        goto exit;
    }

    dev->ready = true;
exit:
    return ret;
}

bool uart_hdb_is_ready(uart_hdb_t* dev) {
    return dev->ready;
}

int uart_hdb_write(uart_hdb_t* dev, const uint8_t* buf, size_t len) {
    int ret = 0;
    if (len >= UART_HDB_MSG_DATA_MAX_SIZE) {
        LOG_WRN("Data to big, increase UART_HDB_MSG_DATA_MAX_SIZE");
        return 1;
    }
    uart_hdb_msg_t msg;
    memcpy(msg.data, buf, len);
    msg.answer_buffer_len = len;
    msg.answer_buffer = NULL;
    msg.answer_buffer_len = 0;
    msg.answer_received = NULL;
    k_msgq_put(&dev->frame_queue, &msg, K_FOREVER);
    return ret;
}

int uart_hdb_transceive(
    uart_hdb_t* dev, const uint8_t* write_buf, size_t write_len, uint8_t* read_buf, size_t read_len) {
    int ret = 0;
    if (write_len >= UART_HDB_MSG_DATA_MAX_SIZE) {
        LOG_WRN("Data to big, increase UART_HDB_MSG_DATA_MAX_SIZE");
        return 1;
    }
    uart_hdb_msg_t msg;
    memcpy(&msg.data, write_buf, write_len);
    msg.answer_buffer_len = write_len;
    msg.answer_buffer = read_buf;
    msg.answer_buffer_len = read_len;
    bool answer_received = false;
    msg.answer_received = &answer_received;
    k_msgq_put(&dev->frame_queue, &msg, K_FOREVER);

    while (!answer_received) {
        k_yield();
    }

    return ret;
}
