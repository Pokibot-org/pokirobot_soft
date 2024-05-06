#include "uart_hdb.h"

#include <zephyr/kernel.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(uart_hdb);

void send_receive_it_clbk_V2(const struct device *dev, void *user_data)
{
	struct uart_hdb_it_data *data = user_data;
	if (uart_irq_rx_ready(dev) > 0) {
		uint8_t byte;
		uart_fifo_read(dev, &byte, sizeof(byte));
		if (data->index >= data->to_receive) {
			return;
		}

		data->rx_buffer[data->index] = byte;
		data->index++;
	}
}

int send_receive_V2(struct uart_hdb *hdb, const uint8_t *send_data, size_t send_size,
							 uint8_t *receive_data, size_t receive_size, k_timeout_t timeout)
{
	struct uart_hdb_it_data *data = &hdb->it_data;
	int ret = -ETIMEDOUT;

	if (send_size + receive_size > sizeof(data->rx_buffer)) {
		LOG_ERR("RX buffer to small");
		return -EINVAL;
	}


	// Clear all the received bytes in case the uart driver implement a fifo and we did received
	// unexpected bytes
	uint8_t none;
	while (!uart_poll_in(hdb->uart, &none)) {
	}

	if (uart_irq_callback_user_data_set(hdb->uart, send_receive_it_clbk_V2, data)) {
		ret = -ENOTSUP;
		goto exit;
	}

	data->index = 0;
	data->to_receive = send_size + receive_size;

	uart_irq_rx_enable(hdb->uart);

	for (size_t i = 0; i < send_size; i++) {
		uart_poll_out(hdb->uart, send_data[i]);
	}

	ret = -ETIMEDOUT;
	uint64_t start_time = k_uptime_ticks();
	while (k_uptime_ticks() - start_time < timeout.ticks) {
		if (data->index == data->to_receive) {
            if (receive_data)
            {
			    memcpy(receive_data, &data->rx_buffer[send_size], receive_size);
            }
			ret = 0;
			break;
		}
	}

	if (ret == -ETIMEDOUT) {
		LOG_WRN("Timeout, wanted to receive %d bytes, got %d bytes", receive_size,
				data->index - send_size);
	}

	uart_irq_rx_disable(hdb->uart);
exit:
	return ret;
}


void send_recive_V1(struct uart_hdb *device, uart_hdb_msg_t msg) {
            // LOG_DBG("message received: %02x %02x %02x %02x",
        //     msg.data[0], msg.data[1], msg.data[2], msg.data[3]);
        // LOG_DBG("message received: %02x %02x %02x %02x %02x %02x %02x %02x",
        //     msg.data[0], msg.data[1], msg.data[2], msg.data[3], msg.data[4],
        //     msg.data[5], msg.data[6], msg.data[7]);

        /* clear all the received bytes in case the uart driver implement a fifo and
        we received unexpected bytes. */
        uint8_t none;
        while (!uart_poll_in(device->uart, &none)) {}

        for (size_t i = 0; i < msg.data_size; i++) {
            uart_poll_out(device->uart, msg.data[i]);
        }
        if (msg.answer_buffer) {
            // do {
            //     uart_poll_in(device->uart, &msg.answer_buffer[0]);
            // } while (msg.answer_buffer[0] != msg.data[msg.data_size - 1]);

            for (size_t i = 0; i < msg.answer_buffer_len; i++) {
                int timeout = 0;
                while (uart_poll_in(device->uart, &msg.answer_buffer[i])) {
                    if (timeout > 100) {
                        LOG_ERR("timeout during reply wait byte %d", i);
                        break;
                    }
                    timeout++;
                    k_sleep(K_USEC(1));
                }
            }
            k_sem_give(&msg.answer_received_sem);
            // LOG_DBG("reply received: %02x %02x %02x %02x %02x %02x %02x %02x",
            //     msg.answer_buffer[0], msg.answer_buffer[1],
            //     msg.answer_buffer[2], msg.answer_buffer[3],
            //     msg.answer_buffer[4], msg.answer_buffer[5],
            //     msg.answer_buffer[6], msg.answer_buffer[7]);
        }
}

int uart_hdb_read(const uart_hdb_t *dev, uint8_t *buf, size_t len)
{
    int ret = 0;
    for (int i = 0; i < len; i++) {
        uart_poll_in(dev->uart, buf + i);
    }
    return ret;
}

void uart_hdb_thread(void *arg1, void *arg2, void *arg3)
{
    // const uint8_t min_wait_in_bit_time = 8;
    uart_hdb_t *device = (uart_hdb_t *)arg1;
    LOG_INF("uart_hdb thread launched");
    while (1) {
        uart_hdb_msg_t msg;
        k_msgq_get(&device->frame_queue, &msg, K_FOREVER);
        if (!msg.data_size) {
            LOG_DBG("data_size = 0");
            continue;
        }
#if 1
        send_receive_V2(device, msg.data, msg.data_size, msg.answer_buffer, msg.answer_buffer_len, K_MSEC(150));
        k_sem_give(&msg.answer_received_sem);
#else
        send_recive_V1(device, msg);
#endif

    }
}


int uart_hdb_init(uart_hdb_t *dev, const struct device *uart)
{
    int ret = 0;
    if (uart_hdb_is_ready(dev)) {
        LOG_WRN("UART HDB device already initialized");
        ret = 1;
        goto exit;
    }
    if (!device_is_ready(uart)) {
        LOG_ERR("UART device not ready");
        ret = -1;
        goto exit;
    }
    dev->uart = uart;

    k_msgq_init(&dev->frame_queue, dev->frame_queue_buffer, sizeof(uart_hdb_msg_t),
                UART_HDB_MESSAGE_QUEUE_SIZE);

    dev->thread_id =
        k_thread_create(&dev->thread, dev->thread_stack, UART_HDB_STACK_SIZE, uart_hdb_thread, dev,
                        NULL, NULL, UART_HDB_THREAD_PRIORITY, 0, K_NO_WAIT);
    if (!dev->thread_id) {
        LOG_ERR("Error in uart_hdb thread creation");
        ret = -4;
        goto exit;
    }

    dev->ready = true;
exit:
    return ret;
}

bool uart_hdb_is_ready(uart_hdb_t *dev)
{
    return dev->ready;
}

int uart_hdb_write(uart_hdb_t *dev, const uint8_t *buf, size_t len)
{
    // LOG_DBG("uart_hdb_write");
    if (len >= UART_HDB_MSG_DATA_MAX_SIZE) {
        LOG_WRN("Data to big, increase UART_HDB_MSG_DATA_MAX_SIZE");
        return 1;
    }
    uart_hdb_msg_t msg;
    memcpy(msg.data, buf, len);
    msg.data_size = len;
    msg.answer_buffer = NULL;
    msg.answer_buffer_len = 0;
    k_sem_init(&msg.answer_received_sem, 0, 1);
    k_msgq_put(&dev->frame_queue, &msg, K_FOREVER);
    return 0;
}

int uart_hdb_transceive(uart_hdb_t *dev, const uint8_t *write_buf, size_t write_len,
                        uint8_t *read_buf, size_t read_len)
{
    // LOG_DBG("uart_hdb_transceive");

    if (write_len >= UART_HDB_MSG_DATA_MAX_SIZE) {
        LOG_WRN("Data to big, increase UART_HDB_MSG_DATA_MAX_SIZE");
        return 1;
    }
    uart_hdb_msg_t msg;
    memcpy(&msg.data, write_buf, write_len);
    msg.data_size = write_len;
    msg.answer_buffer = read_buf;
    msg.answer_buffer_len = read_len;
    k_sem_init(&msg.answer_received_sem, 0, 1);
    k_msgq_put(&dev->frame_queue, &msg, K_FOREVER);

    if (k_sem_take(&msg.answer_received_sem, K_MSEC(500)) < 0)
    {
        LOG_ERR("Timeout in transceive");
        return -1;
    }

    return 0;
}
