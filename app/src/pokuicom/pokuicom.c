#include "poktocol/poktocol.h"
#include "pokuicom.h"
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pokuicom);

static const struct device *uart_dev = DEVICE_DT_GET(DT_ALIAS(ui_uart));

struct poktocol obj;
bool match_started = false;
bool has_color_info = false;
enum pokprotocol_team received_color;

static void pokprotocol_receive(struct poktocol_msg *msg, void *user_data) 
{
    LOG_INF("Received msg type %d", msg->type);

    switch (msg->type) {
        case POKTOCOL_DATA_TYPE_SCORE:
            break;
        case POKTOCOL_DATA_TYPE_TEAM:
            has_color_info = true;
            received_color = msg->data.team;
            break;
        case POKTOCOL_DATA_TYPE_MATCH_STARTED:
            match_started = true;
            break;
    }
}

static void pokprotocol_send_buffer(char *buffer, size_t len, void *user_data) 
{
    for (size_t i = 0; i< len; i++) {
        uart_poll_out(uart_dev, buffer[i]);
    }
}

void pokuicom_send_score(uint8_t score) 
{
    struct poktocol_msg msg = {
        .cmd = POKTOCOL_CMD_TYPE_WRITE,
        .type = POKTOCOL_DATA_TYPE_SCORE,
        .data.score = score
    };

    pokprotocol_send(&obj, &msg);
}

bool pokuicom_is_match_started(void)
{
    return match_started;
}

int pokuicom_get_team_color(enum pokprotocol_team *color)
{
    if (!has_color_info) {
        return -1;
    }
    *color = received_color;
    return 0;
}

void uart_irq_callback(const struct device *dev, void *user_data)
{
    char c;
    uart_poll_in(uart_dev, &c);
    pokprotocol_feed_byte(&obj, c);
}

int pokuicom_init(void)
{
    struct poktocol_config cfg = {
        .receive = pokprotocol_receive,
        .send = pokprotocol_send_buffer,
        .user_data = NULL
    };
    pokprotocol_init(&obj, &cfg);
    uart_irq_callback_set(uart_dev, uart_irq_callback);
    uart_irq_rx_enable(uart_dev);
    return 0;
}

SYS_INIT(pokuicom_init, APPLICATION, 1);