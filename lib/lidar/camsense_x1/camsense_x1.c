#include "camsense_x1.h"

#include <device.h>
#include <devicetree.h>
#include <zephyr.h>

#include <drivers/uart.h>
#include <logging/log.h>

#ifndef CONFIG_UART_INTERRUPT_DRIVEN
#error CONFIG_UART_INTERRUPT_DRIVEN must be enabled
#endif

/*
Format of frames:
    <0x55><0xAA><0x03><0x08>
    <speedL><speedH>
    <startAngleL><startAngleH>
    <distance0L><distance0H><quality0>
    <distance1L><distance1H><quality1>
    <distance2L><distance2H><quality2>
    <distance3L><distance3H><quality3>
    <distance4L><distance4H><quality4>
    <distance5L><distance5H><quality5>
    <distance6L><distance6H><quality6>
    <distance7L><distance7H><quality7>
    <endAngleL><endAngleH>
    <unknown><unknown> could be a CRC

    A package always starts with <0x55><0xAA><0x03><0x08>
*/

LOG_MODULE_REGISTER(camsense_x1_driver, 3);

// DEFINES
#define CAMSENSE_X1_FRAME_SIZE 36
#define CAMSENSE_X1_SPEED_L_INDEX 0
#define CAMSENSE_X1_SPEED_H_INDEX 1
#define CAMSENSE_X1_START_ANGLE_L_INDEX 2
#define CAMSENSE_X1_START_ANGLE_H_INDEX 3
#define CAMSENSE_X1_FIRST_POINT_INDEX 4
#define CAMSENSE_X1_POINT_DISTANCE_L_RELATIVE_INDEX 0
#define CAMSENSE_X1_POINT_DISTANCE_H_RELATIVE_INDEX 1
#define CAMSENSE_X1_POINT_QUALITY_RELATIVE_INDEX 2
#define CAMSENSE_X1_END_ANGLE_L_INDEX 28
#define CAMSENSE_X1_END_ANGLE_H_INDEX 29
#define CAMSENSE_X1_NUMBER_ON_POINTS_IN_MESSAGE 8

#define CAMSENSE_X1_NODE DT_ALIAS(camsense_uart)
#if !(DT_NODE_EXISTS(CAMSENSE_X1_NODE))
#error camsense-uart uart alias is not in device tree and configured
#else

typedef enum {
    frame_parsing_state_header_sync,
    frame_parsing_state_normal
} Frame_parsing_state;

// PRIVATE VARIABLE
static const uint8_t camsense_x1_header[] = {0x55, 0xAA, 0x03, 0x08};
#define CAMSENSE_X1_HEADER_SIZE ARRAY_SIZE(camsense_x1_header)

typedef struct camsense_x1_obj {
    uint8_t frame_buffer[CAMSENSE_X1_FRAME_SIZE];
    float current_speed;
    camsense_x1_msg_clbk msg_callback;
    void* user_data;
    const struct device* uart_dev;
    uint16_t current_number_of_points;
} camsense_x1_obj_t;

camsense_x1_obj_t obj = {0};

// PRIVATE FUNC
/**
 * @brief Process received frame from lidar
 *
 * @param payload: recived frame with no resync header
 */
void process_recived_frame(uint8_t* payload) {
    LOG_DBG("Receiving lidar frame");
    lidar_message_t message;
    obj.current_speed = ((uint16_t)(payload[CAMSENSE_X1_SPEED_H_INDEX] << 8) |
                            payload[CAMSENSE_X1_SPEED_L_INDEX]) /
                        3840.0; // 3840 = (64 * 60)
    message.start_angle =
        -640 + (((uint16_t)payload[CAMSENSE_X1_START_ANGLE_H_INDEX]) << 8 |
                   payload[CAMSENSE_X1_START_ANGLE_L_INDEX]) /
                   64.0; // TODO: Use shift not /
    message.end_angle =
        -640 + (((uint16_t)payload[CAMSENSE_X1_END_ANGLE_H_INDEX]) << 8 |
                   payload[CAMSENSE_X1_END_ANGLE_L_INDEX]) /
                   64.0;

    for (int point_index = 0; point_index < NUMBER_OF_LIDAR_POINTS; point_index++) {
        uint8_t distance_l =
            payload[CAMSENSE_X1_FIRST_POINT_INDEX +
                    CAMSENSE_X1_POINT_DISTANCE_L_RELATIVE_INDEX +
                    (point_index * 3)];
        uint8_t distance_h =
            payload[CAMSENSE_X1_FIRST_POINT_INDEX +
                    CAMSENSE_X1_POINT_DISTANCE_H_RELATIVE_INDEX +
                    (point_index * 3)];
        uint8_t quality = payload[CAMSENSE_X1_FIRST_POINT_INDEX +
                                  CAMSENSE_X1_POINT_QUALITY_RELATIVE_INDEX +
                                  (point_index * 3)];

        message.points[point_index].distance =
            (((uint16_t)distance_h) << 8) | (uint16_t)distance_l;
        message.points[point_index].quality = quality;
    }

    // If one rotation happend, call the
    // on_rotation_callbackon_rotation_callback callback
    if (obj.msg_callback) {
        obj.msg_callback(&message, obj.user_data);
    }
}

void camsense_x1_read_one_frame(void) {
    int err = uart_rx_enable(obj.uart_dev, obj.frame_buffer,
        CAMSENSE_X1_HEADER_SIZE, SYS_FOREVER_MS);
    if (err) {
        LOG_ERR("Cant start full frame read");
    }
}

void camsense_x1_full_frame_callback(
    const struct device* dev, struct uart_event* evt, void* user_data) {
    if (evt->type == UART_RX_RDY) {
        uint8_t* frame = evt->data.rx.buf;
        if (memcmp(frame, camsense_x1_header, CAMSENSE_X1_HEADER_SIZE) == 0) {
            process_recived_frame(&frame[4]);
        } else {
            // Resync
            LOG_WRN("Wrong header, resync started");
            uart_irq_rx_enable(obj.uart_dev);
            return;
        }

        // Reload it/dma
        camsense_x1_read_one_frame();
    }
}

void uart_rx_callback(const struct device* dev, void* user_data) {
    // No need to test uart_irq_rx_ready() if the only it trigger enabeled is rx
    uint8_t recived_byte = 0;
    static uint8_t frame_index = 0;
    static uint8_t header_sync_index = 0;
    static Frame_parsing_state parsing_state = frame_parsing_state_header_sync;

    uart_fifo_read(dev, &recived_byte, 1);

    switch (parsing_state) {
    case frame_parsing_state_header_sync:

        if (recived_byte == camsense_x1_header[header_sync_index]) {
            header_sync_index += 1;
            if (header_sync_index == CAMSENSE_X1_HEADER_SIZE) {
                header_sync_index = 0;
                parsing_state = frame_parsing_state_normal;
            }
        } else {
            header_sync_index = 0;
        }

        break;
    case frame_parsing_state_normal:
        obj.frame_buffer[frame_index] = recived_byte;
        frame_index += 1;
        if (frame_index ==
            (CAMSENSE_X1_FRAME_SIZE - CAMSENSE_X1_HEADER_SIZE - 1)) {
            frame_index = 0;
            parsing_state = frame_parsing_state_header_sync;
            process_recived_frame(obj.frame_buffer);
        }
        break;
    default:
        LOG_ERR("Not supposed to be here, state %d", parsing_state);
        break;
    }
}

// PUBLIC FUNC
/**
 * @brief Camsense driver init
 *
 *  @return -1 if error in the init, -2 if already init, of otherwise
 */
uint8_t camsense_x1_init(camsense_x1_msg_clbk fun, void* user_data) {
    if (obj.uart_dev) {
        LOG_WRN("camsense_x1_init already called");
        return 1;
    }
    // CONFIG UART
    obj.uart_dev = device_get_binding(DT_LABEL(CAMSENSE_X1_NODE));
    if (obj.uart_dev == NULL) {
        LOG_ERR("Cant get the uart device binding");
        return 1;
    }
    const struct uart_config cfg = {.baudrate = 115200,
        .data_bits = UART_CFG_DATA_BITS_8,
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
        .parity = UART_CFG_PARITY_NONE,
        .stop_bits = UART_CFG_STOP_BITS_1};
    int err = uart_configure(obj.uart_dev, &cfg);
    if (err) {
        LOG_ERR("Cant configure the uart");
        return 1;
    }

    obj.user_data = user_data;
    obj.msg_callback = fun;
    // START DRIVER
    uart_irq_callback_user_data_set(obj.uart_dev, uart_rx_callback, NULL);
    uart_irq_rx_enable(obj.uart_dev);
    LOG_INF("Init done");
    return 0;
}

float camsense_x1_get_sensor_speed() {
    return obj.current_speed;
}

#endif
