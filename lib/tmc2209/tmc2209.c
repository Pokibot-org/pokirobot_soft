#include "tmc2209.h"

#include <zephyr.h>

#include <drivers/uart.h>
#include <logging/log.h>

LOG_MODULE_REGISTER(tmc2209);

void _tmc2209_gen_write_buf(uint8_t buf[TMC2209_WREQUEST_FRAME_SIZE], uint8_t slave, uint8_t reg, uint32_t data) {
    buf[0] = FIELD_PREP(GENMASK(7, 4), TMC2209_RESERVED) | FIELD_PREP(GENMASK(3, 0), TMC2209_SYNC);
    buf[1] = FIELD_PREP(GENMASK(7, 0), slave);
    buf[2] = FIELD_PREP(GENMASK(7, 7), TMC2209_RW_WRITE) | FIELD_PREP(GENMASK(6, 0), reg);
    buf[3] = FIELD_PREP(GENMASK(7, 0), FIELD_GET(GENMASK(31, 24), data));
    buf[4] = FIELD_PREP(GENMASK(7, 0), FIELD_GET(GENMASK(23, 16), data));
    buf[5] = FIELD_PREP(GENMASK(7, 0), FIELD_GET(GENMASK(15, 8), data));
    buf[6] = FIELD_PREP(GENMASK(7, 0), FIELD_GET(GENMASK(7, 0), data));
    buf[7] = tmc2209_crc(buf, TMC2209_WREQUEST_FRAME_SIZE);
}

void _tmc2209_gen_read_buf(uint8_t buf[TMC2209_RREQUEST_FRAME_SIZE], uint8_t slave, uint8_t reg) {
    buf[0] = FIELD_PREP(GENMASK(7, 4), TMC2209_RESERVED) | FIELD_PREP(GENMASK(3, 0), TMC2209_SYNC);
    buf[1] = FIELD_PREP(GENMASK(7, 0), slave);
    buf[2] = FIELD_PREP(GENMASK(7, 7), TMC2209_RW_READ) | FIELD_PREP(GENMASK(6, 0), reg);
    buf[3] = tmc2209_crc(buf, TMC2209_RREQUEST_FRAME_SIZE);
}

uint8_t tmc2209_crc(uint8_t* buf, size_t len) {
    uint8_t crc = 0;
    for (size_t i = 0; i < (len - 1); i++) {
        uint8_t byte = buf[i];
        for (size_t j = 0; j < 8; j++) {
            if ((crc >> 7) ^ (byte & 0x01)) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc = (crc << 1);
            }
            byte = byte >> 1;
        }
    }
    return crc;
}

int tmc2209_wrequest(tmc2209_t* dev, uint8_t reg, uint32_t data) {
    int ret = 0;
    uint8_t tx_buf[TMC2209_WREQUEST_FRAME_SIZE];
    _tmc2209_gen_write_buf(tx_buf, dev->addr, reg, data);
    ret = uart_hdb_write(dev->uart_hdb, tx_buf, TMC2209_WREQUEST_FRAME_SIZE);
    return ret;
}

// int tmc2209_rrequest(tmc2209_t* dev, uint8_t reg) {
//     int ret = 0;
//     uint8_t tx_buf[TMC2209_RREQUEST_FRAME_SIZE];
//     uint8_t rx_buf[TMC2209_RREPLY_FRAME_SIZE];
//     _tmc2209_gen_read_buf(tx_buf, dev->addr, reg);
//     ret = _tmc2209_write(dev, tx_buf, TMC2209_RREQUEST_FRAME_SIZE);
//     _tmc2209_read(dev, rx_buf, TMC2209_RREQUEST_FRAME_SIZE); // flush
// exit:
//     return ret;
// }
//
// int tmc2209_rreply(tmc2209_t* dev, uint32_t* data) {
//     int ret = 0;
//     uint8_t rx_buf[TMC2209_RREPLY_FRAME_SIZE];
//     ret = _tmc2209_read(dev, rx_buf, TMC2209_RREPLY_FRAME_SIZE);
//     uint8_t crc = tmc2209_crc(rx_buf, TMC2209_RREPLY_FRAME_SIZE);
//     if (crc != rx_buf[7]) { // could check for reply addr and sync too
//         ret = TMC2209_ERR_RREPLY_CRC;
//         goto exit;
//     }
//     *data = FIELD_PREP(GENMASK(31,24), rx_buf[3]) |
//         FIELD_PREP(GENMASK(23,16), rx_buf[4]) |
//         FIELD_PREP(GENMASK(15,8), rx_buf[5]) |
//         FIELD_PREP(GENMASK(7,0), rx_buf[6]);
// exit:
//     return ret;
// }
//
// int tmc2209_transeive(tmc2209_t* dev, uint8_t reg, uint32_t* data) {
//     int ret = 0;
//     ret = tmc2209_rrequest(dev, reg);
//     if (ret) { goto exit; }
//     ret = tmc2209_rreply(dev, data);
//     if (ret) { goto exit; }
// exit:
//     return ret;
// }

int tmc2209_init(tmc2209_t* dev, uart_hdb_t* uart_hdb, uint8_t addr) {
    int ret = 0;
    if (!uart_hdb) {
        LOG_ERR("uart_hdb is NULL");
        return 1;
    }
    dev->uart_hdb = uart_hdb;
    dev-> addr = addr;
    LOG_INF("Device<%p> init ok", (void*)dev);
    return ret;
}

int tmc2209_set_speed(tmc2209_t* dev, int32_t speed) {
    int ret = 0;
    if (speed < TMC2209_VACTUAL_MIN || speed > TMC2209_VACTUAL_MIN) {
        ret = TMC2209_ERR_SPEED_RANGE;
        goto exit;
    }
    tmc2209_wrequest(dev, TMC2209_REG_VACTUAL, speed);
exit:
    return ret;
}
