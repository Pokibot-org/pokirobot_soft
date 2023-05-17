#include "tmc2209.h"

#include <zephyr/kernel.h>

#include <zephyr/sys/util.h>
#include "uart_hdb/uart_hdb.h"
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(tmc2209);

void _tmc2209_gen_write_buf(uint8_t buf[TMC2209_WREQUEST_FRAME_SIZE], uint8_t slave, uint8_t reg,
                            uint32_t data)
{
    buf[0] = FIELD_PREP(GENMASK(7, 4), TMC2209_RESERVED) | FIELD_PREP(GENMASK(3, 0), TMC2209_SYNC);
    buf[1] = FIELD_PREP(GENMASK(7, 0), slave);
    buf[2] = FIELD_PREP(GENMASK(7, 7), TMC2209_RW_WRITE) | FIELD_PREP(GENMASK(6, 0), reg);
    buf[3] = FIELD_PREP(GENMASK(7, 0), FIELD_GET(GENMASK(31, 24), data));
    buf[4] = FIELD_PREP(GENMASK(7, 0), FIELD_GET(GENMASK(23, 16), data));
    buf[5] = FIELD_PREP(GENMASK(7, 0), FIELD_GET(GENMASK(15, 8), data));
    buf[6] = FIELD_PREP(GENMASK(7, 0), FIELD_GET(GENMASK(7, 0), data));
    buf[7] = tmc2209_crc(buf, TMC2209_WREQUEST_FRAME_SIZE);
}

void _tmc2209_gen_read_buf(uint8_t buf[TMC2209_RREQUEST_FRAME_SIZE], uint8_t slave, uint8_t reg)
{
    buf[0] = FIELD_PREP(GENMASK(7, 4), TMC2209_RESERVED) | FIELD_PREP(GENMASK(3, 0), TMC2209_SYNC);
    buf[1] = FIELD_PREP(GENMASK(7, 0), slave);
    buf[2] = FIELD_PREP(GENMASK(7, 7), TMC2209_RW_READ) | FIELD_PREP(GENMASK(6, 0), reg);
    buf[3] = tmc2209_crc(buf, TMC2209_RREQUEST_FRAME_SIZE);
}

uint8_t tmc2209_crc(uint8_t *buf, size_t len)
{
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

int tmc2209_wrequest(tmc2209_t *dev, uint8_t reg, uint32_t data)
{
    // LOG_DBG("tmc2209_wrequest");
    int ret = 0;
    uint8_t tx_buf[TMC2209_WREQUEST_FRAME_SIZE];
    _tmc2209_gen_write_buf(tx_buf, dev->addr, reg, data);
    ret = uart_hdb_write(dev->uart_hdb, tx_buf, TMC2209_WREQUEST_FRAME_SIZE);
    return ret;
}

int tmc2209_rrequest(tmc2209_t *dev, uint8_t reg, uint32_t *data)
{
    LOG_DBG("tmc2209_rrequest");
    int ret = 0;
    uint8_t rx_buf[TMC2209_RREQUEST_FRAME_SIZE];
    uint8_t reply[TMC2209_RREPLY_FRAME_SIZE];
    _tmc2209_gen_read_buf(rx_buf, dev->addr, reg);
    ret = uart_hdb_transceive(dev->uart_hdb, rx_buf, TMC2209_RREQUEST_FRAME_SIZE, reply,
                              TMC2209_RREPLY_FRAME_SIZE);
    *data = FIELD_PREP(GENMASK(31, 24), reply[3]) | FIELD_PREP(GENMASK(23, 16), reply[4]) |
            FIELD_PREP(GENMASK(15, 8), reply[5]) | FIELD_PREP(GENMASK(7, 0), reply[6]);
    return ret;
}

int tmc2209_init(tmc2209_t *dev, uart_hdb_t *uart_hdb, uint8_t addr)
{
    int ret = 0;
    if (!uart_hdb) {
        LOG_ERR("uart_hdb is NULL");
        return 1;
    }
    dev->uart_hdb = uart_hdb;
    dev->addr = addr;
    tmc2209_set_senddelay(dev, 2);
    tmc2209_set_mres(dev, TMC2209_MRES_256);
    tmc2209_set_ihold(dev, 1);
    tmc2209_set_speed(dev, 0);
    LOG_INF("tmc2209 <%p> init ok", (void *)dev);
    return ret;
}

int tmc2209_set_speed(tmc2209_t *dev, int32_t speed)
{
    // LOG_DBG("tmc2209_set_speed");
    // switch (dev->addr) {
    // case 0:
    //     speed *= 1;
    //     break;
    // case 1:
    //     speed *= 4;
    //     break;
    // case 2:
    //     speed *= 8;
    //     break;
    // }
    int ret = 0;
    if (speed < TMC2209_VACTUAL_MIN || speed > TMC2209_VACTUAL_MAX) {
        ret = TMC2209_ERR_SPEED_RANGE;
        goto exit;
    }
    tmc2209_wrequest(dev, TMC2209_REG_VACTUAL, speed);
exit:
    return ret;
}

int tmc2209_set_senddelay(tmc2209_t *dev, uint32_t senddelay)
{
    int ret = 0;
    uint32_t data = FIELD_PREP(GENMASK(11, 8), senddelay);
    tmc2209_wrequest(dev, TMC2209_REG_SLAVECONF, data);
exit:
    return ret;
}

int tmc2209_set_ihold(tmc2209_t *dev, uint32_t ihold)
{
    int ret = 0;
    uint32_t data = TMC2209_IHOLD_IRUN_DEFAULT | FIELD_PREP(GENMASK(4, 0), ihold);
    tmc2209_wrequest(dev, TMC2209_REG_IHOLD_IRUN, data);
exit:
    return ret;
}

int tmc2209_set_mres(tmc2209_t *dev, uint32_t mres)
{
    int ret = 0;
    uint32_t gconf = TMC2209_GCONF_DEFAULT | FIELD_PREP(GENMASK(7, 7), TMC2209_MSTEP_REG_SELECT);
    tmc2209_wrequest(dev, TMC2209_REG_GCONF, gconf);
    uint32_t chopconf = TMC2209_CHOPCONF_DEFAULT | FIELD_PREP(GENMASK(27, 24), mres);
    tmc2209_wrequest(dev, TMC2209_REG_CHOPCONF, chopconf);
exit:
    return ret;
}

int tmc2209_get_gconf(tmc2209_t *dev, uint32_t *gconf)
{
    int ret = 0;
    tmc2209_rrequest(dev, TMC2209_REG_GCONF, gconf);
exit:
    return ret;
}

int tmc2209_get_ifcnt(tmc2209_t *dev, uint32_t *ifcnt)
{
    int ret = 0;
    tmc2209_rrequest(dev, TMC2209_REG_IFCNT, ifcnt);
exit:
    return ret;
}
