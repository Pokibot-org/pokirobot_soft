#ifndef TMC2209_H
#define TMC2209_H

#include <device.h>
#include <zephyr.h>
#include "uart_hdb/uart_hdb.h"

// general
#define TMC2209_SYNC 0x5
#define TMC2209_RESERVED 0x0
#define TMC2209_RW_READ 0
#define TMC2209_RW_WRITE 1
#define TMC2209_WREQUEST_FRAME_SIZE 8
#define TMC2209_RREQUEST_FRAME_SIZE 4
#define TMC2209_RREPLY_FRAME_SIZE 8

// values
#define TMC2209_VACTUAL_MAX ((1 << 23) - 1)
#define TMC2209_VACTUAL_MIN (-TMC2209_VACTUAL_MAX)

// registers
#define TMC2209_REG_GCONF 0x00
#define TMC2209_REG_GSTAT 0x01
#define TMC2209_REG_IFCNT 0x02
#define TMC2209_REG_SLAVECONF 0x03
#define TMC2209_REG_OTP_PROG 0x04
#define TMC2209_REG_OTP_READ 0x05
#define TMC2209_REG_IOIN 0x06
#define TMC2209_REG_FACTORY_CONF 0x07

#define TMC2209_REG_IHOLD_IRUN 0x10
#define TMC2209_REG_TPOWER_DOWN 0x11
#define TMC2209_REG_TSTEP 0x12
#define TMC2209_REG_TPWMTHRS 0x13
#define TMC2209_REG_TCOOLTHRS 0x14

#define TMC2209_REG_VACTUAL 0x22

#define TMC2209_REG_SGTHRS 0x40
#define TMC2209_REG_SG_RESULT 0x41
#define TMC2209_REG_COOLCONF 0x42

#define TMC2209_REG_MSCNT 0x6A
#define TMC2209_REG_MSCURACT 0x6B
#define TMC2209_REG_CHOPCONF 0x6C
#define TMC2209_REG_DRV_STATUS 0x6F
#define TMC2209_REG_PWMCONF 0x70
#define TMC2209_REG_PWM_SCALE 0x71
#define TMC2209_REG_PWM_AUTO 0x72

// error and warning codes
#define TMC2209_ERR_RREPLY_CRC 0xFF01
#define TMC2209_ERR_SPEED_RANGE 0x2201

typedef struct tmc2209 {
    uart_hdb_t* uart_hdb;
    const uint8_t addr;
} tmc2209_t;

void _tmc2209_gen_write_buf(uint8_t buf[TMC2209_WREQUEST_FRAME_SIZE], uint8_t slave, uint8_t reg, uint32_t data);
void _tmc2209_gen_read_buf(uint8_t buf[TMC2209_RREQUEST_FRAME_SIZE], uint8_t slave, uint8_t reg);

uint8_t tmc2209_crc(uint8_t* buf, size_t len);
int tmc2209_wrequest(tmc2209_t* dev, uint8_t reg, uint32_t data);
// int tmc2209_rrequest(tmc2209_t* dev, uint8_t reg);
// int tmc2209_rreply(tmc2209_t* dev, uint32_t* data);
// int tmc2209_transeive(tmc2209_t* dev, uint8_t reg, uint32_t* data);

int tmc2209_init(tmc2209_t* dev, uart_hdb_t* uart_hdb);
int tmc2209_set_speed(tmc2209_t* dev, int32_t speed);

#endif // TMC2209_H
