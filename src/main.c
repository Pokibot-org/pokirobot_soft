#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <logging/log.h>

#include "tmc2209.h"

LOG_MODULE_REGISTER(main);

void main(void) {
    LOG_INF("boot\n");
    while (1)
    {
        k_sleep(K_MSEC(1000));
    }
}

