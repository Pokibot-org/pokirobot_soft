#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include "strat/strat.h"

LOG_MODULE_REGISTER(main);

void main(void)
{
    LOG_INF("BOOTING !\n");

    strat_init(TEAM_COLOR_YELLOW);
    strat_run();
    while (1) {
        k_sleep(K_MSEC(1000));
    }
}
