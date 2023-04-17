#include "pokipump.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pokipump);

#if !(DT_NODE_HAS_STATUS(DT_ALIAS(sw_pump), okay) &&                           \
      DT_NODE_HAS_STATUS(DT_ALIAS(sw_valve), okay))
#error Device tree not valid, missing pump and valve
const struct gpio_dt_spec spec_pump;
const struct gpio_dt_spec spec_valve;
#else
const struct gpio_dt_spec spec_pump =
    GPIO_DT_SPEC_GET(DT_ALIAS(sw_pump), gpios);
const struct gpio_dt_spec spec_valve =
    GPIO_DT_SPEC_GET(DT_ALIAS(sw_valve), gpios);
#endif


int pokipump_init(void) {
    int ret = 0;

    if (!device_is_ready(spec_pump.port)) {
        LOG_ERR("Device %s is not ready\n", spec_pump.port->name);
        return -1;
    }

    if (!device_is_ready(spec_valve.port)) {
        LOG_ERR("Device %s is not ready\n", spec_valve.port->name);
        return -1;
    }

    ret = gpio_pin_configure_dt(&spec_pump, GPIO_OUTPUT_LOW);
    if (ret != 0) {
        LOG_ERR("Failed to configure %s\n", spec_pump.port->name);
        return ret;
    }

    ret = gpio_pin_configure_dt(&spec_valve, GPIO_OUTPUT_LOW);
    if (ret != 0) {
        LOG_ERR("Failed to configure %s\n", spec_valve.port->name);
        return ret;
    }
    return ret;
}

int pokipump_suck(void) {
    int err = 0;
    err |= gpio_pin_set_dt(&spec_valve, 1);
    err |= gpio_pin_set_dt(&spec_pump, 0);
    return err;
}

int pokipump_release(void) {
    int err = 0;
    err |= gpio_pin_set_dt(&spec_pump, 0);
    err |= gpio_pin_set_dt(&spec_valve, 1);
    return err;
}
