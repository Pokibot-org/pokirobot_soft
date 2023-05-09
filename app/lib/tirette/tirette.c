#include "tirette.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(tirette);

#if !(DT_NODE_HAS_STATUS(DT_ALIAS(sw_tirette), okay))
#error Device tree not valid, missing stuff for figurine lifter
static const struct gpio_dt_spec tirette_spec;
#else
static const struct gpio_dt_spec tirette_spec = GPIO_DT_SPEC_GET(DT_ALIAS(sw_tirette), gpios);
#endif

int tirette_init(const struct device *dev)
{
	UNUSED(dev);
	int err = 0;
	err |= gpio_pin_configure_dt(&tirette_spec, GPIO_INPUT);
	if (err) {
		LOG_ERR("Error in init %d", err);
	}

	LOG_INF("Init done");
	return err;
}

void tirette_wait_until_released(void)
{
	while (gpio_pin_get_dt(&tirette_spec)) {
		k_sleep(K_MSEC(1));
	}
}

SYS_INIT(tirette_init, APPLICATION, 90);