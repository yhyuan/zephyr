/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief
 *
 * A common driver for GD32 pinmux. Each SoC must implement a SoC
 * specific part of the driver.
 */

#include <errno.h>

#include <kernel.h>
#include <device.h>
#include <soc.h>
#include <drivers/clock_control.h>
#include <drivers/pinmux.h>
#include <gpio/gpio_gd32.h>

#include "gd32vf103_gpio.h"

#define MODE_MASK  0x7Cu
#define SPEED_MASK 0x03u
#define INOUT_MASK 0x10u

static inline u32_t gpio_speed(u32_t flags)
{
	int pincfg = 0;

	gpio_gd32_flags_to_conf(flags, &pincfg);
	return (pincfg & INOUT_MASK) ? (pincfg & SPEED_MASK) : 0;
}

static inline u32_t gpio_mode_with_af(u32_t flags)
{
	int pincfg = 0;

	gpio_gd32_flags_to_conf(flags, &pincfg);
	// Use Higher 16bit in flags to store AF flag
	return ((pincfg & ~SPEED_MASK) | ((flags & 0xc0000)>>16));
}


static inline u32_t pin_to_base(u32_t pin)
{
	static const u32_t base[] = {
		DT_GPIO_GD32_GPIOA_BASE_ADDRESS,
		DT_GPIO_GD32_GPIOB_BASE_ADDRESS,
		DT_GPIO_GD32_GPIOC_BASE_ADDRESS,
		DT_GPIO_GD32_GPIOD_BASE_ADDRESS,
		DT_GPIO_GD32_GPIOE_BASE_ADDRESS,
	};

	if((pin/16) >= ARRAY_SIZE(base)) return 0;

	return base[pin/16];
}

static inline u32_t pin_to_bit(u32_t pin)
{
	return 1<<(pin%16);
}

static int pinmux_gd32_set(struct device *dev, u32_t pin, u32_t flags)
{
	int pincfg = 0;

	ARG_UNUSED(dev);
#if 0
	const struct gpio_gd32_config *cfg = dev->config->config_info;
	/* enable clock for subsystem */
	struct device *clk =
		device_get_binding(GD32_CLOCK_CONTROL_NAME);

	if (clock_control_on(clk,
			     (clock_control_subsys_t *)&cfg->pclken) != 0) {
		return -EIO;
	}
#endif

	gpio_init(pin_to_base(pin), gpio_mode_with_af(flags), gpio_speed(flags), pin_to_bit(pin)) ;

        return 0;
}

static int pinmux_gd32_get(struct device *dev, u32_t pin, u32_t *func)
{
        u32_t base = pin_to_base(pin);

	ARG_UNUSED(dev);

	if(base == 0) {
		return -EINVAL;
	}

	if(pin < 8) {
		*func = (GPIO_CTL0(base) >> (4*pin)) & 0xF;
	}
	else if(pin < 16) {
		*func = (GPIO_CTL1(base) >> (4*(pin-8))) & 0xF;
	}
	else {
		return -EIO;
	}

        return 0;
}

static int pinmux_gd32_pullup(struct device *dev, u32_t pin, u8_t func)
{
        return -ENOTSUP;
}

static int pinmux_gd32_input(struct device *dev, u32_t pin, u8_t func)
{
        return -ENOTSUP;
}

static int pinmux_gd32_init(struct device *dev)
{
        return 0;
}

static const struct pinmux_driver_api pinmux_gd32_driver_api = {
        .set = pinmux_gd32_set,
        .get = pinmux_gd32_get,
        .pullup = pinmux_gd32_pullup,
        .input = pinmux_gd32_input,
};

DEVICE_AND_API_INIT(pinmux_gd32_dev, CONFIG_PINMUX_NAME,
		    &pinmux_gd32_init, NULL, NULL,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
		    &pinmux_gd32_driver_api);
