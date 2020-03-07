/*
 * Copyright (c) 2018 Foundries.io
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <errno.h>
#include <soc.h>
#include <drivers/clock_control.h>
#include <drivers/clock_control/gd32_clock_control.h>

#define LOG_LEVEL CONFIG_CLOCK_CONTROL_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(clock_control);

struct gd32_rcu_config {
	u32_t base_address;
};

#define DEV_CFG(dev)  ((struct gd32_rcu_config *)(dev->config->config_info))
#define DEV_BASE(dev) (DEV_CFG(dev)->base_address)

static uint32_t en_offset[] = {0, AHBEN_REG_OFFSET, APB1EN_REG_OFFSET, APB2EN_REG_OFFSET};

static int gd32_rcu_on(struct device *dev, clock_control_subsys_t sub_system)
{
	uint32_t rcu_gpioa = RCU_GPIOA;
	struct gd32_pclken* pclken = (struct gd32_pclken*)sub_system;
	rcu_periph_clock_enable(RCU_REGIDX_BIT(en_offset[pclken->bus], pclken->enr));

	return 0;
}

static int gd32_rcu_off(struct device *dev, clock_control_subsys_t sub_system)
{
//void rcu_periph_clock_sleep_disable(rcu_periph_sleep_enum periph);
	rcu_periph_clock_disable((rcu_periph_enum)sub_system);
	return 0;
}

static int gd32_rcu_init(struct device *dev)
{
	///rcu_deinit();

	return 0;
}

static const struct clock_control_driver_api gd32_rcu_api = {
	.on = gd32_rcu_on,
	.off = gd32_rcu_off,
	//.get_rate = gd32_rcu_get_rate,
	//.get_status = gd32_rcu_get_status,
};

static struct gd32_rcu_config gd32_rcu_config = {
	.base_address = DT_INST_0_GIGADEVICE_GD32_RCU_BASE_ADDRESS,
};

DEVICE_AND_API_INIT(gd32_rcu, GD32_CLOCK_CONTROL_NAME,
		    &gd32_rcu_init,
		    NULL, &gd32_rcu_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_OBJECTS,
		    &gd32_rcu_api);
