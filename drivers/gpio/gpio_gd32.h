/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_GPIO_GPIO_GD32_H_
#define ZEPHYR_DRIVERS_GPIO_GPIO_GD32_H_

/**
 * @file header for GD32 GPIO
 */

#include <clock_control/gd32_clock_control.h>
//#include <pinmux/gd32/pinmux_gd32.h>
#include <drivers/gpio.h>

/* GPIO buses definitions */

#define GD32_PORT_NOT_AVAILABLE 0xFFFFFFFF

#ifdef CONFIG_SOC_SERIES_GD32F0X
#define GD32_CLOCK_BUS_GPIO GD32_CLOCK_BUS_AHB1
#define GD32_PERIPH_GPIOA LL_AHB1_GRP1_PERIPH_GPIOA
#define GD32_PERIPH_GPIOB LL_AHB1_GRP1_PERIPH_GPIOB
#define GD32_PERIPH_GPIOC LL_AHB1_GRP1_PERIPH_GPIOC
#define GD32_PERIPH_GPIOD LL_AHB1_GRP1_PERIPH_GPIOD
#define GD32_PERIPH_GPIOE LL_AHB1_GRP1_PERIPH_GPIOE
#define GD32_PERIPH_GPIOF LL_AHB1_GRP1_PERIPH_GPIOF
#elif CONFIG_SOC_SERIES_GD32F1X
#define GD32_CLOCK_BUS_GPIO GD32_CLOCK_BUS_APB2
#define GD32_PERIPH_GPIOA LL_APB2_GRP1_PERIPH_GPIOA
#define GD32_PERIPH_GPIOB LL_APB2_GRP1_PERIPH_GPIOB
#define GD32_PERIPH_GPIOC LL_APB2_GRP1_PERIPH_GPIOC
#define GD32_PERIPH_GPIOD LL_APB2_GRP1_PERIPH_GPIOD
#define GD32_PERIPH_GPIOE LL_APB2_GRP1_PERIPH_GPIOE
#define GD32_PERIPH_GPIOF LL_APB2_GRP1_PERIPH_GPIOF
#define GD32_PERIPH_GPIOG LL_APB2_GRP1_PERIPH_GPIOG
#elif CONFIG_SOC_SERIES_GD32F2X
#define GD32_CLOCK_BUS_GPIO GD32_CLOCK_BUS_AHB1
#define GD32_PERIPH_GPIOA LL_AHB1_GRP1_PERIPH_GPIOA
#define GD32_PERIPH_GPIOB LL_AHB1_GRP1_PERIPH_GPIOB
#define GD32_PERIPH_GPIOC LL_AHB1_GRP1_PERIPH_GPIOC
#define GD32_PERIPH_GPIOD LL_AHB1_GRP1_PERIPH_GPIOD
#define GD32_PERIPH_GPIOE LL_AHB1_GRP1_PERIPH_GPIOE
#define GD32_PERIPH_GPIOF LL_AHB1_GRP1_PERIPH_GPIOF
#define GD32_PERIPH_GPIOG LL_AHB1_GRP1_PERIPH_GPIOG
#define GD32_PERIPH_GPIOH LL_AHB1_GRP1_PERIPH_GPIOH
#define GD32_PERIPH_GPIOI LL_AHB1_GRP1_PERIPH_GPIOI
#elif CONFIG_SOC_SERIES_GD32F3X
#define GD32_CLOCK_BUS_GPIO GD32_CLOCK_BUS_AHB1
#define GD32_PERIPH_GPIOA LL_AHB1_GRP1_PERIPH_GPIOA
#define GD32_PERIPH_GPIOB LL_AHB1_GRP1_PERIPH_GPIOB
#define GD32_PERIPH_GPIOC LL_AHB1_GRP1_PERIPH_GPIOC
#define GD32_PERIPH_GPIOD LL_AHB1_GRP1_PERIPH_GPIOD
#define GD32_PERIPH_GPIOE LL_AHB1_GRP1_PERIPH_GPIOE
#define GD32_PERIPH_GPIOF LL_AHB1_GRP1_PERIPH_GPIOF
#define GD32_PERIPH_GPIOG LL_AHB1_GRP1_PERIPH_GPIOG
#define GD32_PERIPH_GPIOH LL_AHB1_GRP1_PERIPH_GPIOH
#elif CONFIG_SOC_SERIES_GD32F4X
#define GD32_CLOCK_BUS_GPIO GD32_CLOCK_BUS_AHB1
#define GD32_PERIPH_GPIOA LL_AHB1_GRP1_PERIPH_GPIOA
#define GD32_PERIPH_GPIOB LL_AHB1_GRP1_PERIPH_GPIOB
#define GD32_PERIPH_GPIOC LL_AHB1_GRP1_PERIPH_GPIOC
#define GD32_PERIPH_GPIOD LL_AHB1_GRP1_PERIPH_GPIOD
#define GD32_PERIPH_GPIOE LL_AHB1_GRP1_PERIPH_GPIOE
#define GD32_PERIPH_GPIOF LL_AHB1_GRP1_PERIPH_GPIOF
#define GD32_PERIPH_GPIOG LL_AHB1_GRP1_PERIPH_GPIOG
#define GD32_PERIPH_GPIOH LL_AHB1_GRP1_PERIPH_GPIOH
#define GD32_PERIPH_GPIOI LL_AHB1_GRP1_PERIPH_GPIOI
#define GD32_PERIPH_GPIOJ LL_AHB1_GRP1_PERIPH_GPIOJ
#define GD32_PERIPH_GPIOK LL_AHB1_GRP1_PERIPH_GPIOK
#elif CONFIG_SOC_SERIES_GD32F7X
#define GD32_CLOCK_BUS_GPIO GD32_CLOCK_BUS_AHB1
#define GD32_PERIPH_GPIOA LL_AHB1_GRP1_PERIPH_GPIOA
#define GD32_PERIPH_GPIOB LL_AHB1_GRP1_PERIPH_GPIOB
#define GD32_PERIPH_GPIOC LL_AHB1_GRP1_PERIPH_GPIOC
#define GD32_PERIPH_GPIOD LL_AHB1_GRP1_PERIPH_GPIOD
#define GD32_PERIPH_GPIOE LL_AHB1_GRP1_PERIPH_GPIOE
#define GD32_PERIPH_GPIOF LL_AHB1_GRP1_PERIPH_GPIOF
#define GD32_PERIPH_GPIOG LL_AHB1_GRP1_PERIPH_GPIOG
#define GD32_PERIPH_GPIOH LL_AHB1_GRP1_PERIPH_GPIOH
#define GD32_PERIPH_GPIOI LL_AHB1_GRP1_PERIPH_GPIOI
#define GD32_PERIPH_GPIOJ LL_AHB1_GRP1_PERIPH_GPIOJ
#define GD32_PERIPH_GPIOK LL_AHB1_GRP1_PERIPH_GPIOK
#elif CONFIG_SOC_SERIES_GD32H7X
#define GD32_CLOCK_BUS_GPIO GD32_CLOCK_BUS_AHB4
#define GD32_PERIPH_GPIOA LL_AHB4_GRP1_PERIPH_GPIOA
#define GD32_PERIPH_GPIOB LL_AHB4_GRP1_PERIPH_GPIOB
#define GD32_PERIPH_GPIOC LL_AHB4_GRP1_PERIPH_GPIOC
#define GD32_PERIPH_GPIOD LL_AHB4_GRP1_PERIPH_GPIOD
#define GD32_PERIPH_GPIOE LL_AHB4_GRP1_PERIPH_GPIOE
#define GD32_PERIPH_GPIOF LL_AHB4_GRP1_PERIPH_GPIOF
#define GD32_PERIPH_GPIOG LL_AHB4_GRP1_PERIPH_GPIOG
#define GD32_PERIPH_GPIOH LL_AHB4_GRP1_PERIPH_GPIOH
#define GD32_PERIPH_GPIOI LL_AHB4_GRP1_PERIPH_GPIOI
#define GD32_PERIPH_GPIOJ LL_AHB4_GRP1_PERIPH_GPIOJ
#define GD32_PERIPH_GPIOK LL_AHB4_GRP1_PERIPH_GPIOK
#elif CONFIG_SOC_SERIES_GD32G0X
#define GD32_CLOCK_BUS_GPIO GD32_CLOCK_BUS_IOP
#define GD32_PERIPH_GPIOA LL_IOP_GRP1_PERIPH_GPIOA
#define GD32_PERIPH_GPIOB LL_IOP_GRP1_PERIPH_GPIOB
#define GD32_PERIPH_GPIOC LL_IOP_GRP1_PERIPH_GPIOC
#define GD32_PERIPH_GPIOD LL_IOP_GRP1_PERIPH_GPIOD
#define GD32_PERIPH_GPIOF LL_IOP_GRP1_PERIPH_GPIOF
#elif CONFIG_SOC_SERIES_GD32L0X
#define GD32_CLOCK_BUS_GPIO GD32_CLOCK_BUS_IOP
#define GD32_PERIPH_GPIOA LL_IOP_GRP1_PERIPH_GPIOA
#define GD32_PERIPH_GPIOB LL_IOP_GRP1_PERIPH_GPIOB
#define GD32_PERIPH_GPIOC LL_IOP_GRP1_PERIPH_GPIOC
#define GD32_PERIPH_GPIOD LL_IOP_GRP1_PERIPH_GPIOD
#define GD32_PERIPH_GPIOE LL_IOP_GRP1_PERIPH_GPIOE
#define GD32_PERIPH_GPIOH LL_IOP_GRP1_PERIPH_GPIOH
#elif CONFIG_SOC_SERIES_GD32L1X
#define GD32_CLOCK_BUS_GPIO GD32_CLOCK_BUS_AHB1
#define GD32_PERIPH_GPIOA LL_AHB1_GRP1_PERIPH_GPIOA
#define GD32_PERIPH_GPIOB LL_AHB1_GRP1_PERIPH_GPIOB
#define GD32_PERIPH_GPIOC LL_AHB1_GRP1_PERIPH_GPIOC
#define GD32_PERIPH_GPIOD LL_AHB1_GRP1_PERIPH_GPIOD
#define GD32_PERIPH_GPIOE LL_AHB1_GRP1_PERIPH_GPIOE
#define GD32_PERIPH_GPIOF LL_AHB1_GRP1_PERIPH_GPIOF
#define GD32_PERIPH_GPIOG LL_AHB1_GRP1_PERIPH_GPIOG
#define GD32_PERIPH_GPIOH LL_AHB1_GRP1_PERIPH_GPIOH
#elif CONFIG_SOC_SERIES_GD32L4X
#define GD32_CLOCK_BUS_GPIO GD32_CLOCK_BUS_AHB2
#define GD32_PERIPH_GPIOA LL_AHB2_GRP1_PERIPH_GPIOA
#define GD32_PERIPH_GPIOB LL_AHB2_GRP1_PERIPH_GPIOB
#define GD32_PERIPH_GPIOC LL_AHB2_GRP1_PERIPH_GPIOC
#define GD32_PERIPH_GPIOD LL_AHB2_GRP1_PERIPH_GPIOD
#define GD32_PERIPH_GPIOE LL_AHB2_GRP1_PERIPH_GPIOE
#define GD32_PERIPH_GPIOF LL_AHB2_GRP1_PERIPH_GPIOF
#define GD32_PERIPH_GPIOG LL_AHB2_GRP1_PERIPH_GPIOG
#define GD32_PERIPH_GPIOH LL_AHB2_GRP1_PERIPH_GPIOH
#define GD32_PERIPH_GPIOI LL_AHB2_GRP1_PERIPH_GPIOI
#elif CONFIG_SOC_SERIES_GD32MP1X
#define GD32_CLOCK_BUS_GPIO GD32_CLOCK_BUS_AHB4
#define GD32_PERIPH_GPIOA LL_AHB4_GRP1_PERIPH_GPIOA
#define GD32_PERIPH_GPIOB LL_AHB4_GRP1_PERIPH_GPIOB
#define GD32_PERIPH_GPIOC LL_AHB4_GRP1_PERIPH_GPIOC
#define GD32_PERIPH_GPIOD LL_AHB4_GRP1_PERIPH_GPIOD
#define GD32_PERIPH_GPIOE LL_AHB4_GRP1_PERIPH_GPIOE
#define GD32_PERIPH_GPIOF LL_AHB4_GRP1_PERIPH_GPIOF
#define GD32_PERIPH_GPIOG LL_AHB4_GRP1_PERIPH_GPIOG
#define GD32_PERIPH_GPIOH LL_AHB4_GRP1_PERIPH_GPIOH
#define GD32_PERIPH_GPIOI LL_AHB4_GRP1_PERIPH_GPIOI
#define GD32_PERIPH_GPIOJ LL_AHB4_GRP1_PERIPH_GPIOJ
#define GD32_PERIPH_GPIOK LL_AHB4_GRP1_PERIPH_GPIOK
#elif CONFIG_SOC_SERIES_GD32WBX
#define GD32_CLOCK_BUS_GPIO GD32_CLOCK_BUS_AHB2
#define GD32_PERIPH_GPIOA LL_AHB2_GRP1_PERIPH_GPIOA
#define GD32_PERIPH_GPIOB LL_AHB2_GRP1_PERIPH_GPIOB
#define GD32_PERIPH_GPIOC LL_AHB2_GRP1_PERIPH_GPIOC
#define GD32_PERIPH_GPIOD LL_AHB2_GRP1_PERIPH_GPIOD
#define GD32_PERIPH_GPIOE LL_AHB2_GRP1_PERIPH_GPIOE
#define GD32_PERIPH_GPIOH LL_AHB2_GRP1_PERIPH_GPIOH
#elif CONFIG_SOC_SERIES_GD32G4X
#define GD32_CLOCK_BUS_GPIO GD32_CLOCK_BUS_AHB2
#define GD32_PERIPH_GPIOA LL_AHB2_GRP1_PERIPH_GPIOA
#define GD32_PERIPH_GPIOB LL_AHB2_GRP1_PERIPH_GPIOB
#define GD32_PERIPH_GPIOC LL_AHB2_GRP1_PERIPH_GPIOC
#define GD32_PERIPH_GPIOD LL_AHB2_GRP1_PERIPH_GPIOD
#define GD32_PERIPH_GPIOE LL_AHB2_GRP1_PERIPH_GPIOE
#define GD32_PERIPH_GPIOF LL_AHB2_GRP1_PERIPH_GPIOF
#define GD32_PERIPH_GPIOG LL_AHB2_GRP1_PERIPH_GPIOG
#endif /* CONFIG_SOC_SERIES_* */

#if 0
#ifdef CONFIG_SOC_SERIES_GD32F1X
#define GD32_PINCFG_MODE_OUTPUT        (GD32_MODE_OUTPUT     \
					 | GD32_CNF_GP_OUTPUT \
					 | GD32_CNF_PUSH_PULL)
#define GD32_PINCFG_MODE_INPUT         GD32_MODE_INPUT
#define GD32_PINCFG_PULL_UP            (GD32_CNF_IN_PUPD | GD32_PUPD_PULL_UP)
#define GD32_PINCFG_PULL_DOWN          (GD32_CNF_IN_PUPD | \
					GD32_PUPD_PULL_DOWN)
#define GD32_PINCFG_FLOATING           (GD32_CNF_IN_FLOAT | \
					GD32_PUPD_NO_PULL)
#else
#define GD32_PINCFG_MODE_OUTPUT        GD32_MODER_OUTPUT_MODE
#define GD32_PINCFG_MODE_INPUT         GD32_MODER_INPUT_MODE
#define GD32_PINCFG_PULL_UP            GD32_PUPDR_PULL_UP
#define GD32_PINCFG_PULL_DOWN          GD32_PUPDR_PULL_DOWN
#define GD32_PINCFG_FLOATING           GD32_PUPDR_NO_PULL
#endif /* CONFIG_SOC_SERIES_GD32F1X */
#endif

#define GD32_MODE_INOUT_MASK 0x3
#define GD32_MODE_INOUT_SHIFT 0x0

#define GD32_CNF_IN_MASK 0x3
#define GD32_CNF_IN_SHIFT 2

#define GD32_CNF_IN_ANALOG 0
#define GD32_CNF_IN_FLOAT  0x4
#define GD32_CNF_IN_PUD    0x8

#define GD32_MODE_INPUT 0

#define GD32_PUPD_MASK 0x3
#define GD32_PUPD_SHIFT 4

#define GD32_PUPD_PULL_UP   0x20
#define GD32_PUPD_PULL_DOWN 0x40

#define GD32_CNF_OUT_1_MASK 0x1
#define GD32_CNF_OUT_1_SHIFT 4
#define GD32_MODE_OSPEED_MASK 0x3 
#define GD32_MODE_OSPEED_SHIFT 0

#define GD32_MODE_OUTPUT_MAX_10 0x1
#define GD32_MODE_OUTPUT_MAX_2  0x2
#define GD32_MODE_OUTPUT_MAX_50 0x3

#define GD32_CNF_OUT_MASK 0x3
#define GD32_CNF_OUT_SHIFT 2

#define GD32_CNF_OUT_PP (0x0 << GD32_CNF_OUT_SHIFT)
#define GD32_CNF_OUT_OD (0x1 << GD32_CNF_OUT_SHIFT)
#define GD32_CNF_AF_PP  (0x2 << GD32_CNF_OUT_SHIFT)
#define GD32_CNF_AF_OD  (0x3 << GD32_CNF_OUT_SHIFT)

/**
 * @brief configuration of GPIO device
 */
struct gpio_gd32_config {
	/* port base address */
	u32_t *base;
	/* IO port */
	int port;
	struct gd32_pclken pclken;
};

/**
 * @brief driver data
 */
struct gpio_gd32_data {
	/* Enabled INT pins generating a cb */
	u32_t cb_pins;
	/* user ISR cb */
	sys_slist_t cb;
};

/**
 * @brief helper for configuration of GPIO pin
 *
 * @param base_addr GPIO port base address
 * @param pin IO pin
 * @param func GPIO mode
 * @param altf Alternate function
 */
int gpio_gd32_configure(u32_t *base_addr, int pin, int conf, int altf);

#endif /* ZEPHYR_DRIVERS_GPIO_GPIO_GD32_H_ */
