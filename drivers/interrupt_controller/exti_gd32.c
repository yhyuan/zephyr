/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 * Copyright (c) 2017 RnDity Sp. z o.o.
 * Copyright (c) 2019 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for External interrupt/event controller in GD32 MCUs
 *
 * Driver is currently implemented to support following EXTI lines
 * GD32F1/GD32F3: Lines 0 to 15. Lines > 15 not supported
 * GD32F0/GD32L0/GD32L4/GD32G0/GD32G4: Lines 0 to 15. Lines > 15 are not mapped on an IRQ
 * GD32F2/GD32F4: Lines 0 to 15, 16, 17 18, 21 and 22. Others not supported
 * GD32F7: Lines 0 to 15, 16, 17 18, 21, 22 and 23. Others not supported
 *
 */
#include <device.h>
#include <soc.h>
#include <sys/__assert.h>
#include "exti_gd32.h"

#if defined(CONFIG_SOC_SERIES_GD32F0X) || \
    defined(CONFIG_SOC_SERIES_GD32L0X) || \
    defined(CONFIG_SOC_SERIES_GD32G0X)
const IRQn_Type exti_irq_table[] = {
	EXTI0_1_IRQn, EXTI0_1_IRQn, EXTI2_3_IRQn, EXTI2_3_IRQn,
	EXTI4_15_IRQn, EXTI4_15_IRQn, EXTI4_15_IRQn, EXTI4_15_IRQn,
	EXTI4_15_IRQn, EXTI4_15_IRQn, EXTI4_15_IRQn, EXTI4_15_IRQn,
	EXTI4_15_IRQn, EXTI4_15_IRQn, EXTI4_15_IRQn, EXTI4_15_IRQn
};
#elif defined(CONFIG_SOC_SERIES_GD32F1X) || \
	defined(CONFIG_SOC_SERIES_GD32H7X) || \
	defined(CONFIG_SOC_SERIES_GD32L1X) || \
	defined(CONFIG_SOC_SERIES_GD32L4X) || \
	defined(CONFIG_SOC_SERIES_GD32WBX) || \
	defined(CONFIG_SOC_SERIES_GD32G4X)
const IRQn_Type exti_irq_table[] = {
	EXTI0_IRQn, EXTI1_IRQn, EXTI2_IRQn, EXTI3_IRQn,
	EXTI4_IRQn, EXTI9_5_IRQn, EXTI9_5_IRQn, EXTI9_5_IRQn,
	EXTI9_5_IRQn, EXTI9_5_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn,
	EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn
};
#elif defined(CONFIG_SOC_SERIES_GD32F3X)
const IRQn_Type exti_irq_table[] = {
	EXTI0_IRQn, EXTI1_IRQn, EXTI2_TSC_IRQn, EXTI3_IRQn,
	EXTI4_IRQn, EXTI9_5_IRQn, EXTI9_5_IRQn, EXTI9_5_IRQn,
	EXTI9_5_IRQn, EXTI9_5_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn,
	EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn
};
#elif defined(CONFIG_SOC_SERIES_GD32F2X) || \
	defined(CONFIG_SOC_SERIES_GD32F4X)
const IRQn_Type exti_irq_table[] = {
	EXTI0_IRQn, EXTI1_IRQn, EXTI2_IRQn, EXTI3_IRQn,
	EXTI4_IRQn, EXTI9_5_IRQn, EXTI9_5_IRQn, EXTI9_5_IRQn,
	EXTI9_5_IRQn, EXTI9_5_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn,
	EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn,
	PVD_IRQn, 0xFF, OTG_FS_WKUP_IRQn, 0xFF,
	0xFF, TAMP_STAMP_IRQn, RTC_WKUP_IRQn
};
#elif defined(CONFIG_SOC_SERIES_GD32F7X)
const IRQn_Type exti_irq_table[] = {
	EXTI0_IRQn, EXTI1_IRQn, EXTI2_IRQn, EXTI3_IRQn,
	EXTI4_IRQn, EXTI9_5_IRQn, EXTI9_5_IRQn, EXTI9_5_IRQn,
	EXTI9_5_IRQn, EXTI9_5_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn,
	EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn, EXTI15_10_IRQn,
	PVD_IRQn, 0xFF, OTG_FS_WKUP_IRQn, 0xFF,
	0xFF, TAMP_STAMP_IRQn, RTC_WKUP_IRQn, LPTIM1_IRQn
};
#elif defined(CONFIG_SOC_SERIES_GD32MP1X)
const IRQn_Type exti_irq_table[] = {
	EXTI0_IRQn, EXTI1_IRQn, EXTI2_IRQn, EXTI3_IRQn,
	EXTI4_IRQn, EXTI5_IRQn, EXTI6_IRQn, EXTI7_IRQn,
	EXTI8_IRQn, EXTI9_IRQn, EXTI10_IRQn, EXTI11_IRQn,
	EXTI12_IRQn, EXTI13_IRQn, EXTI14_IRQn, EXTI15_IRQn
};
#endif

/* wrapper for user callback */
struct __exti_cb {
	gd32_exti_callback_t cb;
	void *data;
};

/* driver data */
struct gd32_exti_data {
	/* per-line callbacks */
	struct __exti_cb cb[1];//(exti_irq_table)];
};

int gd32_exti_enable(int port, int line)
{
	int irqnum = 0;

	/* Enable requested line interrupt */
	if (line < 32) {
	//	LL_EXTI_EnableIT_0_31(1 << line);
	} else {
		__ASSERT_NO_MSG(line);
	}

	/* Get matching exti irq mathcing provided line thanks to irq_table */
	if (line < 9) {//TODO RRAY_SIZE(exti_irq_table)) {
		//irqnum = exti_irq_table[line];
		if (irqnum == 0xFF)
			return 0;
	} else {
		return -ENOTSUP;
	}

	/* Enable exti irq interrupt */
	irq_enable(irqnum);

	return 0;
}

void gd32_exti_disable(int port, int line)
{
	if (line < 32) {
//		LL_EXTI_DisableIT_0_31(1 << line);
	} else {

		__ASSERT_NO_MSG(line);

	}
}

/**
 * @brief check if interrupt is pending
 *
 * @param line line number
 */
static inline int gd32_exti_is_pending(int line)
{
	if (line < 32) {
#if defined(CONFIG_SOC_SERIES_GD32MP1X) || defined(CONFIG_SOC_SERIES_GD32G0X)
		return (LL_EXTI_IsActiveRisingFlag_0_31(1 << line) ||
			LL_EXTI_IsActiveFallingFlag_0_31(1 << line));
#else
//		return LL_EXTI_IsActiveFlag_0_31(1 << line);
#endif
	} else {
		__ASSERT_NO_MSG(line);
		return 0;
	}
}

/**
 * @brief clear pending interrupt bit
 *
 * @param line line number
 */
static inline void gd32_exti_clear_pending(int line)
{
	if (line < 32) {
#if defined(CONFIG_SOC_SERIES_GD32MP1X) || defined(CONFIG_SOC_SERIES_GD32G0X)
		LL_EXTI_ClearRisingFlag_0_31(1 << line);
		LL_EXTI_ClearFallingFlag_0_31(1 << line);
#else
//		LL_EXTI_ClearFlag_0_31(1 << line);
#endif
	} else {
		__ASSERT_NO_MSG(line);
	}
}

void gd32_exti_trigger(int port, int line, int trigger)
{
	if (trigger & EXTI_TRIG_RISING) {
		if (line < 32) {
//			LL_EXTI_EnableRisingTrig_0_31(1 << line);
		} else {
			__ASSERT_NO_MSG(line);
		}
	}

	if (trigger & EXTI_TRIG_FALLING) {
		if (line < 32) {
//			LL_EXTI_EnableFallingTrig_0_31(1 << line);
		} else {
			__ASSERT_NO_MSG(line);
		}
	}
}

/**
 * @brief EXTI ISR handler
 *
 * Check EXTI lines in range @min @max for pending interrupts
 *
 * @param arg isr argument
 * @param min low end of EXTI# range
 * @param max low end of EXTI# range
 */
static void __gd32_exti_isr(int min, int max, void *arg)
{
	struct device *dev = arg;
	struct gd32_exti_data *data = dev->driver_data;
	int line;

	/* see which bits are set */
	for (line = min; line < max; line++) {
		/* check if interrupt is pending */
		if (gd32_exti_is_pending(line)) {
			/* clear pending interrupt */
			gd32_exti_clear_pending(line);

			/* run callback only if one is registered */
			if (!data->cb[line].cb) {
				continue;
			}

			data->cb[line].cb(line, data->cb[line].data);
		}
	}
}

#if defined(CONFIG_SOC_SERIES_GD32F0X) || \
	defined(CONFIG_SOC_SERIES_GD32L0X) || \
	defined(CONFIG_SOC_SERIES_GD32G0X)
static inline void __gd32_exti_isr_0_1(void *arg)
{
	__gd32_exti_isr(0, 2, arg);
}

static inline void __gd32_exti_isr_2_3(void *arg)
{
	__gd32_exti_isr(2, 4, arg);
}

static inline void __gd32_exti_isr_4_15(void *arg)
{
	__gd32_exti_isr(4, 16, arg);
}

#else
static inline void __gd32_exti_isr_0(void *arg)
{
	__gd32_exti_isr(0, 1, arg);
}

static inline void __gd32_exti_isr_1(void *arg)
{
	__gd32_exti_isr(1, 2, arg);
}

static inline void __gd32_exti_isr_2(void *arg)
{
	__gd32_exti_isr(2, 3, arg);
}

static inline void __gd32_exti_isr_3(void *arg)
{
	__gd32_exti_isr(3, 4, arg);
}

static inline void __gd32_exti_isr_4(void *arg)
{
	__gd32_exti_isr(4, 5, arg);
}

#if defined(CONFIG_SOC_SERIES_GD32MP1X)
static inline void __gd32_exti_isr_5(void *arg)
{
	__gd32_exti_isr(5, 6, arg);
}

static inline void __gd32_exti_isr_6(void *arg)
{
	__gd32_exti_isr(6, 7, arg);
}

static inline void __gd32_exti_isr_7(void *arg)
{
	__gd32_exti_isr(7, 8, arg);
}

static inline void __gd32_exti_isr_8(void *arg)
{
	__gd32_exti_isr(8, 9, arg);
}

static inline void __gd32_exti_isr_9(void *arg)
{
	__gd32_exti_isr(9, 10, arg);
}

static inline void __gd32_exti_isr_10(void *arg)
{
	__gd32_exti_isr(10, 11, arg);
}

static inline void __gd32_exti_isr_11(void *arg)
{
	__gd32_exti_isr(11, 12, arg);
}

static inline void __gd32_exti_isr_12(void *arg)
{
	__gd32_exti_isr(12, 13, arg);
}

static inline void __gd32_exti_isr_13(void *arg)
{
	__gd32_exti_isr(13, 14, arg);
}

static inline void __gd32_exti_isr_14(void *arg)
{
	__gd32_exti_isr(14, 15, arg);
}

static inline void __gd32_exti_isr_15(void *arg)
{
	__gd32_exti_isr(15, 16, arg);
}
#endif

static inline void __gd32_exti_isr_9_5(void *arg)
{
	__gd32_exti_isr(5, 10, arg);
}

static inline void __gd32_exti_isr_15_10(void *arg)
{
	__gd32_exti_isr(10, 16, arg);
}

#if defined(CONFIG_SOC_SERIES_GD32F4X) || \
	defined(CONFIG_SOC_SERIES_GD32F7X) || \
	defined(CONFIG_SOC_SERIES_GD32F2X) || \
	defined(CONFIG_SOC_SERIES_GD32MP1X)
static inline void __gd32_exti_isr_16(void *arg)
{
	__gd32_exti_isr(16, 17, arg);
}

static inline void __gd32_exti_isr_18(void *arg)
{
	__gd32_exti_isr(18, 19, arg);
}

static inline void __gd32_exti_isr_21(void *arg)
{
	__gd32_exti_isr(21, 22, arg);
}

static inline void __gd32_exti_isr_22(void *arg)
{
	__gd32_exti_isr(22, 23, arg);
}
#endif
#if defined(CONFIG_SOC_SERIES_GD32F7X) || \
	defined(CONFIG_SOC_SERIES_GD32MP1X)
static inline void __gd32_exti_isr_23(void *arg)
{
	__gd32_exti_isr(23, 24, arg);
}
#endif
#endif /* CONFIG_SOC_SERIES_GD32F0X */

static void __gd32_exti_connect_irqs(struct device *dev);

/**
 * @brief initialize EXTI device driver
 */
static int gd32_exti_init(struct device *dev)
{
	__gd32_exti_connect_irqs(dev);

	return 0;
}

static struct gd32_exti_data exti_data;
DEVICE_INIT(exti_gd32, GD32_EXTI_NAME, gd32_exti_init,
	    &exti_data, NULL,
	    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);

/**
 * @brief set & unset for the interrupt callbacks
 */
int gd32_exti_set_callback(int line, int port, gd32_exti_callback_t cb,
				void *arg)
{
	struct device *dev = DEVICE_GET(exti_gd32);
	struct gd32_exti_data *data = dev->driver_data;

	if (data->cb[line].cb) {
		return -EBUSY;
	}

	data->cb[line].cb = cb;
	data->cb[line].data = arg;

	return 0;
}

void gd32_exti_unset_callback(int line)
{
	struct device *dev = DEVICE_GET(exti_gd32);
	struct gd32_exti_data *data = dev->driver_data;

	data->cb[line].cb = NULL;
	data->cb[line].data = NULL;
}

/**
 * @brief connect all interrupts
 */
static void __gd32_exti_connect_irqs(struct device *dev)
{
	ARG_UNUSED(dev);

#if defined(CONFIG_SOC_SERIES_GD32F0X) || \
	defined(CONFIG_SOC_SERIES_GD32L0X) || \
	defined(CONFIG_SOC_SERIES_GD32G0X)
	IRQ_CONNECT(EXTI0_1_IRQn,
		CONFIG_EXTI_GD32_EXTI1_0_IRQ_PRI,
		__gd32_exti_isr_0_1, DEVICE_GET(exti_gd32),
		0);
	IRQ_CONNECT(EXTI2_3_IRQn,
		CONFIG_EXTI_GD32_EXTI3_2_IRQ_PRI,
		__gd32_exti_isr_2_3, DEVICE_GET(exti_gd32),
		0);
	IRQ_CONNECT(EXTI4_15_IRQn,
		CONFIG_EXTI_GD32_EXTI15_4_IRQ_PRI,
		__gd32_exti_isr_4_15, DEVICE_GET(exti_gd32),
		0);
#elif defined(CONFIG_SOC_SERIES_GD32F1X) || \
	defined(CONFIG_SOC_SERIES_GD32F2X) || \
	defined(CONFIG_SOC_SERIES_GD32F3X) || \
	defined(CONFIG_SOC_SERIES_GD32F4X) || \
	defined(CONFIG_SOC_SERIES_GD32F7X) || \
	defined(CONFIG_SOC_SERIES_GD32H7X) || \
	defined(CONFIG_SOC_SERIES_GD32L1X) || \
	defined(CONFIG_SOC_SERIES_GD32L4X) || \
	defined(CONFIG_SOC_SERIES_GD32MP1X) || \
	defined(CONFIG_SOC_SERIES_GD32WBX) || \
	defined(CONFIG_SOC_SERIES_GD32G4X)
	IRQ_CONNECT(EXTI0_IRQn,
		CONFIG_EXTI_GD32_EXTI0_IRQ_PRI,
		__gd32_exti_isr_0, DEVICE_GET(exti_gd32),
		0);
	IRQ_CONNECT(EXTI1_IRQn,
		CONFIG_EXTI_GD32_EXTI1_IRQ_PRI,
		__gd32_exti_isr_1, DEVICE_GET(exti_gd32),
		0);
#ifdef CONFIG_SOC_SERIES_GD32F3X
	IRQ_CONNECT(EXTI2_TSC_IRQn,
		CONFIG_EXTI_GD32_EXTI2_IRQ_PRI,
		__gd32_exti_isr_2, DEVICE_GET(exti_gd32),
		0);
#else
	IRQ_CONNECT(EXTI2_IRQn,
		CONFIG_EXTI_GD32_EXTI2_IRQ_PRI,
		__gd32_exti_isr_2, DEVICE_GET(exti_gd32),
		0);
#endif /* CONFIG_SOC_SERIES_GD32F3X */
	IRQ_CONNECT(EXTI3_IRQn,
		CONFIG_EXTI_GD32_EXTI3_IRQ_PRI,
		__gd32_exti_isr_3, DEVICE_GET(exti_gd32),
		0);
	IRQ_CONNECT(EXTI4_IRQn,
		CONFIG_EXTI_GD32_EXTI4_IRQ_PRI,
		__gd32_exti_isr_4, DEVICE_GET(exti_gd32),
		0);
#ifndef CONFIG_SOC_SERIES_GD32MP1X
	IRQ_CONNECT(EXTI9_5_IRQn,
		CONFIG_EXTI_GD32_EXTI9_5_IRQ_PRI,
		__gd32_exti_isr_9_5, DEVICE_GET(exti_gd32),
		0);
	IRQ_CONNECT(EXTI15_10_IRQn,
		CONFIG_EXTI_GD32_EXTI15_10_IRQ_PRI,
		__gd32_exti_isr_15_10, DEVICE_GET(exti_gd32),
		0);
#else
	IRQ_CONNECT(EXTI5_IRQn,
		CONFIG_EXTI_GD32_EXTI5_IRQ_PRI,
		__gd32_exti_isr_5, DEVICE_GET(exti_gd32),
		0);
	IRQ_CONNECT(EXTI6_IRQn,
		CONFIG_EXTI_GD32_EXTI6_IRQ_PRI,
		__gd32_exti_isr_6, DEVICE_GET(exti_gd32),
		0);
	IRQ_CONNECT(EXTI7_IRQn,
		CONFIG_EXTI_GD32_EXTI7_IRQ_PRI,
		__gd32_exti_isr_7, DEVICE_GET(exti_gd32),
		0);
	IRQ_CONNECT(EXTI8_IRQn,
		CONFIG_EXTI_GD32_EXTI8_IRQ_PRI,
		__gd32_exti_isr_8, DEVICE_GET(exti_gd32),
		0);
	IRQ_CONNECT(EXTI9_IRQn,
		CONFIG_EXTI_GD32_EXTI9_IRQ_PRI,
		__gd32_exti_isr_9, DEVICE_GET(exti_gd32),
		0);
	IRQ_CONNECT(EXTI10_IRQn,
		CONFIG_EXTI_GD32_EXTI10_IRQ_PRI,
		__gd32_exti_isr_10, DEVICE_GET(exti_gd32),
		0);
	IRQ_CONNECT(EXTI11_IRQn,
		CONFIG_EXTI_GD32_EXTI11_IRQ_PRI,
		__gd32_exti_isr_11, DEVICE_GET(exti_gd32),
		0);
	IRQ_CONNECT(EXTI12_IRQn,
		CONFIG_EXTI_GD32_EXTI12_IRQ_PRI,
		__gd32_exti_isr_12, DEVICE_GET(exti_gd32),
		0);
	IRQ_CONNECT(EXTI13_IRQn,
		CONFIG_EXTI_GD32_EXTI13_IRQ_PRI,
		__gd32_exti_isr_13, DEVICE_GET(exti_gd32),
		0);
	IRQ_CONNECT(EXTI14_IRQn,
		CONFIG_EXTI_GD32_EXTI14_IRQ_PRI,
		__gd32_exti_isr_14, DEVICE_GET(exti_gd32),
		0);
	IRQ_CONNECT(EXTI15_IRQn,
		CONFIG_EXTI_GD32_EXTI15_IRQ_PRI,
		__gd32_exti_isr_15, DEVICE_GET(exti_gd32),
		0);
#endif /* CONFIG_SOC_SERIES_GD32MP1X */

#if defined(CONFIG_SOC_SERIES_GD32F2X) || \
	defined(CONFIG_SOC_SERIES_GD32F4X) || \
	defined(CONFIG_SOC_SERIES_GD32F7X)
	IRQ_CONNECT(PVD_IRQn,
		CONFIG_EXTI_GD32_PVD_IRQ_PRI,
		__gd32_exti_isr_16, DEVICE_GET(exti_gd32),
		0);
	IRQ_CONNECT(OTG_FS_WKUP_IRQn,
		CONFIG_EXTI_GD32_OTG_FS_WKUP_IRQ_PRI,
		__gd32_exti_isr_18, DEVICE_GET(exti_gd32),
		0);
	IRQ_CONNECT(TAMP_STAMP_IRQn,
		CONFIG_EXTI_GD32_TAMP_STAMP_IRQ_PRI,
		__gd32_exti_isr_21, DEVICE_GET(exti_gd32),
		0);
	IRQ_CONNECT(RTC_WKUP_IRQn,
		CONFIG_EXTI_GD32_RTC_WKUP_IRQ_PRI,
		__gd32_exti_isr_22, DEVICE_GET(exti_gd32),
		0);
#endif
#if CONFIG_SOC_SERIES_GD32F7X
	IRQ_CONNECT(LPTIM1_IRQn,
		CONFIG_EXTI_GD32_LPTIM1_IRQ_PRI,
		__gd32_exti_isr_23, DEVICE_GET(exti_gd32),
		0);
#endif /* CONFIG_SOC_SERIES_GD32F7X */
#endif
}
