/*
 * Copyright (c) 2017, embedjournal.com
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <drivers/gpio.h>
#include <drivers/clock_control.h>
#include <drivers/pinmux.h>

#define PINMUX_AF_PP 0x80000
#define PINMUX_AF_OD 0xc0000

#define GD32_PINMUX_FUNC_USART0_TX           (GPIO_DIR_OUT|PINMUX_AF_PP)
#define GD32_PINMUX_FUNC_USART0_RX           (GPIO_DIR_IN)
#define GD32_PINMUX_FUNC_USART1_TX           (GPIO_DIR_OUT|PINMUX_AF_PP)
#define GD32_PINMUX_FUNC_USART1_RX           (GPIO_DIR_IN)
#define GD32_PINMUX_FUNC_USART2_TX           (GPIO_DIR_OUT|PINMUX_AF_PP)
#define GD32_PINMUX_FUNC_USART2_RX           (GPIO_DIR_IN)
#define GD32_PINMUX_FUNC_UART3_TX            (GPIO_DIR_OUT|PINMUX_AF_PP)
#define GD32_PINMUX_FUNC_UART3_RX            (GPIO_DIR_IN)
#define GD32_PINMUX_FUNC_UART4_TX            (GPIO_DIR_OUT|PINMUX_AF_PP)
#define GD32_PINMUX_FUNC_UART4_RX            (GPIO_DIR_IN)
#define GD32_PINMUX_FUNC_I2C0_SCL            (GPIO_DIR_OUT|PINMUX_AF_OD)
#define GD32_PINMUX_FUNC_I2C0_SDA            (GPIO_DIR_OUT|PINMUX_AF_OD)
#define GD32_PINMUX_FUNC_I2C1_SCL            (GPIO_DIR_OUT|PINMUX_AF_OD)
#define GD32_PINMUX_FUNC_I2C1_SDA            (GPIO_DIR_OUT|PINMUX_AF_OD)
#define GD32_PINMUX_FUNC_SPI0_MASTER_HW_NSS  (GPIO_DIR_OUT|PINMUX_AF_PP)
#define GD32_PINMUX_FUNC_SPI0_MASTER_SW_NSS  (GPIO_DIR_OUT)
#define GD32_PINMUX_FUNC_SPI0_MASTER_SCK     (GPIO_DIR_OUT|PINMUX_AF_PP)
#define GD32_PINMUX_FUNC_SPI0_MASTER_MISO    (GPIO_DIR_IN)
#define GD32_PINMUX_FUNC_SPI0_MASTER_MOSI    (GPIO_DIR_OUT|PINMUX_AF_PP)
#define GD32_PINMUX_FUNC_SPI1_MASTER_HW_NSS  (GPIO_DIR_OUT|PINMUX_AF_PP)
#define GD32_PINMUX_FUNC_SPI1_MASTER_SW_NSS  (GPIO_DIR_OUT)
#define GD32_PINMUX_FUNC_SPI1_MASTER_SCK     (GPIO_DIR_OUT|PINMUX_AF_PP)
#define GD32_PINMUX_FUNC_SPI1_MASTER_MISO    (GPIO_DIR_IN)
#define GD32_PINMUX_FUNC_SPI1_MASTER_MOSI    (GPIO_DIR_OUT|PINMUX_AF_PP)
#define GD32_PINMUX_FUNC_SPI2_MASTER_HW_NSS  (GPIO_DIR_OUT|PINMUX_AF_PP)
#define GD32_PINMUX_FUNC_SPI2_MASTER_SW_NSS  (GPIO_DIR_OUT)
#define GD32_PINMUX_FUNC_SPI2_MASTER_SCK     (GPIO_DIR_OUT|PINMUX_AF_PP)
#define GD32_PINMUX_FUNC_SPI2_MASTER_MISO    (GPIO_DIR_IN)
#define GD32_PINMUX_FUNC_SPI2_MASTER_MOSI    (GPIO_DIR_OUT|PINMUX_AF_PP)
#define GD32_PINMUX_FUNC_SPI0_SLAVE_NSS      (GPIO_DIR_IN)
#define GD32_PINMUX_FUNC_SPI0_SLAVE_SCK      (GPIO_DIR_IN)
#define GD32_PINMUX_FUNC_SPI0_SLAVE_MISO     (GPIO_DIR_OUT|PINMUX_AF_PP)
#define GD32_PINMUX_FUNC_SPI0_SLAVE_MOSI     (GPIO_DIR_IN)
#define GD32_PINMUX_FUNC_SPI1_SLAVE_NSS      (GPIO_DIR_IN)
#define GD32_PINMUX_FUNC_SPI1_SLAVE_SCK      (GPIO_DIR_IN)
#define GD32_PINMUX_FUNC_SPI1_SLAVE_MISO     (GPIO_DIR_OUT|PINMUX_AF_PP)
#define GD32_PINMUX_FUNC_SPI1_SLAVE_MOSI     (GPIO_DIR_IN)
#define GD32_PINMUX_FUNC_SPI2_SLAVE_NSS      (GPIO_DIR_IN)
#define GD32_PINMUX_FUNC_SPI2_SLAVE_SCK      (GPIO_DIR_IN)
#define GD32_PINMUX_FUNC_SPI2_SLAVE_MISO     (GPIO_DIR_OUT|PINMUX_AF_PP)
#define GD32_PINMUX_FUNC_SPI2_SLAVE_MOSI     (GPIO_DIR_IN)
#define GD32_PINMUX_FUNC_CAN0_TX             (GPIO_DIR_OUT|PINMUX_AF_PP)
#define GD32_PINMUX_FUNC_CAN0_RX             (GPIO_DIR_IN |GPIO_PUD_PULL_UP)
#define GD32_PINMUX_FUNC_CAN1_TX             (GPIO_DIR_OUT|PINMUX_AF_PP)
#define GD32_PINMUX_FUNC_CAN1_RX             (GPIO_DIR_IN |GPIO_PUD_PULL_UP)

#define PA 0
#define PB 1
#define PC 2
#define PD 3
#define PE 4

#if 0

/* RCU_AHBEN */
#define RCU_AHBEN_DMA0EN                BIT(0)                    /*!< DMA0 clock enable */
#define RCU_AHBEN_DMA1EN                BIT(1)                    /*!< DMA1 clock enable */
#define RCU_AHBEN_SRAMSPEN              BIT(2)                    /*!< SRAM clock enable when sleep mode */
#define RCU_AHBEN_FMCSPEN               BIT(4)                    /*!< FMC clock enable when sleep mode */
#define RCU_AHBEN_CRCEN                 BIT(6)                    /*!< CRC clock enable */
#define RCU_AHBEN_EXMCEN                BIT(8)                    /*!< EXMC clock enable */
#define RCU_AHBEN_USBFSEN               BIT(12)                   /*!< USBFS clock enable */

/* RCU_APB2EN */
#define RCU_APB2EN_ADC0EN               BIT(9)                    /*!< ADC0 clock enable */
#define RCU_APB2EN_ADC1EN               BIT(10)                   /*!< ADC1 clock enable */
#define RCU_APB2EN_TIMER0EN             BIT(11)                   /*!< TIMER0 clock enable */

/* RCU_APB1EN */
#define RCU_APB1EN_TIMER1EN             BIT(0)                    /*!< TIMER1 clock enable */
#define RCU_APB1EN_TIMER2EN             BIT(1)                    /*!< TIMER2 clock enable */
#define RCU_APB1EN_TIMER3EN             BIT(2)                    /*!< TIMER3 clock enable */
#define RCU_APB1EN_TIMER4EN             BIT(3)                    /*!< TIMER4 clock enable */
#define RCU_APB1EN_TIMER5EN             BIT(4)                    /*!< TIMER5 clock enable */
#define RCU_APB1EN_TIMER6EN             BIT(5)                    /*!< TIMER6 clock enable */
#define RCU_APB1EN_WWDGTEN              BIT(11)                   /*!< WWDGT clock enable */
#define RCU_APB1EN_BKPIEN               BIT(27)                   /*!< backup interface clock enable */
#define RCU_APB1EN_PMUEN                BIT(28)                   /*!< PMU clock enable */
#define RCU_APB1EN_DACEN                BIT(29)                   /*!< DAC clock enable */

#endif

#define CLKEN(bus, func) {CK_##bus, RCU_##bus##EN_##func##EN }

#define CLOCK_USART0 {CLKEN(APB2, PA), CLKEN(APB2, USART0) }
#define CLOCK_USART1 {CLKEN(APB2, PA), CLKEN(APB1, USART1) }
#define CLOCK_USART2 {CLKEN(APB2, PB), CLKEN(APB1, USART2) }
#define CLOCK_UART3  {CLKEN(APB2, PC), CLKEN(APB1, UART3)  }
#define CLOCK_UART4  {CLKEN(APB2, PC), CLKEN(APB2, PD), CLKEN(APB1, UART4) }

#define CLOCK_I2C0   {CLKEN(APB2, PA), CLKEN(APB1, I2C0) }
#define CLOCK_I2C1   {CLKEN(APB2, PA), CLKEN(APB1, I2C1) }

#define CLOCK_SPI0   {CLKEN(APB2, PA), CLKEN(APB2, SPI0) }
#define CLOCK_SPI1   {CLKEN(APB2, PB), CLKEN(APB1, SPI1) }
#define CLOCK_SPI2   {CLKEN(APB2, PA), CLKEN(APB2, PB), CLKEN(APB1, SPI1) }

#define CLOCK_CAN0   {CLKEN(APB2, PA), CLKEN(APB1, CAN0) }
#define CLOCK_CAN0   {CLKEN(APB2, PB), CLKEN(APB1, CAN1) }


#define PINMUX_SET(func,port,pin) pinmux_pin_set(device_get_binding(CONFIG_PINMUX_NAME), port*16+pin, GD32_PINMUX_FUNC_##func)

#define CLOCK_ENABLE(configs) { \
	static struct gd32_pclken clkconfs[] = CLOCK_##configs; \
	for(int i=0; i<ARRAY_SIZE(clkconfs); i++) { \
		clock_control_on(device_get_binding(GD32_CLOCK_CONTROL_NAME), (clock_control_subsys_t *) &clkconfs[i]); \
	}\
}

static int pinmux_gd32_init(struct device *port)
{
	ARG_UNUSED(port);

#ifdef CONFIG_UART_0
	CLOCK_ENABLE(USART0);
	PINMUX_SET(USART0_TX, PA,9);
	PINMUX_SET(USART0_RX, PA,10);
#endif	/* CONFIG_UART_0 */
#ifdef CONFIG_UART_1
	CLOCK_ENABLE(USART1);
	PINMUX_SET(USART1_TX, PA,2);
	PINMUX_SET(USART1_RX, PA,3);
#endif	/* CONFIG_UART_1 */
#ifdef CONFIG_UART_2
	CLOCK_ENABLE(USART2);
	PINMUX_SET(USART2_TX, PB,10);
	PINMUX_SET(USART2_RX, PB,11);
#endif	/* CONFIG_UART_2 */
#ifdef CONFIG_UART_3
	CLOCK_ENABLE(UART3);
	PINMUX_SET(UART3_TX, PC,10);
	PINMUX_SET(UART3_RX, PC,11);
#endif	/* CONFIG_UART_3 */
#ifdef CONFIG_UART_4
	/CLOCK_ENABLE(UART4);
	PINMUX_SET(UART4_TX, PC,12);
	PINMUX_SET(UART4_RX, PD,2);
#endif	/* CONFIG_UART_4 */
#ifdef CONFIG_I2C_0
	CLOCK_ENABLE(I2C0);
	PINMUX_SET(I2C0_SCL, PB,6);
	PINMUX_SET(I2C0_SDA, PB,7);
#endif /* CONFIG_I2C_1 */
#ifdef CONFIG_I2C_1
	CLOCK_ENABLE(I2C1);
	PINMUX_SET(I2C1_SCL, PB,10);
	PINMUX_SET(I2C1_SDA, PB,11);
#endif /* CONFIG_I2C_2 */
#ifdef CONFIG_SPI_0
	CLOCK_ENABLE(SPI0);
#ifdef CONFIG_GD32_SPI_0_USE_HW_SS
	PINMUX_SET(SPI0_MASTER_HW_NSS,  PA,4);
#else
	PINMUX_SET(SPI0_MASTER_SW_NSS,  PA,4);
#endif /* CONFIG_SPI_GD32_USE_HW_SS */
	PINMUX_SET(SPI0_MASTER_SCK,  PA,5);
	PINMUX_SET(SPI0_MASTER_MISO, PA,6);
	PINMUX_SET(SPI0_MASTER_MOSI, PA,7);
#endif /* CONFIG_SPI_0 */
#ifdef CONFIG_SPI_1
	CLOCK_ENABLE(SPI1);
#ifdef CONFIG_GD32_SPI_1_USE_HW_SS
	PINMUX_SET(SPI1_MASTER_HW_NSS,  PB,12);
#else
	PINMUX_SET(SPI1_MASTER_SW_NSS,  PB,12);
#endif /* CONFIG_SPI_GD32_USE_HW_SS */
	PINMUX_SET(SPI1_MASTER_SCK,  PB,13);
	PINMUX_SET(SPI1_MASTER_MISO, PB,14);
	PINMUX_SET(SPI1_MASTER_MOSI, PB,15);
#endif /* CONFIG_SPI_1 */
#ifdef CONFIG_SPI_2
	CLOCK_ENABLE(SPI2);
#ifdef CONFIG_GD32_SPI_2_USE_HW_SS
	PINMUX_SET(SPI2_MASTER_HW_NSS,  PA,15);
#endif /* CONFIG_SPI_GD32_USE_HW_SS */
	PINMUX_SET(SPI2_MASTER_SCK,  PB,3);
	PINMUX_SET(SPI2_MASTER_MISO, PB,4);
	PINMUX_SET(SPI2_MASTER_MOSI, PB,5);
#endif /* CONFIG_SPI_2 */
#ifdef CONFIG_CAN_0
	CLOCK_ENABLE(CAN0);
	PINMUX_SET(CAN0_TX, PA,11);
	PINMUX_SET(CAN0_RX, PA,12);
#endif /* CONFIG_I2C_1 */
#ifdef CONFIG_CAN_1
	CLOCK_ENABLE(CAN1);
	PINMUX_SET(CAN1_TX, PB,12);
	PINMUX_SET(CAN1_RX, PB,13);
#endif /* CONFIG_I2C_2 */

	return 0;
}

SYS_INIT(pinmux_gd32_init, PRE_KERNEL_2, CONFIG_PINMUX_GD32_DEVICE_INITIALIZATION_PRIORITY);
