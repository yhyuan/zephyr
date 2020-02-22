/*
 * Copyright (c) 2017 Jean-Paul Etienne <fractalclone@gmail.com>
 * Contributors: 2018 Antmicro <www.antmicro.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "riscv_encoding.h"
	
    .weak  eclic_msip_handler
    .weak  eclic_mtip_handler
    .weak  eclic_bwei_handler
    .weak  eclic_pmovi_handler
    .weak  WWDGT_IRQHandler
    .weak  LVD_IRQHandler
    .weak  TAMPER_IRQHandler
    .weak  RTC_IRQHandler
    .weak  FMC_IRQHandler
    .weak  RCU_IRQHandler
    .weak  EXTI0_IRQHandler
    .weak  EXTI1_IRQHandler
    .weak  EXTI2_IRQHandler
    .weak  EXTI3_IRQHandler
    .weak  EXTI4_IRQHandler
    .weak  DMA0_Channel0_IRQHandler
    .weak  DMA0_Channel1_IRQHandler
    .weak  DMA0_Channel2_IRQHandler
    .weak  DMA0_Channel3_IRQHandler
    .weak  DMA0_Channel4_IRQHandler
    .weak  DMA0_Channel5_IRQHandler
    .weak  DMA0_Channel6_IRQHandler
    .weak  ADC0_1_IRQHandler
    .weak  CAN0_TX_IRQHandler
    .weak  CAN0_RX0_IRQHandler
    .weak  CAN0_RX1_IRQHandler
    .weak  CAN0_EWMC_IRQHandler
    .weak  EXTI5_9_IRQHandler
    .weak  TIMER0_BRK_IRQHandler
    .weak  TIMER0_UP_IRQHandler
    .weak  TIMER0_TRG_CMT_IRQHandler
    .weak  TIMER0_Channel_IRQHandler
    .weak  TIMER1_IRQHandler
    .weak  TIMER2_IRQHandler
    .weak  TIMER3_IRQHandler
    .weak  I2C0_EV_IRQHandler
    .weak  I2C0_ER_IRQHandler
    .weak  I2C1_EV_IRQHandler
    .weak  I2C1_ER_IRQHandler
    .weak  SPI0_IRQHandler
    .weak  SPI1_IRQHandler
    .weak  USART0_IRQHandler
    .weak  USART1_IRQHandler
    .weak  USART2_IRQHandler
    .weak  EXTI10_15_IRQHandler
    .weak  RTC_Alarm_IRQHandler
    .weak  USBFS_WKUP_IRQHandler
    .weak  EXMC_IRQHandler
    .weak  TIMER4_IRQHandler
    .weak  SPI2_IRQHandler
    .weak  UART3_IRQHandler
    .weak  UART4_IRQHandler
    .weak  TIMER5_IRQHandler
    .weak  TIMER6_IRQHandler
    .weak  DMA1_Channel0_IRQHandler
    .weak  DMA1_Channel1_IRQHandler
    .weak  DMA1_Channel2_IRQHandler
    .weak  DMA1_Channel3_IRQHandler
    .weak  DMA1_Channel4_IRQHandler
    .weak  CAN1_TX_IRQHandler
    .weak  CAN1_RX0_IRQHandler
    .weak  CAN1_RX1_IRQHandler
    .weak  CAN1_EWMC_IRQHandler
    .weak  USBFS_IRQHandler



#include <toolchain.h>
vector_base:
    j __start
    .align    2
    .word     0
    .word     0
    .word     __irq_wrapper
    .word     0
    .word     0
    .word  	  0
    .word  	  __irq_wrapper
    .word  	  0
    .word  	  0
    .word  	  0
    .word  	  0
    .word  	  0
    .word  	  0
    .word  	  0
    .word 	  0
    .word  	  0
    .word  	  __irq_wrapper
    .word  	  __irq_wrapper
    .word  	  __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
    .word     0
    .word     0
    .word     0
    .word     0
	.word     0
	.word     __irq_wrapper
	.word     0
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
    .word     0
    .word     0
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper
	.word     __irq_wrapper

/* exports */
GTEXT(__start)

/* imports */
GTEXT(__initialize)
GTEXT(__irq_wrapper)

SECTION_FUNC(vectors, __start)
	.option norvc;

    /* Set the the NMI base to share with mtvec by setting CSR_MMISC_CTL */
    //li t0, 0x200
    //csrs CSR_MMISC_CTL, t0

	/* Intial the mtvt*/
    la t0, vector_base
    csrw CSR_MTVT, t0

	/* Intial the mtvt2 and enable it*/
    //la t0, irq_entry
    //csrw CSR_MTVT2, t0
    //csrs CSR_MTVT2, 0x1

	/*
	 * Set mtvec (Machine Trap-Vector Base-Address Register)
	 * to __irq_wrapper.
	 */
	la t0, __irq_wrapper
	csrw mtvec, t0

	/* Jump to __initialize */
	tail __initialize
