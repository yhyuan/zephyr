/*
 * Copyright (c) 2017 Jean-Paul Etienne <fractalclone@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file configuration macros for riscv SOCs supporting the riscv
 *       privileged architecture specification
 */

#ifndef __SOC_COMMON_H_
#define __SOC_COMMON_H_

/* IRQ numbers */
#define RISCV_MACHINE_SOFT_IRQ       3  /* Machine Software Interrupt */
#define RISCV_MACHINE_TIMER_IRQ      7  /* Machine Timer Interrupt */
#define RISCV_MACHINE_EXT_IRQ        11 /* Machine External Interrupt */

/* Exception numbers */
#define RISCV_MACHINE_ECALL_EXP      11 /* Machine ECALL instruction */

/*
 * SOC-specific MSTATUS related info
 */
/* MSTATUS register to save/restore upon interrupt/exception/context switch */
#define SOC_MSTATUS_REG              mstatus

#define SOC_MSTATUS_IEN              (1 << 3) /* Machine Interrupt Enable bit */

/* Previous Privilege Mode - Machine Mode */
#define SOC_MSTATUS_MPP_M_MODE       (3 << 11)
/* Interrupt Enable Bit in Previous Privilege Mode */
#define SOC_MSTATUS_MPIE             (1 << 7)

/*
 * Default MSTATUS register value to restore from stack
 * upon scheduling a thread for the first time
 */
#define SOC_MSTATUS_DEF_RESTORE      (SOC_MSTATUS_MPP_M_MODE | SOC_MSTATUS_MPIE)


/* SOC-specific MCAUSE bitfields */
#ifdef CONFIG_64BIT
/* Interrupt Mask */
#define SOC_MCAUSE_IRQ_MASK          (1 << 63)
/* Exception code Mask */
#else
/* Interrupt Mask */
#define SOC_MCAUSE_IRQ_MASK          (1 << 31)
/* Exception code Mask */
#endif
/* ECALL exception number */
#define SOC_MCAUSE_ECALL_EXP         RISCV_MACHINE_ECALL_EXP
#define SOC_MCAUSE_EXP_MASK          CONFIG_RISCV_SOC_MCAUSE_EXCEPTION_MASK

/* SOC-Specific EXIT ISR command */
#define SOC_ERET                     mret

#ifndef _ASMLANGUAGE

void riscv_irq_enable(unsigned int irq);
void riscv_irq_disable(unsigned int irq);
void riscv_irq_priority_set(unsigned int irq, unsigned int prio);
int riscv_irq_is_enabled(unsigned int irq);

#if defined(CONFIG_RISCV_SOC_INTERRUPT_INIT)
void soc_interrupt_init(void);
#endif

#if defined(CONFIG_RISCV_HAS_PLIC)
#define ARCH_IRQ_CONNECT(irq_p, priority_p, isr_p, isr_param_p, flags_p) \
({ \
	Z_ISR_DECLARE(irq_p, 0, isr_p, isr_param_p); \
	arch_irq_priority_set(irq_p, priority_p); \
	irq_p; \
})
#endif

#endif /* !_ASMLANGUAGE */

#endif /* __SOC_COMMON_H_ */
