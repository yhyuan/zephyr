//See LICENSE for license details.
#include <gd32vf103.h>
#include <stdint.h>
#include <stdio.h>
//#include <unistd.h>
#include "riscv_encoding.h"
#include "n200_func.h"
#include <init.h>
  
void eclic_init ( uint32_t num_irq )
{

  typedef volatile uint32_t vuint32_t;

  //clear cfg register 
  *(volatile uint8_t*)(ECLIC_ADDR_BASE+ECLIC_CFG_OFFSET)=0;

  //clear minthresh register 
  *(volatile uint8_t*)(ECLIC_ADDR_BASE+ECLIC_MTH_OFFSET)=0;

  //clear all IP/IE/ATTR/CTRL bits for all interrupt sources
  vuint32_t * ptr;

  vuint32_t * base = (vuint32_t*)(ECLIC_ADDR_BASE + ECLIC_INT_IP_OFFSET);
  vuint32_t * upper = (vuint32_t*)(base + num_irq*4);

  for (ptr = base; ptr < upper; ptr=ptr+4){
    *ptr = 0;
  }
}

void eclic_mode_enable() {
  uint32_t mtvec_value = read_csr(mtvec);
  mtvec_value = mtvec_value & 0xFFFFFFC0;
  mtvec_value = mtvec_value | 0x00000003;
  write_csr(mtvec,mtvec_value);
}


static int _init(struct device* dev)
{
	ARG_UNUSED(dev);

	SystemInit();

	//ECLIC init
	eclic_init(ECLIC_NUM_INTERRUPTS);
	eclic_mode_enable();

	//printf("After ECLIC mode enabled, the mtvec value is %x \n\n\r", read_csr(mtvec));

	// // It must be NOTED:
	//  //    * In the RISC-V arch, if user mode and PMP supported, then by default if PMP is not configured
	//  //      with valid entries, then user mode cannot access any memory, and cannot execute any instructions.
	//  //    * So if switch to user-mode and still want to continue, then you must configure PMP first
	//pmp_open_all_space();
	//switch_m2u_mode();
	
    /* Before enter into main, add the cycle/instret disable by default to save power,
    only use them when needed to measure the cycle/instret */
	asm("csrsi " STRINGIFY(CSR_MCOUNTINHIBIT) ",0x5");
	return 0;
}

SYS_INIT(_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
