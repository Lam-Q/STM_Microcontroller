/*
 * stm32f407VG.h
 *
 *  Created on: Jun 18, 2022
 *      Author: lamqi
 */

#ifndef INC_STM32F407VG_H_
#define INC_STM32F407VG_H_

/*
 * Base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR	0x08000000U  // Flash Base address
#define SRAM1_BASEADDR	0x20000000U // 112KB Sram1 Base Address
#define SRAM1_BASEADDR	0x2001C000U //16KB SRAM2 Base Address
#define SRAM			SRAM1_BASEADDR // SRAM same as SRAM1 base address
#define ROM 			0x1FFF0000U  // System memory



/*
 * AHBx and APBx Bus Pheripheral Base addresses
 */

#define PERIPH_BASE		0x40000000U //
#define APB1PERIPH_BASE PERIPH_BASE // APB1 BASE ADDRESS
#define APB2PERIPH_BASE	0x40010000U // APB2 Base ADDRESS
#define AHB1PERIPH_BASE 0x40020000U // aHB1 BASE ADDRESS
#define AHB2PERIPH_BASE 0x50000000U // AHB2 Base Address


/*
 * GPIO BASE Address
 */

#define GPIOA_BASEADDR	(AHB1PERIPH_BASE + 0x0000 )
#define GPIOB_BASEADDR	(AHB1PERIPH_BASE + 0x0400 )
#define GPIOC_BASEADDR	(AHB1PERIPH_BASE + 0x0800 )
#define GPIOD_BASEADDR	(AHB1PERIPH_BASE + 0x0C00 )
#define GPIOE_BASEADDR	(AHB1PERIPH_BASE + 0x1000 )
#define GPIOF_BASEADDR	(AHB1PERIPH_BASE + 0x1400 )
#define GPIOG_BASEADDR	(AHB1PERIPH_BASE + 0x1800 )
#define GPIOH_BASEADDR	(AHB1PERIPH_BASE + 0x1C00 )
#define GPIOI_BASEADDR	(AHB1PERIPH_BASE + 0x2000 )
#define GPIOJ_BASEADDR	(AHB1PERIPH_BASE + 0x2400 )
#define GPIOK_BASEADDR	(AHB1PERIPH_BASE + 0x2800 )


/*
 * BASE ADDRESess of Peripherals which are on APB1 Bus
 *
 */
#define TIM2_BASEADDR (APB1PERIPH_BASEADDR + 0x5400)

#define I2C1_BASEADDR (APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR (APB1PERIPH_BASEADDR + 0x5800)
#define I2C1_BASEADDR (APB1PERIPH_BASEADDR + 0x5C00)


/*
 * BASE ADDRESess of Peripherals which are on APB2 Bus
 *
 */

#define EXTI_BASEADDR (APB2PERIPH_BASEADDR + 0x3C00)














#endif /* INC_STM32F407VG_H_ */





















