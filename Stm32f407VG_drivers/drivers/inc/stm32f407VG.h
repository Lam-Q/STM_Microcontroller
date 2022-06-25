/*
 * stm32f407VG.h
 *
 *  Created on: Jun 18, 2022
 *      Author: lamqi
 */

#ifndef INC_STM32F407VG_H_
#define INC_STM32F407VG_H_
#define __vo volatile

#include <stdint.h>
/*
 * Base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR	0x08000000U  // Flash Base address
#define SRAM1_BASEADDR	0x20000000U // 112KB Sram1 Base Address
#define SRAM2_BASEADDR	0x2001C000U //16KB SRAM2 Base Address
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

#define GPIOA_BASEADDR		(AHB1PERIPH_BASE + 0x0000 )
#define GPIOB_BASEADDR		(AHB1PERIPH_BASE + 0x0400 )
#define GPIOC_BASEADDR		(AHB1PERIPH_BASE + 0x0800 )
#define GPIOD_BASEADDR		(AHB1PERIPH_BASE + 0x0C00 )
#define GPIOE_BASEADDR		(AHB1PERIPH_BASE + 0x1000 )
#define GPIOF_BASEADDR		(AHB1PERIPH_BASE + 0x1400 )
#define GPIOG_BASEADDR		(AHB1PERIPH_BASE + 0x1800 )
#define GPIOH_BASEADDR		(AHB1PERIPH_BASE + 0x1C00 )
#define GPIOI_BASEADDR		(AHB1PERIPH_BASE + 0x2000 )
#define GPIOJ_BASEADDR		(AHB1PERIPH_BASE + 0x2400 )
#define GPIOK_BASEADDR		(AHB1PERIPH_BASE + 0x2800 )
#define CRC_BASEADDR		(AHB1PERIPH_BASE + 0x3000 )
#define RCC_BASEADDR		(AHB1PERIPH_BASE + 0x3800 )

/*
 * BASE ADDRESess of Peripherals which are on APB1 Bus
 *
 */

#define TIM2_BASEADDR		(APB1PERIPH_BASEADDR + 0x0000)
#define TIM3_BASEADDR		(APB1PERIPH_BASEADDR + 0x0400)
#define TIM4_BASEADDR		(APB1PERIPH_BASEADDR + 0x0800)
#define TIM5_BASEADDR 		(APB1PERIPH_BASEADDR + 0x0C00)
#define TIM6_BASEADDR 		(APB1PERIPH_BASEADDR + 0x1000)
#define TIM7_BASEADDR 		(APB1PERIPH_BASEADDR + 0x1400)
#define TIM12_BASEADDR 		(APB1PERIPH_BASEADDR + 0x1800)
#define TIM13_BASEADDR  	(APB1PERIPH_BASEADDR + 0x1C00)
#define TIM14_BASEADDR 		(APB1PERIPH_BASEADDR + 0x2000)
#define RTC_BASEADDR  		(APB1PERIPH_BASEADDR + 0x2800)
#define WWDG_BASEADDR  		(APB1PERIPH_BASEADDR + 0x2C00)
#define IWDG_BASEADDR  		(APB1PERIPH_BASEADDR + 0x3000)
#define I2S2ext_BASEADDR  	(APB1PERIPH_BASEADDR + 0x3400)
#define SPI2_BASEADDR  		(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR  		(APB1PERIPH_BASEADDR + 0x3C00)
#define I2S3ext_BASEADDR  	(APB1PERIPH_BASEADDR + 0x4000)
#define USART2_BASEADDR  	(APB1PERIPH_BASEADDR + 0x4400)  // Can work in synchronous and ASsyncronous
#define USART3_BASEADDR  	(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR  	(APB1PERIPH_BASEADDR + 0x4C00)  // Doesnt support synchronouse
#define UART5_BASEADDR  	(APB1PERIPH_BASEADDR + 0x5000)
#define I2C1_BASEADDR 		(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR		(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR 		(APB1PERIPH_BASEADDR + 0x5C00)
#define CAN1_BASEADDR  		(APB1PERIPH_BASEADDR + 0x6400)
#define CAN2_BASEADDR  		(APB1PERIPH_BASEADDR + 0x6800)
#define CAN3_BASEADDR  		(APB1PERIPH_BASEADDR + 0x7000)
#define CAN4_BASEADDR  		(APB1PERIPH_BASEADDR + 0x7400)
#define UART7_BASEADDR  	(APB1PERIPH_BASEADDR + 0x7800)
#define UART8_BASEADDR  	(APB1PERIPH_BASEADDR + 0x7C00)


/*
 * BASE ADDRESess of Peripherals which are on APB2 Bus
 *
 */



#define TIM1_BASEADDR 		(APB2PERIPH_BASEADDR + 0x0000)
#define TIM8_BASEADDR 		(APB2PERIPH_BASEADDR + 0x0400)
#define USART1_BASEADDR		(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR		(APB2PERIPH_BASEADDR + 0x1400)
#define ADC1_2_3_BASEADDR	(APB2PERIPH_BASEADDR + 0x2000)
#define SDIO_BASEADDR		(APB2PERIPH_BASEADDR + 0x2C00)
#define SPI1_BASEADDR		(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR		(APB2PERIPH_BASEADDR + 0x3400)
#define SYSCFG_BASEADDR		(APB2PERIPH_BASEADDR + 0x3800)
#define EXTI_BASEADDR 		(APB2PERIPH_BASEADDR + 0x3C00)
#define TIM9_BASEADDR 		(APB2PERIPH_BASEADDR + 0x4000)
#define TIM10_BASEADDR 		(APB2PERIPH_BASEADDR + 0x4400)
#define TIM11_BASEADDR 		(APB2PERIPH_BASEADDR + 0x4800)
#define SPI5_BASEADDR 		(APB2PERIPH_BASEADDR + 0x5000)
#define SPI6_BASEADDR 		(APB2PERIPH_BASEADDR + 0x5400)
#define SAI1_BASEADDR 		(APB2PERIPH_BASEADDR + 0x5800)
#define LCD_TFT_BASEADDR 	(APB2PERIPH_BASEADDR + 0x6800)



/*
 * Peripheral register definitnion for GPIO
 */
typedef struct
{
	__vo uint32_t MODER;			//GPIO port mode register Address offset: 0x00
	__vo uint32_t OTYPER;		//Address offset: 0x00
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];


}GPIO_RegDef_t;

typedef struct
{
	__vo uint32_t CR;			//GPIO port mode register Address offset: 0x00
	__vo uint32_t PLLCFGR;		//Address offset: 0x00
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	uint32_t RESERVED2;
    __vo uint32_t APB1ENR;
    __vo uint32_t APB2ENR;
    uint32_t RESERVED3[2];
    __vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	__vo uint32_t RESERVED4;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t RESERVED6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;

}RCC_RegDef_t;

/*
 * Peripheral definition (Peripheral base address typecasted to xxx_RegDef_t
 *
 */

#define GPIOA 			((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 			((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 			((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 			((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 			((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 			((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 			((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 			((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI 			((GPIO_RegDef_t*)GPIOI_BASEADDR)


#define RCC				((RCC_RegDef_t*)RCC_BASEADDR)


/*
 * Clock Enable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()	(RCC->AHB1ENR |= (1 << 0 ))
#define GPIOB_PCLK_EN()	(RCC->AHB1ENR |= (1 << 1 ))
#define GPIOC_PCLK_EN()	(RCC->AHB1ENR |= (1 << 2 ))
#define GPIOD_PCLK_EN()	(RCC->AHB1ENR |= (1 << 3 ))
#define GPIOE_PCLK_EN()	(RCC->AHB1ENR |= (1 << 4 ))
#define GPIOF_PCLK_EN()	(RCC->AHB1ENR |= (1 << 5 ))
#define GPIOG_PCLK_EN()	(RCC->AHB1ENR |= (1 << 6 ))
#define GPIOH_PCLK_EN()	(RCC->AHB1ENR |= (1 << 7 ))
#define GPIOI_PCLK_EN()	(RCC->AHB1ENR |= (1 << 8 ))

/*
 * Clock Enable macros for I2Cx peripherals
 */

#define I2C1_PCLK_EN()	(RCC->APB1ENR |= (1 << 21 ))
#define I2C2_PCLK_EN()	(RCC->APB1ENR |= (1 << 22 ))
#define I2C3_PCLK_EN()	(RCC->APB1ENR |= (1 << 23 ))

/*
 * Clock Enable macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()	(RCC->APB2ENR |= (1 << 12 ))
#define SPI2_PCLK_EN()	(RCC->APB1ENR |= (1 << 14 ))
#define SPI3_PCLK_EN()	(RCC->APB1ENR |= (1 << 15 ))

/*
 * Clock Enable macros for USARTx peripherals
 */

#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 << 4 ))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1 << 18))
#define USART6_PCLK_EN()	(RCC->APB2ENR |= (1 << 5 ))

/*
 * Clock Enable macros for SYSCFGx peripherals
 */

#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 14 ))

/*
 * Clock Disable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 0 ))
#define GPIOB_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 1 ))
#define GPIOC_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 2 ))
#define GPIOD_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 3 ))
#define GPIOE_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 4 ))
#define GPIOF_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 5 ))
#define GPIOG_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 6 ))
#define GPIOH_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 7 ))
#define GPIOI_PCLK_DI()	(RCC->AHB1ENR &= ~(1 << 8 ))

/*
 * Clock Disable macros for I2Cx peripherals
 */

#define I2C1_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 21 ))
#define I2C2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 22 ))
#define I2C3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 23 ))

/*
 * Clock Disable macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 12 ))
#define SPI2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 14 ))
#define SPI3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 15 ))

/*
 * Clock Disable macros for USARTx peripherals
 */

#define USART1_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 4 ))
#define USART2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 18))
#define USART6_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 5 ))

/*
 * Clock Disable macros for SYSCFGx peripherals
 */

#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 14 ))



//some generic macros

#define ENABLE  			1
#define DISABLE			 	0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET 		SET
#define GPIO_PIN_RESET 		RESET




#endif /* INC_STM32F407VG_H_ */






















