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



/**************************************START: PROCESSOR SPECIFIC DETAILS***********************************************8
 * ARM CORTEX Mx Processor NVIC ISERx Register Addresses
 */

#define NVIC_ISER0			((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1			((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2			((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3			((__vo uint32_t*)0xE000E10C)


/*
 * ARM Cortex Mx Processor Priority Register Addresses Calculation
 */

#define NVIC_PR_BASE_ADDR	((__vo uint32_t*)0xE000E400)
#define NO_PR_BITS_IMPLEMENTED 4
/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0			((__vo uint32_t*)0xE000E11U)




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

#define PERIPH_BASE			0x40000000U //
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

#define TIM2_BASEADDR		(APB1PERIPH_BASE + 0x0000)
#define TIM3_BASEADDR		(APB1PERIPH_BASE + 0x0400)
#define TIM4_BASEADDR		(APB1PERIPH_BASE + 0x0800)
#define TIM5_BASEADDR 		(APB1PERIPH_BASE + 0x0C00)
#define TIM6_BASEADDR 		(APB1PERIPH_BASE + 0x1000)
#define TIM7_BASEADDR 		(APB1PERIPH_BASE + 0x1400)
#define TIM12_BASEADDR 		(APB1PERIPH_BASE + 0x1800)
#define TIM13_BASEADDR  	(APB1PERIPH_BASE + 0x1C00)
#define TIM14_BASEADDR 		(APB1PERIPH_BASE + 0x2000)
#define RTC_BASEADDR  		(APB1PERIPH_BASE + 0x2800)
#define WWDG_BASEADDR  		(APB1PERIPH_BASE + 0x2C00)
#define IWDG_BASEADDR  		(APB1PERIPH_BASE + 0x3000)
#define I2S2ext_BASEADDR  	(APB1PERIPH_BASE + 0x3400)
#define SPI2_BASEADDR  		(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR  		(APB1PERIPH_BASE + 0x3C00)
#define I2S3ext_BASEADDR  	(APB1PERIPH_BASE + 0x4000)
#define USART2_BASEADDR  	(APB1PERIPH_BASE + 0x4400)  // Can work in synchronous and ASsyncronous
#define USART3_BASEADDR  	(APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR  	(APB1PERIPH_BASE + 0x4C00)  // Doesnt support synchronouse
#define UART5_BASEADDR  	(APB1PERIPH_BASE + 0x5000)
#define I2C1_BASEADDR 		(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR		(APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR 		(APB1PERIPH_BASE + 0x5C00)
#define CAN1_BASEADDR  		(APB1PERIPH_BASE + 0x6400)
#define CAN2_BASEADDR  		(APB1PERIPH_BASE + 0x6800)
#define CAN3_BASEADDR  		(APB1PERIPH_BASE + 0x7000)
#define CAN4_BASEADDR  		(APB1PERIPH_BASE + 0x7400)
#define UART7_BASEADDR  	(APB1PERIPH_BASE + 0x7800)
#define UART8_BASEADDR  	(APB1PERIPH_BASE + 0x7C00)


/*
 * BASE ADDRESess of Peripherals which are on APB2 Bus
 *
 */



#define TIM1_BASEADDR 		(APB2PERIPH_BASE + 0x0000)
#define TIM8_BASEADDR 		(APB2PERIPH_BASE + 0x0400)
#define USART1_BASEADDR		(APB2PERIPH_BASE + 0x1000)
#define USART6_BASEADDR		(APB2PERIPH_BASE + 0x1400)
#define ADC1_2_3_BASEADDR	(APB2PERIPH_BASE + 0x2000)
#define SDIO_BASEADDR		(APB2PERIPH_BASE + 0x2C00)
#define SPI1_BASEADDR		(APB2PERIPH_BASE + 0x3000)
#define SPI4_BASEADDR		(APB2PERIPH_BASE + 0x3400)
#define SYSCFG_BASEADDR		(APB2PERIPH_BASE + 0x3800)
#define EXTI_BASEADDR 		(APB2PERIPH_BASE + 0x3C00)
#define TIM9_BASEADDR 		(APB2PERIPH_BASE + 0x4000)
#define TIM10_BASEADDR 		(APB2PERIPH_BASE + 0x4400)
#define TIM11_BASEADDR 		(APB2PERIPH_BASE + 0x4800)
#define SPI5_BASEADDR 		(APB2PERIPH_BASE + 0x5000)
#define SPI6_BASEADDR 		(APB2PERIPH_BASE + 0x5400)
#define SAI1_BASEADDR 		(APB2PERIPH_BASE + 0x5800)
#define LCD_TFT_BASEADDR 	(APB2PERIPH_BASE + 0x6800)



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
 * Peripheral register definitnion for EXTI
 */
typedef struct
{
	__vo uint32_t IMR;	    // /*!< TODO Give short disctiption
	__vo uint32_t EMR;		// TODO
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;

}EXTI_RegDef_t;


/*
 * Peripheral register definition for SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP;	    // /*!< TODO Give short disctiption
	__vo uint32_t PMC;		// TODO
	__vo uint32_t EXTICR[4];
	__vo uint32_t CMPCR;

}SYSCFG_RegDef_t;

/*
 * Peripheral register definition for SPI
 */
typedef struct
{
	uint32_t SPI_CR1; 		/*!< possible values from @SPI Control Register 1 >*/
	uint32_t SPI_CR2;   		/*!< possible values from @SPI Control Register 2 >*/
	uint32_t SPI_SR; 		/*!< possible values from @SPI Status Register >*/
	uint32_t SPI_DR;			/*!< possible values from @SPI Data Register>*/
	uint32_t SPI_CRCPR;		/*!< possible values from @SPI CRC Polynomial register >*/
	uint32_t SPI_RXCRCR;		/*!< possible values from @SPI RX CRC Register >*/
	uint32_t SPI_TXCRCR;		/*!< possible values from @SPI TX CRC Register >*/
	uint32_t SPI_I2SCFGR;	/*!< possible values from @SPI_I2S configuration register >*/
	uint32_t SPI_I2SPR;		/*!< possible values from @SPI_I2S prescaler register >*/

}SPI_RegDef_t;


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

#define EXTI			((EXTI_RegDef_t*)EXTI_BASEADDR)

#define RCC				((RCC_RegDef_t*)RCC_BASEADDR)

#define SYSCFG  		((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1				((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2				((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3				((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4				((SPI_RegDef_t*)SPI4_BASEADDR)
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

/*
 * Macros to reset GPIO peripherals
 */

#define GPIOA_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 0));   (RCC->AHB1RSTR &= ~(1 << 0));  } while(0)
#define GPIOB_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 1));   (RCC->AHB1RSTR &= ~(1 << 1));  } while(0)
#define GPIOC_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 2));   (RCC->AHB1RSTR &= ~(1 << 2));  } while(0)
#define GPIOD_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 3));   (RCC->AHB1RSTR &= ~(1 << 3));  } while(0)
#define GPIOE_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 4));   (RCC->AHB1RSTR &= ~(1 << 4));  } while(0)
#define GPIOF_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 5));   (RCC->AHB1RSTR &= ~(1 << 5));  } while(0)
#define GPIOG_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 6));   (RCC->AHB1RSTR &= ~(1 << 6));  } while(0)
#define GPIOH_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 7));   (RCC->AHB1RSTR &= ~(1 << 7));  } while(0)
#define GPIOI_REG_RESET()   do{ (RCC->AHB1RSTR |= (1 << 8));   (RCC->AHB1RSTR &= ~(1 << 8));  } while(0)

/*
 * Macros to reset SPI peripherals
 */

#define SPI1_REG_RESET()   do{ (RCC->APB2RSTR |= (1 << 12));   (RCC->APB2RSTR &= ~(1 << 12));  } while(0)
#define SPI2_REG_RESET()   do{ (RCC->APB1RSTR |= (1 << 14));   (RCC->APB1RSTR &= ~(1 << 14));  } while(0)
#define SPI3_REG_RESET()   do{ (RCC->APB1RSTR |= (1 << 15));   (RCC->APB1RSTR &= ~(1 << 15));  } while(0)


/*
 * Returns Portcode for given GPIOx Base addresss
 * Uses conditonal or Ternary Operator
 */

#define GPIO_BASEADDR_TO_CODE(x)		((x==GPIOA)? 0:\
										 (x==GPIOB)? 1:\
										 (x==GPIOC)? 2:\
										 (x==GPIOD)? 3:\
										 (x==GPIOE)? 4:\
										 (x==GPIOF)? 5:\
										 (x==GPIOG)? 6:\
										 (x==GPIOH)? 7:\
										 (x==GPIOI)? 8:0)


/*
 *  IRQ ( Interrupt Request) Number of STM32F407x
 */

#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI5_10			40

#define NVIC_IRQ_PRI0			0
#define NVIC_IRQ_PRI1			1
#define NVIC_IRQ_PRI2			2
#define NVIC_IRQ_PRI3			3
#define NVIC_IRQ_PRI4			4
#define NVIC_IRQ_PRI5			5
#define NVIC_IRQ_PRI6			6
#define NVIC_IRQ_PRI7			7
#define NVIC_IRQ_PRI8			8
#define NVIC_IRQ_PRI9			9
#define NVIC_IRQ_PRI10			10
#define NVIC_IRQ_PRI11			11
#define NVIC_IRQ_PRI12			12
#define NVIC_IRQ_PRI13			13
#define NVIC_IRQ_PRI14			14
#define NVIC_IRQ_PRI15			15
#define NVIC_IRQ_PRI16			16
#define NVIC_IRQ_PRI17			17
#define NVIC_IRQ_PRI18			18
#define NVIC_IRQ_PRI19			19
#define NVIC_IRQ_PRI20			20
#define NVIC_IRQ_PRI21			21
#define NVIC_IRQ_PRI22			22
#define NVIC_IRQ_PRI23			23
#define NVIC_IRQ_PRI24			24
#define NVIC_IRQ_PRI25			25
#define NVIC_IRQ_PRI26			26
#define NVIC_IRQ_PRI27			27
#define NVIC_IRQ_PRI28			28
#define NVIC_IRQ_PRI29			29
#define NVIC_IRQ_PRI30			30
#define NVIC_IRQ_PRI31			31
#define NVIC_IRQ_PRI32			32
#define NVIC_IRQ_PRI33			33
#define NVIC_IRQ_PRI34			34
#define NVIC_IRQ_PRI35			35
#define NVIC_IRQ_PRI36			36
#define NVIC_IRQ_PRI37			37
#define NVIC_IRQ_PRI38			38
#define NVIC_IRQ_PRI39			39
#define NVIC_IRQ_PRI40			40
#define NVIC_IRQ_PRI41			41
#define NVIC_IRQ_PRI42			42
#define NVIC_IRQ_PRI43			43
#define NVIC_IRQ_PRI44			44
#define NVIC_IRQ_PRI45			45
#define NVIC_IRQ_PRI46			46
#define NVIC_IRQ_PRI47			47
#define NVIC_IRQ_PRI48			48
#define NVIC_IRQ_PRI49			49
#define NVIC_IRQ_PRI50			50

//some generic macros

#define ENABLE  			1
#define DISABLE			 	0
#define SET					ENABLE
#define RESET				DISABLE
#define GPIO_PIN_SET 		SET
#define GPIO_PIN_RESET 		RESET
#define BTN_PRESSED			1
#define FLAG_SET			1
#define FLAG_RESET			0



#endif /* INC_STM32F407VG_H_ */



/*****************************************************************************
 * Bit position definitions for SPI
 *****************************************************************************/
/*
 * Bit Position definition SPI_CR1
 */

#define SPI_CR1_CPHA 		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

/*
 * Bit Position definition SPI_CR2
 */

#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7


/*
 * Bit position definition SPI_SR
 */

#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8












































