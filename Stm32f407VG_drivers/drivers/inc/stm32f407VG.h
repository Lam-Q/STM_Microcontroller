/*
 * stm32f407VG.h
 *
 *  Created on: Jun 18, 2022
 *      Author: lamqi
 */

#ifndef INC_STM32F407VG_H_
#define INC_STM32F407VG_H_
#define __vo volatile
#define __weak __attribute__((weak))
#include <stdint.h>
#include <stddef.h>


#define NULL ( (void *) 0)

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
#define NVIC_ICER1			((__vo uint32_t*)0xE000E11U)



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
	__vo uint32_t SPI_CR1; 		/*!< possible values from @SPI Control Register 1 >*/
	__vo uint32_t SPI_CR2;   		/*!< possible values from @SPI Control Register 2 >*/
	__vo uint32_t SPI_SR; 		/*!< possible values from @SPI Status Register >*/
	__vo uint32_t SPI_DR;			/*!< possible values from @SPI Data Register>*/
	__vo uint32_t SPI_CRCPR;		/*!< possible values from @SPI CRC Polynomial register >*/
	__vo uint32_t SPI_RXCRCR;		/*!< possible values from @SPI RX CRC Register >*/
	__vo uint32_t SPI_TXCRCR;		/*!< possible values from @SPI TX CRC Register >*/
	__vo uint32_t SPI_I2SCFGR;	/*!< possible values from @SPI_I2S configuration register >*/
	__vo uint32_t SPI_I2SPR;		/*!< possible values from @SPI_I2S prescaler register >*/

}SPI_RegDef_t;

/*
 * Peripheral register definition for I2C
 */
typedef struct
{
	__vo uint32_t I2C_CR1; 		/*!< possible values from @SPI Control Register 1 >*/
	__vo uint32_t I2C_CR2;   		/*!< possible values from @SPI Control Register 2 >*/
	__vo uint32_t I2C_OAR1; 		/*!< possible values from @SPI Status Register >*/
	__vo uint32_t I2C_OAR2;			/*!< possible values from @SPI Data Register>*/
	__vo uint32_t I2C_DR;		/*!< possible values from @SPI CRC Polynomial register >*/
	__vo uint32_t I2C_SR1;		/*!< possible values from @SPI RX CRC Register >*/
	__vo uint32_t I2C_SR2;		/*!< possible values from @SPI TX CRC Register >*/
	__vo uint32_t I2C_CCR;	/*!< possible values from @SPI_I2S configuration register >*/
	__vo uint32_t I2C_TRISE;		/*!< possible values from @SPI_I2S prescaler register >*/
	__vo uint32_t I2C_FLTR;		/*!< possible values from @SPI_I2S prescaler register >*/

}I2C_RegDef_t;


/*
 * Peripheral Register definition for USART/UART
 */

typedef struct
{
	__vo uint32_t USART_SR; 		/*!< possible values from @USART Status Register >*/
	__vo uint32_t USART_DR;   		/*!< possible values from @USART Data Register >*/
	__vo uint32_t USART_BRR; 		/*!< possible values from @USART Baud Rate Register >*/
	__vo uint32_t USART_CR1;		/*!< possible values from @USART Control Register 1>*/
	__vo uint32_t USART_CR2;		/*!< possible values from @USART Control Register 2>*/
	__vo uint32_t USART_CR3;		/*!< possible values from @USART Control Register 3>*/
	__vo uint32_t USART_GTPR;		/*!< possible values from @USART Guard Time and Prescaler Register >*/
}USART_RegDef_t;



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

#define SPI1			((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2			((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3			((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4			((SPI_RegDef_t*)SPI4_BASEADDR)

#define I2C1			((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2			((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3			((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1			((USART_RegDef_t*)USART1_BASEADDR)
#define USART2			((USART_RegDef_t*)USART2_BASEADDR)
#define USART3			((USART_RegDef_t*)USART3_BASEADDR)
#define UART4			((USART_RegDef_t*)UART4_BASEADDR)
#define UART5			((USART_RegDef_t*)UART5_BASEADDR)
#define USART6			((USART_RegDef_t*)USART6_BASEADDR)


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
 * Macros to reset I2C peripherals
 */

#define I2C1_REG_RESET()   do{ (RCC->APB1RSTR |= (1 << 21));   (RCC->APB1RSTR &= ~(1 << 21));  } while(0)
#define I2C2_REG_RESET()   do{ (RCC->APB1RSTR |= (1 << 22));   (RCC->APB1RSTR &= ~(1 << 22));  } while(0)
#define I2C3_REG_RESET()   do{ (RCC->APB1RSTR |= (1 << 23));   (RCC->APB1RSTR &= ~(1 << 23));  } while(0)


/*
 * Macros to reset USART peripherals
 */

#define USART1_REG_RESET()   do{ (RCC->APB2RSTR |= (1 << 4));   (RCC->APB1RSTR &= ~(1 << 4));   } while(0)
#define USART2_REG_RESET()   do{ (RCC->APB1RSTR |= (1 << 17));  (RCC->APB1RSTR &= ~(1 << 17));  } while(0)
#define USART3_REG_RESET()   do{ (RCC->APB1RSTR |= (1 << 18));  (RCC->APB1RSTR &= ~(1 << 18));  } while(0)
#define USART6_REG_RESET()   do{ (RCC->APB1RSTR |= (1 << 5));   (RCC->APB1RSTR &= ~(1 << 5));   } while(0)


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
#define IRQ_NO_SPI1				35
#define IRQ_NO_SPI2				36
#define IRQ_NO_SPI3				51

#define IRQ_NO_I2C1_EV			31
#define IRQ_NO_I2C1_ER			32
#define IRQ_NO_I2C2_EV			33
#define IRQ_NO_I2C2_ER			34

#define IRQ_NO_USART1			37
#define IRQ_NO_USART2			38
#define IRQ_NO_USART3			39
#define IRQ_NO_UART4			52
#define IRQ_NO_UART5			53






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

/*****************************************************************************
 * Bit position definitions for I2C
 *****************************************************************************/
/*
 * Bit Position definition I2C_CR1
 */

#define I2C_CR1_PE			0		//1 = Enables Peripheral
#define I2C_CR1_SMBUS		1		//1 = SMBus Mode, 0 = I2C mode
#define I2C_CR1_DONOTUSE	2		// RESERVED MUST BE KEPT at reset value
#define I2C_CR1_SMBTYPE		3		// 1= SMBus Host, 0 = SMBus Device
#define I2C_CR1_ENARP		4		// 1 = ARP enable, 0 = ARP Disable
#define I2C_CR1_ENPEC		5		// 1 = PEC Calc Enable, 0 = PEC calc disable
#define I2C_CR1_ENGC		6		// 1 = General Call Enabled ACked, 0 = address 00h is NAck
#define I2C_CR1_NOSTRETCH	7		// 1 = Clock Stretching Disabled
#define I2C_CR1_START		8		// 1 = Start Signal Generation
#define I2C_CR1_STOP		9		// 1 = Stop Generation
#define I2C_CR1_ACK			10		// 1 = Acknowledge return after a byte is received, 0 = No Ack
#define I2C_CR1_POS			11		// 1 = Ack Bit controls next byte, 0 = Ack bit controls current Byte
#define I2C_CR1_PEC			12		// 1 = PEC Transfer, 0 = No PEC Transfer
#define I2C_CR1_BIDIMODE	13		// 1 = Drives SMBA pin low, 0 = Release SMBA pin High
#define I2C_CR1_DONOTUSE1	14		// RESERVED MUST BE KEPT at reset value
#define I2C_CR1_SWRST		15		// 1 = I2C Peripheral under reset state, 0 = Not Reset

/*
 * Bit Position definition I2C_CR2
 */

#define I2C_CR2_FREQ		0		// Set PCLock Freq, Minimum 2Mhz
#define I2C_CR2_ITERREN		8		// 1 = Error Interrupt Enabled, 0 = Disabled
#define I2C_CR2_ITEVTEN		9		// 1 = Event Interrupt Enabled, 0 = Disabled
#define I2C_CR2_ITBUFFEN	10		// 1 = TxE=1 or RxE=1 generates interrupt, 0 = Dont generate
#define I2C_CR2_DMAEN		11		// 1 = DMA Request Enabled when TxE=1/RxE=1, 0 = disabled
#define I2C_CR2_LAST		12		// 1 = Next DMA EOT is last transfer, 0 = next DMA not last



/*
 * Bit position definition I2C_SR1
 */
#define I2C_SR1_SB			0		// 1 = Start Condition Generated, 0 = No start
#define I2C_SR1_ADDR		1		// 1 = Received Address matched, 0 = Addr Mismatc
#define I2C_SR1_BTF			2		// 1 = Data Byte Transfer succeeded, 0 = Data Byte not done
#define I2C_SR1_ADD10		3		// 1 = Master has sent first addr byte, 0 = No ADD10 Event ocurred
#define I2C_SR1_STOPF		4		// 1 = Stop condition detected, 0 = No Stop Condition detected
#define I2C_SR1_RxNE		6		// 1 = Data Register Not Empty, 0 = Data Register Empty
#define I2C_SR1_TxE			7		// 1 = Data register empty, 0 = Data register Not Empty
#define I2C_SR1_BERR		8		// 1 = Misplaced Start or Stop Condition
#define I2C_SR1_ARLO		9		// 1 = Arbitration Lost detectected
#define I2C_SR1_AF			10		// 1 = Acknowledge Failure, 0 = No Ack failure
#define I2C_SR1_OVR			11		// 1 = Ovverrun or underrun, 0 = No Overrun or underrun
#define I2C_SR1_PECERR		12		// 1 = PEC Error, 0 = PEC no Error
#define I2C_SR1_TIMEOUT		14		// 1 = SCL Remained LOW FOR 25ms, 0 No timeout error
#define I2C_SR1_SMBALERT	15		// 1(master) = SMBUS alert occured on pin, 1(master) = SBAlert Response address header to SMBALERT LOW received.


/*
 * Bit position definition I2C_SR2
 */

#define I2C_SR2_MSL			0		// 1 = Master Mode
#define I2C_SR2_BUSY		1		// 1 = Received Address matched, 0 = Addr Mismatch
#define I2C_SR2_TRA			2		// 1 = Data Bytes transmitted
#define I2C_SR2_GENCALL		4		// 1 = General Call address received when ENGC = 1
#define I2C_SR2_SMBDEFAULT	5		// 1 = SMBUS Device Default address received when ENARP = 1
#define I2C_SR2_SMBHOST		6		// 1 = SMBUS Host address received when SMBTYE = 1,0 = No SMBUS Host addr
#define I2C_SR2_DUALF		7		// 1 = Received addr matched with OAR2, 0=Received addr matched with OAR1
#define I2C_SR2_PEC			8		// Packet error checking register

/*
 * Bit position definition I2C_SR2
 */
#define I2C_CCR_CCR			0		//
#define I2C_CCR_DUTY		14
#define I2C_CCR_FS			15


/*****************************************************************************
 * Bit position definitions for USART/UART
 *****************************************************************************/

/*
 * Bit position definition USART_SR
 */

#define USART_SR_PE			0		// Parity Error Bit, 1 = Parity Error
#define USART_SR_FE			1		// Framing Error, 1 = Framing error or break char detected
#define USART_SR_NF			2 		// 1 = noise flag detected
#define USART_SR_ORE		3 		// 1 = Overrun Error is detected
#define USART_SR_IDLE		4		// 1 = Idle line is detected
#define USART_SR_RXNE		5		// 1 = Received data is ready to be read.
#define USART_SR_TC			6 		// 1 = Transmission is completed
#define USART_SR_TXE		7		// 1 = Data is transferred to the shift register
#define USART_SR_LBD		8		// 1 = LIN Break detected
#define USART_SR_CTS		9		// 1 = A change is detected on the CTS status line

/*
 * USART Baud Rate Register
 */

#define USART_BRR_DIV_Fraction	0		// Bit3:0 = DIV_Fraction[3:0]
#define USART_BRR_DIV_Mantissa [11:0]	// Bit 15:4 = USARTDIV

/*
 *  USART Control Register 1
 */
#define USART_CR1_SBK		0		// 1 =  Break character will be transmitted
#define USART_CR1_RWU		1		// 1 = Receiver in mute mode
#define USART_CR1_RE		2		// 1 = Receiver is enbabled and begins searching for start bit
#define USART_CR1_TE		3		// 1 = Transimtter is enabled
#define USART_CR1_IDLEIE	4		// 1 =  AN USART Interrupt is genearted when IDLE=1
#define USART_CR1_RXNEIE	5		// 1 = An USART INterrupt is generated whenever ORE = 1 or RXNE =1
#define USART_CR1_TCIE		6 		// 1 = INterrupt generated when TC = 1
#define USART_CR1_TXEIE		7		// 1 = Interrupt generated when TXE = 1
#define USART_CR1_PEIE		8 		// 1 = INterrupt generated when PE = 1
#define USART_CR1_PS		9		// 1 = Odd parity (Parity Selection)
#define USART_CR1_PCE		10 		// 1 = Parity control enabled
#define USART_CR1_WAKE		11 		// 1 = Address Mark, 0= idle Line ,(wake meethod)
#define USART_CR1_M			12		// Word Length, 1 =  9 data bits, 0 = 8 data bits
#define USART_CR1_UE		13		// 1 = USART Enabled
#define USART_CR1_OVER8		15		// 1 = OVER8


/*
 *  USART Control Register 2
 */

#define USART_CR2_ADD		0		// Bit3:0 == Address of USART Node
#define USART_CR2_LBDL		5 		// 1 = 11-bit break detection
#define USART_CR2_LBDIE		6		// 1 = interrupt generates when LBD = 1
#define USART_CR2_LBCL		8		// 1 = Clock pulse of last data bit is output to CK pin
#define USART_CR2_CPHA		9		// 1 = The second clock transition is the first data captured edge
#define USART_CR2_CPOL		10		// 1 = Steady High value on CK pin outside transmission window
#define USART_CR2_CLKEN		11		// 1 = Clock pin enabled
#define USART_CR2_STOP		12		// Bit13:12, stop bits (00->11 = 1, 0.5, 2, 1.5 )
#define USART_CR2_LINEN		13		// 1 = LIN Mode enabled

/*
 *  USART Control Register 3
 */

#define USART_CR3_EIE		0		// 1= Interrupt generated when DMAR = 1
#define USART_CR3_IREN		1		// 1 = IrDA enabled
#define USART_CR3_IRLP		2		// 1 = Low Power Mode
#define USART_CR3_HDSEL		3		// 1 = Half Duplex mode is selected
#define USART_CR3_NACK		4 		// 1 = NACK
#define USART_CR3_SCEN		5		// 1 = Smartcard Mode enabled
#define USART_CR3_DMAR		6		// 1 = DMA mode is enabled for recepted
#define USART_CR3_DMAT		7		// 1 = DMA mode is enabled for transmission
#define USART_CR3_RTSE		8		// 1 = RTS interrupt enabled
#define USART_CR3_CTSE		9		// 1 = CTS Mode enabled
#define USART_CR3_CTSIE		10		// 1 = An interrupt is generated when CTS = 1
#define USART_CR3_ONEBIT	11		// 1 = One sample bit method

/*
 *  USART Guard time and prescaler register
 */

#define USART_GTPR_PSC		0		// Bit7:0 = prescaler value
#define USART_GTPR_GT		1		// Bit15:8 = Guard time value



































