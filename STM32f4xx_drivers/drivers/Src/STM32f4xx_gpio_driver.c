/*
 * STM32f4xx_gpio_driver.c
 *
 *  Created on: May 13, 2021
 *      Author: lamqi
 *  Purpose = A GPIO driver provides a configuration structure for the user
 */

#include "STM32f4xx_gpio_driver.h"

/*
 * Peripheral Clock setup
 */

/*
 *
 * @fn - GPIO_PeriClockControl
 *
 * @Brief -  This function enables or disables peripheral clock for the given GPIO Ports
 *
 *
 * @param[in]  - Base address of the GPIO Peripheral
 * @param[in]  - Enable or disable Macro
 * @param[in]  -
 *
 * @return  - none
 *
 * @Note - none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t ENorDI)
{
	if(ENorDI == ENABLE)
	{
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		} else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_EN();
		}
	}
	else
	{
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		} else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		} else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		} else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		} else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		} else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_DI();
		} else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_DI();
		} else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		} else if (pGPIOx == GPIOI) {
			GPIOI_PCLK_DI();
		}
	}


}

/*
 * GPIO Initialization and De-initialization
 */
/*
 *
 * @fn - GPIO_Init
 *
 * @Brief -  This function initializes the GPIO Port
 *
 *
 * @param[in]  - GPIO Base address
 * @param[in]  - Enable or disable Macro
 * @param[in]  -
 *
 * @return  - none
 *
 * @Note - none
 */

void GPIO_init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp= 0; // Temp Register
	uint32_t temp1 = 0;
	uint32_t temp2 = 0;
	// 1) configure the GPIO PinMode - 00 = Input, 01 = GPIO, 10 = Alt Function, 11 = Analog Mode

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
		{
			// the non Interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode  << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

		pGPIOHandle->pGPIOx->MODER &= ~(0x3 <<(2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp =0;

		}else
		{
			// Interrupt Mode
		}

	temp=0;

	// 2) configure the GPIO Pin Speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed  << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 <<(2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	// 3) Configure the GPIO Pin Pull up Pull down Control

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl  << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 <<(2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	// 4) Configure Output type

	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType  << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 <<( pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	// 5) Configure the Alt Functionality
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF <<(4* temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode  << (4* temp2));
		temp = 0;
		temp1 = 0;
		temp2 = 0;

	}



}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if (pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	} else if (pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	} else if (pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	} else if (pGPIOx == GPIOE) {
		GPIOE_REG_RESET();
	} else if (pGPIOx == GPIOF) {
		GPIOF_REG_RESET();
	} else if (pGPIOx == GPIOG) {
		GPIOG_REG_RESET();
	} else if (pGPIOx == GPIOH) {
		GPIOH_REG_RESET();
	} else if (pGPIOx == GPIOI) {
		GPIOI_REG_RESET();
	}
}

/*
 * Data Read and Data write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR>> PinNumber)& 0x00000001);

	return value;

}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
		uint16_t value;
		value = (uint16_t)(pGPIOx->IDR);

		return value;
}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
	if (value == GPIO_PIN_SET)
	{
    // Write 1
	pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
    // Write 0
	pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t value)
{
    // Write to GPIO Port
	pGPIOx->ODR = value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}
/*
 * IRQ Configuration and ISR Handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t ENorDI) // Interrupt configuration
{


}
void GPIO_IRQHandling(uint8_t PinNumber)   // Interrupt handling function
{

}



