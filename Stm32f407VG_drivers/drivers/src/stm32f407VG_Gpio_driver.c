/*
 * stm32f407VG_Gpio_driver.c
 *
 *  Created on: Jun 25, 2022
 *      Author: lamqi
 */

#include "stm32F407vg_gpio_driver.h"


/*
 *  Peripheral clock setup, GPIO_RegDef_t = base address of GPIOA
 */
/*************************************************************************
 * @fn   			- GPIO_PeriClock Control
 *
 * @param[in]		- base address of the gpio peripheral
 * @param[in]		- ENABLE or DISABLE macros
 * @param[in]
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDI)
{
	if(EnorDI == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
		else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}

	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
		else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}
}



/*
 * GPIO init
 */
/*************************************************************************
 * @fn   			- GPIO_Init
 *
 * @param[in]		- Handle of the GPIO peripheral
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint8_t temp;
	//1. Configure the Mode of Gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// the non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else
	{
		// this part will code later. (Interrupt mode)
	}

	temp = 0;
	//2. Configure the speed
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->OSPEEDR |= temp;

		temp = 0;

	//3. configure the pupd setting

		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->PUPDR |= temp;

		temp = 0;

	//4. configure the optype

		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->OTYPER |= temp;

		temp = 0;
	//5. configure the alt functionality
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode)
		{
			uint8_t temp1, temp2;

			temp1= pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
			temp2= pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
			pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode <<4*temp2);
		}
}



/*************************************************************************
 * @fn   			- GPIO_DeInit
 *
 * @param[in]		- Base address of GPIO Peripheral
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) //use Peripheral RST register
{

}

/*
 * Data Read and Write
 */

/*************************************************************************
 * @fn   			- GPIO_ReadFromInputPin
 *
 * @param[in]		- Base address of GPIO Peripheral
 * @param[in]		- Pin Number to Read from
 * @oaran[in]
 *
 * @return			- returns the Boolean value from the Pin
 *
 * @Note			- none
 *
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

}

/*************************************************************************
 * @fn   			- GPIO_ReadFromInputPort
 *
 * @param[in]		- Base address of GPIO Peripheral
 * @param[in]		-
 *
 * @return			- returns the Value from the Input Port (contains 2 bytes thus uint16_t)
 *
 * @Note			- none
 *
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{

}

/*************************************************************************
 * @fn   			- GPIO_WriteToOutputPin
 *
 * @param[in]		- Base address of GPIO Peripheral
 * @param[in]		- Pin Number of Targeted Pin
 * @param[in]		- Value to write to Chosen Pin
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{

}

/*************************************************************************
 * @fn   			- GPIO_WriteToOutputPort
 *
 * @param[in]		- Base address of GPIO Peripheral
 * @param[in]		- Value to write to Chosen Port
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{

}

/*************************************************************************
 * @fn   			- GPIO_ToggleOutputPin
 *
 * @param[in]		- Base address of GPIO Peripheral
 * @param[in]		- Pin Number of Targeted Pin
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{

}

/*
 * IRQ Configuration and ISR Handling
 */
/*************************************************************************
 * @fn   			- GPIO_IRQConfig
 *
 * @param[in]		- Interrupt Request Number
 * @param[in]		- Interrupt Priority
 * @param[in]		- ENABLE or DISABLE Macro
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi)
{

}

/*************************************************************************
 * @fn   			- GPIO_IRQHandling
 *
 * @param[in]		- Pin Number to call interrupt
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{

}