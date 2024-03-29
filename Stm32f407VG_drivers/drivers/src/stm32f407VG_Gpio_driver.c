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
	uint32_t temp;

	//Enable the Peripheral Clock

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);



	//1. Configure the Mode of Gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// the non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// 1. Configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			// 1. Clear the corresponding RTSR bit (Only 1 edge dection at this time
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// 1. Configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			// 1. Clear the corresponding FTSR bit (Only 1 edge dection at this time
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			// 1. Configure the FTSR and RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			// 1. Clear the corresponding FTSR bit (Only 1 edge dection at this time
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		// 2. Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] |= portcode << (temp2*4);

		// 3. enable the exti interrupt delivery using IMR

			EXTI->IMR |= 1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);


	}

	temp = 0;
	//2. Configure the speed
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIOx->OSPEEDR |= temp;

		temp = 0;

	//3. configure the pupd setting

		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIOx->PUPDR |= temp;


		temp = 0;

	//4. configure the optype

		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIOx->OTYPER |= temp;

		temp = 0;
	//5. configure the alt functionality
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode)
		{
			uint8_t temp1, temp2;

			temp1= pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
			temp2= pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
			pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4*temp2) );
			pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode <<(4*temp2));
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

			if(pGPIOx == GPIOA)
			{

				GPIOA_REG_RESET();
			}
			else if (pGPIOx == GPIOB)
			{
				GPIOB_REG_RESET();
			}
			else if (pGPIOx == GPIOC)
			{
				GPIOC_REG_RESET();
			}
			else if (pGPIOx == GPIOD)
			{
				GPIOD_REG_RESET();
			}
			else if (pGPIOx == GPIOE)
			{
				GPIOE_REG_RESET();
			}
			else if (pGPIOx == GPIOF)
			{
				GPIOF_REG_RESET();
			}
			else if (pGPIOx == GPIOG)
			{
				GPIOG_REG_RESET();
			}
			else if (pGPIOx == GPIOH)
			{
				GPIOH_REG_RESET();
			}
			else if (pGPIOx == GPIOI)
			{
				GPIOI_REG_RESET();
			}

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
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
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
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
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
	if(Value == GPIO_PIN_SET)
	{
		//write 1 to output data register
		pGPIOx->ODR |=  (1 << PinNumber);
	}
	else
	{
		//write 0
		pGPIOx->ODR &=  ~(1 << PinNumber);
	}
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
	pGPIOx->ODR = Value;
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
	pGPIOx->ODR ^= (1 << PinNumber);
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
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

	if(EnorDi == ENABLE)
	{
		if (IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);

		}else if (IRQNumber >31 && IRQNumber < 64)  //32 to 64
		{
			//program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber%32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber%64));
		}
	}
	else
	{
		if (IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 &= ~(1 << IRQNumber);
		}else if (IRQNumber >31 && IRQNumber < 64)  //32 to 64
		{
			//program ISER1 register
			*NVIC_ISER1 &= ~(1 << (IRQNumber%32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			//program ISER2 register
			*NVIC_ISER2 &= ~(1 << (IRQNumber%64));
		}
	}
}



/*************************************************************************
 * @fn   			- GPIO_IRQPriorityHandling
 *
 * @param[in]		- IRQ Number
 * @param[in]		- IRQ Priority
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_IRQPriorityHandling(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx= IRQNumber/4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8*iprx_section + 8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx)) |= (IRQPriority <<  shift_amount);

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
	//clear the exti pr register corresponding to the pin number
	if (EXTI->PR & (1 <<PinNumber))
	{
		//clear
		EXTI->PR |= (1 << PinNumber);

	}
}
