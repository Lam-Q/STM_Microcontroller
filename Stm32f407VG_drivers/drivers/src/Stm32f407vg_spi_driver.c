/*
 * Stm32f407vg_spi_driver.c
 *
 *  Created on: Jul 4, 2022
 *      Author: lamqi
 */

#include "stm32F407vg_spi_driver.h"


/*
 *  Peripheral clock setup
 */
/*************************************************************************
 * @fn   			- SPI_PeriClock Control
 *
 * @param[in]		- base address of the SPI peripheral
 * @param[in]		- ENABLE or DISABLE macros
 * @param[in]
 *
 * @return			- none
 *
 * @Note			- none
 *
 ***************************************************************************/

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if (pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if (pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
	}
}
/*
 * SPI init and De-init
 */
/*************************************************************************
 * @fn   			- SPI_Init
 *
 * @param[in]		- Handle of the SPI Peripheral
 * @param[in]		-
 * @param[in]
 *
 * @return			- none
 *
 * @Note			- none
 *
 ***************************************************************************/

void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//first lets configure the SPI_CR1 register

	uint32_t tempreg=0;

	//1. configure the device mode
	tempreg |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. configure the bus config

	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BuS_CONFIG_HD)
	{
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BuS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDI mode Should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		//RXONLY Bit must be set
		tempreg |= (1<< SPI_CR1_RXONLY);
	}

	//3. Configure the SPI Serial Clock Speed (Baud Rate)
	tempreg |= pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR;

	//4. Configure the SPI DFF
	tempreg |= pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_DFF;

	//5. Configure the SPI CPOL
	tempreg |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;

	//6. Configure the SPI CPHA
	tempreg |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;

	pSPIHandle->pSPIx->CR1 |= tempreg;
}

/*************************************************************************
 * @fn   			- SPI_Deinit
 *
 * @param[in]		- BASE ADDRESS of the SPI Peripheral
 * @param[in]		-
 * @param[in]
 *
 * @return			- none
 *
 * @Note			- none
 *
 ***************************************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx)//use Peripheral RST register
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if (pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if (pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
}
/*
 * Data Send and Receive
 */

/*************************************************************************
 * @fn   			- SPI_SendData
 *
 * @param[in]		- BASE ADDRESS of the SPI Peripheral
 * @param[in]		- pTxBuffer
 * @param[in]		- Length of the message
 *
 * @return			- none
 *
 * @Note			- This is a blocking call
 *
 ***************************************************************************/

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while( Len > 0 )
	{
		//1. wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET);

		//2. Check the DFF bit in CR1
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF) )
		{
			//16 bit logic
			//1. Load the data in to the DR
			pSPIx->SPI_DR = *((uint16_t*)pTxBuffer);
			Len--; // decrement length for each byte. 16= 2 bytes so decrement twice
			Len--;
			(uint16_t*)pTxBuffer++;
		}else
		{
			//8 bit logic
			pSPIx->SPI_DR = *pTxBuffer;
			Len--;
		}
	}
}
/*************************************************************************
 * @fn   			- SPI_ReceiveData
 *
 * @param[in]		- BASE ADDRESS of the SPI Peripheral
 * @param[in]		- pRxBuffer
 * @param[in]		- Length of the message
 *
 * @return			- none
 *
 * @Note			- none
 *
 ***************************************************************************/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{

}


/*
 * IRQ Configuration and ISR Handling
 */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityHandling(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(SPI_Handle_t *pHandle);



uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if (pSPIx->SPI_SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}






















































