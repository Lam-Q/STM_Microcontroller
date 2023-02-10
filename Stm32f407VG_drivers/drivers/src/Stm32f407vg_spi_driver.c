/*
 * Stm32f407vg_spi_driver.c
 *
 *  Created on: Jul 4, 2022
 *      Author: lamqi
 */

#include "stm32F407vg_spi_driver.h"
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

/*
 *  SPI get Flag Status
 */
/*************************************************************************
 * @fn   			- SPI_GetFlagStatus
 *
 * @param[in]		- base address of the SPI peripheral
 * @param[in]		- Flagname
 * @param[in]
 *
 * @return			- none
 *
 * @Note			- none
 *
 ***************************************************************************/
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if (pSPIx->SPI_SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}




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

	// Peripheral clock enbale
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);


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
	tempreg |= pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF;

	//5. Configure the SPI CPOL
	tempreg |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;

	//6. Configure the SPI CPHA
	tempreg |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;

	//7. Configure the SPI SSM
	tempreg |= pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->SPI_CR1 = tempreg;
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
		if (pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF) )
		{
			//16 bit logic
			//1. Load the data in to the DR
			pSPIx->SPI_DR = *((uint16_t*)pTxBuffer);
			Len--; // decrement length for each byte. 16= 2 bytes so decrement twice
			Len--; // decrement twice due to 2 bytes
			(uint16_t*)pTxBuffer++; //increment the buffer
		}else
		{
			//8 bit logic
			pSPIx->SPI_DR = *pTxBuffer;
			Len--;
			pTxBuffer++; //increment the buffer
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
	while( Len > 0 )
	{
		//1. wait until RXNE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == (uint8_t)FLAG_RESET);

		//2. Check the DFF bit in CR1
		if (pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF) )
		{
			//16 bit logic
			//1. Load the data from DR to the RXBuffer
			*((uint16_t*)pRxBuffer) = pSPIx->SPI_DR;
			Len--; // decrement length for each byte. 16= 2 bytes so decrement twice
			Len--; // decrement twice due to 2 bytes
			(uint16_t*)pRxBuffer++; //increment the buffer
		}else
		{
			//8 bit logic
			*(pRxBuffer) = pSPIx->SPI_DR;
			Len--;
			pRxBuffer++; //increment the buffer
		}
	}
}




/*************************************************************************
 * @fn   			- SPI_PeripheralControl
 *
 * @param[in]		- BASE ADDRESS of the SPI Peripheral
 * @param[in]		- Enable or Disable
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 *
 ***************************************************************************/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->SPI_CR1 |= (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/*************************************************************************
 * @fn   			- SPI_SSIConfig
 *
 * @param[in]		- BASE ADDRESS of the SPI Peripheral
 * @param[in]		- Enable or Disable
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 *
 ***************************************************************************/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->SPI_CR1 |= (1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/*************************************************************************
 * @fn   			- SPI_SSOEConfig
 *
 * @param[in]		- BASE ADDRESS of the SPI Peripheral
 * @param[in]		- Enable or Disable
 * @param[in]		-
 *
 * @return			- none
 *
 * @Note			- none
 *
 ***************************************************************************/
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->SPI_CR2 |= (1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx->SPI_CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}


/*
 * IRQ Configuration and ISR Handling
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
void SPI_IRQPriorityHandling(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx= IRQNumber/4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8*iprx_section + 8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx)) |= (IRQPriority <<  shift_amount);

}


uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{

	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
	//1. Save the Tx Buffer address and Len information in some global variable
	pSPIHandle->pTxBuffer = pTxBuffer;
	pSPIHandle->TxLen = Len;


	//2. Mark the SPI state as busy in transmission so that
	// no other code can take over same SPI peripheral until transmission is over

	pSPIHandle->TxState = SPI_BUSY_IN_TX;

	//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
	pSPIHandle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_TXEIE);
	}


	return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
	//1. Save the Rx Buffer address and Len information in some global variable
	pSPIHandle->pRxBuffer = pRxBuffer;
	pSPIHandle->RxLen = Len;


	//2. Mark the SPI state as busy in transmission so that
	// no other code can take over same SPI peripheral until transmission is over

	pSPIHandle->RxState = SPI_BUSY_IN_TX;

	//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
	pSPIHandle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_RXNEIE);
	}


	return state;
}

//First have to decode what the source of interrupt is.
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{


	uint8_t temp1, temp2;

	// if TXE Flag
	temp1 = pHandle->pSPIx->SPI_SR & (1 <<SPI_SR_TXE);
	temp2 = pHandle->pSPIx->SPI_CR2 & (1<< SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		//handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	// if RXNE Flag

	temp1 = pHandle->pSPIx->SPI_SR & (1 <<SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->SPI_CR2 & (1<< SPI_CR2_RXNEIE);

	if(temp1 && temp2)
	{
		//handle TXE
		spi_rxe_interrupt_handle(pHandle);
	}

	// if Error Flag

	temp1 = pHandle->pSPIx->SPI_SR & (1 <<SPI_SR_OVR);
	temp2 = pHandle->pSPIx->SPI_CR2 & (1<< SPI_CR2_ERRIE);

	if(temp1 && temp2)
	{
		//handle TXE
		spi_ovr_err_interrupt_handle(pHandle);
	}



}




//some helper function implementation

static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
		//2. Check the DFF bit in CR1
		if (pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF) )
		{
			//16 bit logic
			//1. Load the data in to the DR
			pSPIHandle->pSPIx->SPI_DR = *((uint16_t*)pSPIHandle->pTxBuffer);
			pSPIHandle->TxLen--; // decrement length for each byte. 16= 2 bytes so decrement twice
			pSPIHandle->TxLen--; // decrement twice due to 2 bytes
			(uint16_t*)pSPIHandle->pTxBuffer++; //increment the buffer using 16 bit typecast
		}else
		{
			//8 bit logic
			pSPIHandle->pSPIx->SPI_DR = *pSPIHandle->pTxBuffer;
			pSPIHandle->TxLen--;
			pSPIHandle->pTxBuffer++; //increment the buffer
		}

		if (!pSPIHandle->TxLen)
		{
			pSPIHandle->pSPIx->SPI_CR2 &= ~(1 <<SPI_CR2_TXEIE);

			//reset the used Tx registers
			pSPIHandle->pTxBuffer = NULL;
			pSPIHandle->TxLen = 0;
			pSPIHandle->TxState = SPI_READY;
			SPI_ApplicationEventCallBack(pSPIHandle,SPI_EVENT_TX_CMPLT);
		}
}
static void spi_rxe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//2. Check the DFF bit in CR1
	if (pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF) )
	{
		//16 bit logic
		//1. Load the data from DR to the RXBuffer
		*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->SPI_DR;
		pSPIHandle->RxLen--; // decrement length for each byte. 16= 2 bytes so decrement twice
		pSPIHandle->RxLen--; // decrement twice due to 2 bytes
		(uint16_t*)pSPIHandle->pRxBuffer++; //increment the buffer
	}else
	{
		//8 bit logic
		*(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->SPI_DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++; //increment the buffer
	}
	if (!pSPIHandle->RxLen)
	{
		pSPIHandle->pSPIx->SPI_CR2 &= ~(1 <<SPI_CR2_RXNEIE);

		//reset the used Tx registers
		pSPIHandle->pRxBuffer = NULL;
		pSPIHandle->RxLen = 0;
		pSPIHandle->RxState = SPI_READY;
		SPI_ApplicationEventCallBack(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}
}
static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

	uint8_t temp;
	//1. clear the ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->SPI_DR;
		temp = pSPIHandle->pSPIx->SPI_DR;
	}
	(void)temp;
	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);

}




__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{

	//This is a weak implementation . the user application may override this function.
}












































