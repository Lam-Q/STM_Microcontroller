/*
 * stm32F407VG_I2C_driver.c
 *
 *  Created on: Jul 16, 2022
 *      Author: lamqi
 */
/*
 *  Peripheral clock setup, GPIO_RegDef_t = base address of GPIOA
 */

#include "stm32F407vg_i2c_driver.h"
#include "stm32F407vg_rcc_driver.h"



/*
 ****************************** HELPER FUNCTIONS DEFINITION *******************************
 */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);

static void I2C_ClearADDR_Flag(I2CHandle_t *pI2CHandle);

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);
static void I2C_MasterHandleRXNEInterrupt(I2CHandle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2CHandle_t *pI2CHandle);
/*
 * HelperFunction Content
 */
void I2C_CloseReceiveData(I2CHandle_t *pI2CHandle)
{
	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITBUFFEN);
	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = RESET;
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->RxSize = RESET;
	if (pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
	I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}
}

void I2C_CloseSendData(I2CHandle_t *pI2CHandle)
{
	//disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITBUFFEN);
	//disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = RESET;
	pI2CHandle->TxRxState = I2C_READY;
}

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data)
{
	pI2Cx->I2C_DR = data;
}
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
	return (uint8_t)pI2Cx->I2C_DR;
}


static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_START);
}

void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr <<1;
	SlaveAddr &= ~(1);
	pI2Cx->I2C_DR = SlaveAddr;
}
void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = SlaveAddr <<1;
	SlaveAddr |= 1;  //Slave addr + r/w bit = 1,0 respectively
	pI2Cx->I2C_DR = SlaveAddr;
}

static void I2C_ClearADDR_Flag(I2CHandle_t *pI2CHandle)
{
	uint32_t dummyRead;
	//check for device mode
	if (pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL))
	{
		// device is in master mode

		if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if (pI2CHandle->RxSize == 1)
			{
				//first disable the ack
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				//clear the addr Flag
				dummyRead = pI2CHandle->pI2Cx->I2C_SR1;
				dummyRead = pI2CHandle->pI2Cx->I2C_SR2;
				(void)dummyRead;
			}
		}else
		{
			dummyRead = pI2CHandle->pI2Cx->I2C_SR1;
			dummyRead = pI2CHandle->pI2Cx->I2C_SR2;
			(void)dummyRead;
		}



	}else
	{
		dummyRead = pI2CHandle->pI2Cx->I2C_SR1;
		dummyRead = pI2CHandle->pI2Cx->I2C_SR2;
		(void)dummyRead;
	}

}

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_STOP);
}

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx,uint8_t EnOrDi)
{

		 if(EnOrDi == ENABLE)
		 {
				pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITEVTEN);
				pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITBUFFEN);
				pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITERREN);
		 }else
		 {
				pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITEVTEN);
				pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITBUFFEN);
				pI2Cx->I2C_CR2 &= ~( 1 << I2C_CR2_ITERREN);
		 }


}


uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{

	if (pI2Cx->I2C_SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}




void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}
		else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
		else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}

/*
 * I2C init and De-init
 */
void I2C_Init(I2CHandle_t *pI2CHandle)
{
	uint32_t tempreg = 0;

	// enable the peripheral Clock i2Cx
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	//ack control bit
	tempreg |= (pI2CHandle->I2C_Config.I2C_AckControl << I2C_CR1_ACK);
	pI2CHandle->pI2Cx->I2C_CR1 = tempreg;


	// configure the FREQ field of CR2
	tempreg = 0;
	tempreg |= RCC_GETPCLK1Value()/1000000U;
	pI2CHandle->pI2Cx->I2C_CR2 = (tempreg & 0x3F);

	//program the device own address
	tempreg = 0;
	tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress <<1;
	tempreg |= (1 << 14);
	pI2CHandle->pI2Cx->I2C_OAR1 = tempreg;

	// CCR calculation
	uint16_t ccr_value = 0;
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// mode is standard mode
		ccr_value = (RCC_GETPCLK1Value()/(2*pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempreg |= (ccr_value & 0xFFF);
	}
	else
	{
		//mode is fast mode
		tempreg |= (1 << I2C_CCR_FS);
		tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle <<14);
		if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			ccr_value = (RCC_GETPCLK1Value()/(3*pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}else
		{
			ccr_value = (RCC_GETPCLK1Value()/(25*pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}
		tempreg |= (ccr_value & 0xFFF);
	}
	pI2CHandle->pI2Cx->I2C_CCR = tempreg;

	//Trise Calculation
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// mode is standard mode

		tempreg = (RCC_GETPCLK1Value()/1000000U)+1;
	}else
	{
		// mode is fast mode
		tempreg = ((RCC_GETPCLK1Value()*300 )/1000000000U)+1;
	}
	pI2CHandle->pI2Cx->I2C_TRISE = (tempreg & 0x3F);
}

void I2C_DeInit(I2C_RegDef_t *pI2Cx) //use Peripheral RST register
{
	if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}
	else if (pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}
	else if (pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}
}

/*
 * Data Send and Receive
 */

void I2C_MasterSendData(I2CHandle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr, uint8_t Sr)
{
	//1. Generate the Start Condition
	I2C_GenerateStartCondition( pI2CHandle->pI2Cx);

	//2. Confirm that start generation is completed by checking the SB Flag in the SR1
	// Note: Until SB is cleared, SCL will be stretched (pulled to LOW)

	while(!I2C_GetFlagStatus( pI2CHandle->pI2Cx, I2C_SB_FLAG));

	//3. Send the address of the slave with r/nw bit set to w(0) (total 8 bits)

	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);


	//4. Confirm that address phase is completed by checking the ADDR flag in the SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR_FLAG));

	//5. Clear the ADDR flag according to its software sequence
	// Note: Until the ADDR is cleared SCL will be stretcched (PULLED TO LOW)

	I2C_ClearADDR_Flag(pI2CHandle);

	//6. Send the data until Len becomes 0

	while(Len >0)
	{
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TxE_FLAG)); //Wait till TXE is set
		pI2CHandle->pI2Cx->I2C_DR = *pTxbuffer;
		pTxbuffer++;
		Len--;

	}
	//7. When len becomes zero wait for TXE = 1 and BTF = 1 before generating the STOP condition
	// Note: TXE=1, BTF = 1, means that both SR and DR are empty and next transmission should begin
	// when BTF=1 SCL will be stretched (Pulled to LOW)

	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TxE_FLAG)); //Wait till TXE is set

	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_BTF_FLAG)); //Wait till TXE is set



	//8. Generate STOP Condition and master need not to wait for the completion of stop condition.
	// Note: Generating STOP, automatically clears the BTF


	if(Sr == I2C_NO_SR){
		I2C_GenerateStopCondition( pI2CHandle->pI2Cx);
	}
}


void I2C_MasterReceiveData(I2CHandle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	//1. Generate the Start Condition
	I2C_GenerateStartCondition( pI2CHandle->pI2Cx);
	//2. Confirm that start generation is completed by checking the SB flag in the SR1
	while(!I2C_GetFlagStatus( pI2CHandle->pI2Cx, I2C_SB_FLAG));
	//3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);
	//4. wait until address phase is completed by checking the ADDR flag in the SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR_FLAG));
	// procedure to read only 1 byte from slave

	if (Len == 1 )
	{
		// Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);



		// Clear the Addr Flag
		I2C_ClearADDR_Flag(pI2CHandle);

		// wait until RXNE becomes 1
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RxNE_FLAG));

		//generate STOP condition
		if (Sr == I2C_NO_SR){
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		// read data in to buffer
		*pRxbuffer = pI2CHandle->pI2Cx->I2C_DR;

	}
	if (Len > 1 )
	{
		// Clear the Addr Flag
		I2C_ClearADDR_Flag(pI2CHandle);

		// Read the data until Len becomes zero
		for (uint32_t i = Len; i > 0; i --)
		{

			// wait until RXNE becomes 1
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RxNE_FLAG));

			// if last 2 bytes are remaining
			if ( i == 2)
			{

				//clear ack bit
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				// generate Stop Condition

				if (Sr == I2C_NO_SR){
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
				}

			}

			// read data in to buffer
			*pRxbuffer = pI2CHandle->pI2Cx->I2C_DR;

			// increment the buffer address by 1 byte
			pRxbuffer++;
		}

	}

	// Re-enable acking
	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
	}

}

/*
 * IRQ Configuration and ISR Handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
void I2C_IRQPriorityHandling(uint8_t IRQNumber, uint32_t IRQPriority)
{
	uint8_t iprx= IRQNumber/4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8*iprx_section + 8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + (iprx)) |= (IRQPriority <<  shift_amount);

}



/*
 *  Other Peripheral Control APIs
 */

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{

		if(EnOrDi == ENABLE)
		{
			pI2Cx->I2C_CR1 |= (1 << I2C_CR1_PE);
			//pI2cBaseAddress->CR1 |= I2C_CR1_PE_Bit_Mask;
		}else
		{
			pI2Cx->I2C_CR1 &= ~(1 << 0);
		}


}

void I2C_ManageAcking( I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if (EnOrDi == I2C_ACK_ENABLE)
	{
		pI2Cx->I2C_CR1 |= (1 << I2C_CR1_ACK);
	}
	else
	{
		pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

/*********************************************************************
 * @fn      		  - I2C_MasterSendDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -  Complete the below code . Also include the function prototype in header file

 */

uint8_t I2C_MasterSendDataIT(I2CHandle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition( pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITBUFFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;

}


/*********************************************************************
 * @fn      		  - I2C_MasterReceiveDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Complete the below code . Also include the fn prototype in header file

 */
uint8_t I2C_MasterReceiveDataIT(I2CHandle_t *pI2CHandle ,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_FLAG) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITBUFFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

static void I2C_MasterHandleTXEInterrupt(I2CHandle_t *pI2CHandle)
{
	if(pI2CHandle->TxLen >0)
	{
			//1. Load the data into DR
			pI2CHandle->pI2Cx->I2C_DR = *(pI2CHandle->pTxBuffer);
			//2. Decrement the TxLen
			pI2CHandle->TxLen--;

			//3. Increment the Buffer address
			pI2CHandle->pTxBuffer++;
	}
}
static void I2C_MasterHandleRXNEInterrupt(I2CHandle_t *pI2CHandle)
{

	// 2 cases for data reception
	//RX Length == 1
	if(pI2CHandle->RxSize == 1)
	{
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
		pI2CHandle->RxLen--;

	}
	// RX length greater than 1
	if(pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxLen == 2)
		{
			//clear the ack bit
			I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
		}
		//Read DR
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}
	if(pI2CHandle->RxLen == 0)
	{
		//Close the I2C data recepetion and notify the application

		//1. Generate the stop condition
		if(pI2CHandle->Sr== I2C_NO_SR)
		{
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}
		//2. Close the I2C rx
		I2C_CloseReceiveData(pI2CHandle);

		//3. Notify the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}
void I2C_EV_IRQHandling(I2CHandle_t *pI2CHandle)
{
	// Interrupt handling for both master and slave mode of a device
	uint32_t temp1, temp2, temp3,dummyread1;

	temp1 = pI2CHandle->pI2Cx->I2C_CR2 & (1<< I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->I2C_CR2 & (1<< I2C_CR2_ITBUFFEN);
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1<< I2C_SR1_SB);


	//1. Handle for interrupt generated by SB event
	// Note: SB flag is only applicable in Master mode

	if(temp1 && temp3)
	{
		//The interrupt is generated because of SB event
		//This block will not be executed in slave mode because for slave SB is always zero
		//Now we need to execute the address phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
		else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}

	}

	//2. Handle for Interrupt generated by ADDR event
	// Note: When master mode : Address is sent
	// 		 When slave mode  : Address matched with own address
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1<< I2C_SR1_ADDR);
	if(temp1 && temp3)
	{
		I2C_ClearADDR_Flag(pI2CHandle);
	}


	//3. Handle For Interrupt generated by BTF (Byte Transfer Finished) event
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1<< I2C_SR1_BTF);
	if(temp1 && temp3)
	{
		//BTF Flag is set
		if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			if (pI2CHandle->pI2Cx->I2C_SR1 &(1 << I2C_SR1_TxE))
			{
				//BTF, TXE = 1  (Use BTF Flag to end the transmission)
				//1. generate the STOP Condition
				if (pI2CHandle->TxLen == 0)
				{
					if (pI2CHandle->Sr == I2C_NO_SR)
					{
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}
					//2. reset all the member elements of the handle structure.
					I2C_CloseSendData(pI2CHandle);

					//3. notify the application about transmission complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}

			}
		}
		else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			;
		}
	}
	//4. Handle For interrupt generated by STOPF event
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1<< I2C_SR1_STOPF);
	if(temp1 && temp3)
	{
		//STOPF Flag is set
		// Clear the stoPF (i.e 1) read SR1 2) Write to CR1)
		//dummyread1 = pI2CHandle->pI2Cx->I2C_SR1;
		pI2CHandle->pI2Cx->I2C_CR1 |= 0x0000;

		// Notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}
	// Note: Stop detection flag is applicable only slave mode. For master this flag will

	//5. Handle for interrupt generated by TXE event
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1<< I2C_SR1_TxE);
	if(temp1 && temp2 && temp3)
	{
		//check for device mode
		if(pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL))
		{
			//Txe Flag is set
			if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}
		else
		{
			//slave
			if(pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}
	//6. Handle for interrupt generated by RXE event
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1<< I2C_SR1_RxNE);
	if(temp1 && temp2 && temp3)
	{
		//check for device mode
		if(pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL))
		{
		//Rxne Flag is set
			if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}
		else
		{
			if(!(pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_TRA)))
			{
				//slave mode
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}






}
/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Complete the code also define these macros in the driver
						header file
						#define I2C_ERROR_BERR  3
						#define I2C_ERROR_ARLO  4
						#define I2C_ERROR_AF    5
						#define I2C_ERROR_OVR   6
						#define I2C_ERROR_TIMEOUT 7

 */

void I2C_ER_IRQHandling(I2CHandle_t *pI2CHandle)
{

	uint32_t temp1,temp2;

    //Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->I2C_CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_ARLO);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

	    //Implement the code to clear the ACK failure error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_AF);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_OVR);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is Time out error
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_TIMEOUT);
	    //Implement the code to clear the Time out error flag

		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}

}
