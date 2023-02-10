/*
 * stm32F407VG_I2C_driver.h
 *
 *  Created on: Jul 16, 2022
 *      Author: lamqi
 */

#ifndef INC_STM32F407VG_I2C_DRIVER_H_
#define INC_STM32F407VG_I2C_DRIVER_H_

#include "stm32f407VG.h"


typedef struct
{
	uint32_t I2C_SCLSpeed; 			/*!< possible values from @I2C_SCLSpeed>*/
	uint8_t  I2C_DeviceAddress;   	/*!< Based on User address >*/
	uint8_t  I2C_AckControl; 		/*!< possible values from @I2C_AckControl >*/
	uint8_t  I2C_FMDutyCycle;		/*!< possible values from @I2C_FMDutyCycle>*/


}I2C_Config_t;


typedef struct
{
	// pointer to hold the base address of the GPIO peripheral

	I2C_RegDef_t 	*pI2Cx;
	I2C_Config_t 	I2C_Config;
	uint8_t			*pTxBuffer;     /* !< To Store the app. Tx buffer address>*/
	uint8_t			*pRxBuffer;		/* !< To store the app. Rx buffer address>*/
	uint32_t		TxLen;  		/* !< To store Tx Len > */
	uint32_t		RxLen; 			/* !< To store Rx Len > */
	uint8_t			TxRxState;      /* !< To store Communication State > */
	uint8_t			DevAddr;		/* !< To store slave/device address > */
	uint32_t		RxSize;			/* !< To store Rx size > */
	uint8_t			Sr;				/* !< To store repeated Start Value > */

}I2CHandle_t;


// Options for Configuration I/O
/*
 * @I2C_SCLSpeed>
 */

#define I2C_SCL_SPEED_SM		100000
#define I2C_SCL_SPEED_FM4K		400000
#define I2C_SCL_SPEED_FM2K 		200000

/*
 * @I2C_AckControl
 */

#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0



/*
 * @I2C_FMDutyCycle
 */

#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1




/*
 * I2C Related Status Flag Definition
 */

//Status Flag 1
#define I2C_SB_FLAG			(1 << I2C_SR1_SB)
#define I2C_ADDR_FLAG		(1 << I2C_SR1_ADDR)
#define I2C_BTF_FLAG		(1 << I2C_SR1_BTF)
#define I2C_ADD10_FLAG		(1 << I2C_SR1_ADD10)
#define I2C_STOPF_FLAG		(1 << I2C_SR1_STOPF)
#define I2C_RxNE_FLAG		(1 << I2C_SR1_RxNE)
#define I2C_TxE_FLAG		(1 << I2C_SR1_TxE)
#define I2C_BERR_FLAG		(1 << I2C_SR1_BERR)
#define I2C_ARLO_FLAG		(1 << I2C_SR1_ARLO)
#define I2C_AF_FLAG			(1 << I2C_SR1_AF)
#define I2C_OVR_FLAG		(1 << I2C_SR1_OVR)
#define I2C_PECERR_FLAG		(1 << I2C_SR1_PECERR)
#define I2C_TIMEOUT_FLAG	(1 << I2C_SR1_TIMEOUT)
#define I2C_SMBALERT_FLAG	(1 << I2C_SR1_SMBALERT)

// Status Flag 2
#define I2C_MSL_FLAG			(1 << I2C_SR2_MSL)
#define I2C_BUSY_FLAG			(1 << I2C_SR2_BUSY)
#define I2C_TRA_FLAG			(1 << I2C_SR2_TRA)
#define I2C_GENCALL_FLAG		(1 << I2C_SR2_GENCALL)
#define I2C_SMBDEFAULT_FLAG		(1 << I2C_SR2_SMBDEFAULT)
#define I2C_SMBHOST_FLAG		(1 << I2C_SR2_SMBHOST)
#define I2C_DUALF_FLAG			(1 << I2C_SR2_DUALF)
#define I2C_PEC_FLAG			(1 << I2C_SR2_PEC)
/*
 *  Possible I2C Application States
 */
#define I2C_NO_SR RESET
#define I2C_ENABLE_SR 	  SET



/*
 * I2C application states
 */
#define I2C_READY 					0
#define I2C_BUSY_IN_RX 				1
#define I2C_BUSY_IN_TX 				2

/*
 * I2C application event macros
 */

#define I2C_EV_TX_CMPLT  	 	0
#define I2C_EV_RX_CMPLT  	 	1
#define I2C_EV_STOP       		2
#define I2C_ERROR_BERR 	 		3
#define I2C_ERROR_ARLO  		4
#define I2C_ERROR_AF    		5
#define I2C_ERROR_OVR   		6
#define I2C_ERROR_TIMEOUT 		7
#define I2C_EV_DATA_REQ         8
#define I2C_EV_DATA_RCV         9

/********************************************************************************************************
 * 										API's Supported by this driver
 * 						For more information about the APIs check the function definition
 *
 ********************************************************************************************************/

/*
 *  Peripheral clock setup, GPIO_RegDef_t = base address of GPIOA
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 * I2C init and De-init
 */
void I2C_Init(I2CHandle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx); //use Peripheral RST register

/*
 * Data Send and Receive
 */
void I2C_MasterSendData(I2CHandle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);
void I2C_MasterReceiveData(I2CHandle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);

uint8_t I2C_MasterSendDataIT(I2CHandle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2CHandle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);

void I2C_CloseSendData(I2CHandle_t *pI2CHandle);
void I2C_CloseReceiveData(I2CHandle_t *pI2CHandle);

void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);

/*
 * IRQ Configuration and ISR Handling
 */

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityHandling(uint8_t IRQNumber, uint32_t IRQPriority);
void I2C_EV_IRQHandling(I2CHandle_t *pI2CHandle);  // Event interrupt Handling
void I2C_ER_IRQHandling(I2CHandle_t *pI2CHandle);  // Error interrupt Handling


/*
 *  Other Peripheral Control APIs
 */
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
void I2C_ManageAcking( I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);


uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx,uint8_t EnOrDi);
void I2C_ApplicationEventCallback(I2CHandle_t *pI2CHandle,uint8_t AppEv);


void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);




#endif /* INC_STM32F407VG_I2C_DRIVER_H_ */
