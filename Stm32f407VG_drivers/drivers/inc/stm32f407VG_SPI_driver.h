/*
 * stm32f407VG_SPI_driver.h
 *
 *  Created on: Jul 4, 2022
 *      Author: lamqi
 */

#include "stm32f407VG.h"


typedef struct
{
	uint8_t SPI_DeviceMode; 	/*!< possible values from @Device Modes>*/
	uint8_t SPI_BusConfig;   	/*!< possible values from @Bus Configuration >*/
	uint8_t SPI_SclkSpeed; 		/*!< possible values from @Serial Clock Speed >*/
	uint8_t SPI_DFF;			/*!< possible values from @Data Format Configuration>*/
	uint8_t SPI_CPOL;			/*!< possible values from @Clock Pulse Leading or Falling >*/
	uint8_t SPI_CPHA;			/*!< possible values from @Clock Pulse ... >*/
	uint8_t SPI_SSM;			/*!< possible values from @Slave Select Enable/Disable>*/
}SPI_Config_t;


typedef struct
{
	// pointer to hold the base address of the GPIO peripheral

	SPI_RegDef_t 	*pSPIx;
	SPI_Config_t 	SPI_Config;
	uint8_t			*pTxBuffer;		/* !< To store the app. Tx Buffer address > */
	uint8_t			*pRxBuffer;		/* !< To store the app. Rx Buffer address > */
	uint32_t		TxLen;			/* !< To store the Tx Len   >*/
	uint32_t 		RxLen;			/* !< to store the Rx Len   >*/
	uint8_t			TxState;		/* !< to store the Tx State >*/
	uint8_t			RxState; 		/* !< to store the rx State >*/

}SPI_Handle_t;





/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER	1
#define SPI_DEVICE_MODE_SLAVE 	0


/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD				1
#define SPI_BuS_CONFIG_HD 				2
#define SPI_BuS_CONFIG_SIMPLEX_RXONLY	3

/*
 * @SPI_SCLK
 */
#define SPI_SCLK_SPEED_DIV2					0
#define SPI_SCLK_SPEED_DIV4					1
#define SPI_SCLK_SPEED_DIV8					2
#define SPI_SCLK_SPEED_DIV16				3
#define SPI_SCLK_SPEED_DIV32				4
#define SPI_SCLK_SPEED_DIV64				5
#define SPI_SCLK_SPEED_DIV128				6
#define SPI_SCLK_SPEED_DIV256				7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS				0
#define SPI_DFF_16BITS				1

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_HIGH				1
#define SPI_CPOL_LOW				0

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_HIGH				1
#define SPI_CPHA_LOW				0

/*
 * @SPI_SSM
 */
#define SPI_SSM_EN					1
#define SPI_SSM_DI					0


/*
 * SPI Related Status Flag Definition
 */
#define SPI_TXE_FLAG		(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG		(1 << SPI_SR_RXNE)
#define SPI_CHSIDE_FLAG		(1 << SPI_SR_CHSIDE)
#define SPI_UDR_FLAG		(1 << SPI_SR_UDR)
#define SPI_CRCERR_FLAG		(1 << SPI_SR_CRCERR)
#define SPI_MODF_FLAG		(1 << SPI_SR_MODF)
#define SPI_OVR_FLAG		(1 << SPI_SR_OVR)
#define SPI_BSY_FLAG		(1 << SPI_SR_BSY)
#define SPI_FRE_FLAG		(1 << SPI_SR_FRE)


/*
 *  Possible SPI Application States
 */

#define SPI_READY		0
#define SPI_BUSY_IN_RX	1
#define SPI_BUSY_IN_TX	2

#define SPI_EVENT_TX_CMPLT	1
#define SPI_EVENT_RX_CMPLT	2
#define SPI_EVENT_OVR_ERR	3
#define SPI_EVENT_CRC_ERR	4








/********************************************************************************************************
 * 										API's Supported by this driver
 * 						For more information about the APIs check the function definition
 *
 ********************************************************************************************************/

/*
 *  Peripheral clock setup, GPIO_RegDef_t = base address of GPIOA
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
/*
 * SPI init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx); //use Peripheral RST register

/*
 * Data Send and Receive
 */

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len);   // Standard practice to define length a uint32_t
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len);

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len);   // Standard practice to define length a uint32_t
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR Handling
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityHandling(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 *  Other Peripheral Control APIs
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);


uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);


__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv);











































