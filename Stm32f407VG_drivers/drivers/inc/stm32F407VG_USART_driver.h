/*
 * stm32F407VG_USART_driver.h
 *
 *  Created on: Jul 16, 2022
 *      Author: lamqi
 */

#ifndef INC_STM32F407VG_USART_DRIVER_H_
#define INC_STM32F407VG_USART_DRIVER_H_

#include "stm32f407VG.h"

/*
 * Configuration Structure for USARTx peripheral
 */
typedef struct
{
	uint8_t USART_Mode;
	uint32_t USART_Baud;
	uint8_t USART_NoOfStopBits;
	uint8_t USART_WordLength;
	uint8_t USART_ParityControl;
	uint8_t USART_HWFlowControl;
}USART_Config_t;

/*
 * Handle structure for USARTx peripheral
 */

typedef struct
{
	// pointer to hold the base address of the GPIO peripheral

	USART_RegDef_t 	*pUSARTx;
	USART_Config_t 	USART_Config;
	uint8_t			*pTxBuffer;     /* !< To Store the app. Tx buffer address>*/
	uint8_t			*pRxBuffer;		/* !< To store the app. Rx buffer address>*/
	uint32_t		TxLen;  		/* !< To store Tx Len > */
	uint32_t		RxLen; 			/* !< To store Rx Len > */
	uint8_t			TxBusyState;      /* !< To store Communication State > */
	uint8_t			RxBusyState;		/* !< To store slave/device address > */
}USARTHandle_t;

/*
 * @USART_Mode
 * Posible options for USART_Mode
 */
#define USART_MODE_ONLY_TX		0
#define USART_MODE_ONLY_RX		1
#define USART_MODE_TXRX			2

/*
 * @USART_Baud
 * Possible options for USART_Baud
 */

#define USART_STD_BAUD_1200			1200
#define USART_STD_BAUD_2400			2400
#define USART_STD_BAUD_9600			9600
#define USART_STD_BAUD_19200		19200
#define USART_STD_BAUD_38400		38400
#define USART_STD_BAUD_57600		57600
#define USART_STD_BAUD_115200		115200
#define USART_STD_BAUD_230400		230400
#define USART_STD_BAUD_460800		460800
#define USART_STD_BAUD_921600		921600
#define USART_STD_BAUD_2M			2000000
#define USART_STD_BAUD_3M			3000000


/*
 * @USART_ParityControl
 * Possible options for USART_ParityControl
 */
#define USART_PARITY_EN_ODD			2
#define USART_PARITY_EN_EVEN		1
#define USART_PARITY_EN_DISABLE		0


/*
 * @USART_WordLength
 * Possible options for USART_WordLength
 */

#define USART_WORDLEN_8BITS		0
#define USART_WORDLEN_9BITS		1


/*
 * @USART_NoOfSTopBits
 * Possible options for USART_NoOfStopBits
 */

#define USART_STOPBITS_1		0
#define USART_STOPBITS_0_5		1
#define USART_STOPBITS_2		2
#define USART_STOPBITS_1_5		3



/*
 * @USART_HWFlowControl
 * Possible options for USART_HWFlowControl
 */

#define USART_HW_FLOW_CTRL_NONE			0
#define USART_HW_FLOW_CTRL_CTS			1
#define USART_HW_FLOW_CTRL_RTS			2
#define	USART_HW_FLOW_CTRL_CTS_RTS		3

/*
 * USART Flags
 */

#define USART_FLAG_TXE		(1 << USART_SR_TXE)
#define USART_FLAG_RXNE		(1 << USART_SR_RXNE)
#define USART_FLAG_TC		(1 << USART_SR_TC)


/*
 * Application states
 */

#define USART_BUSY_IN_RX 1
#define USART_BUSY_IN_TX 2
#define USART_READY		 0

#define USART_EVENT_TX_CMPLT		0
#define USART_EVENT_RX_CMPLT		1
#define USART_EVENT_IDLE			2
#define USART_EVENT_CTS				3
#define USART_EVENT_PE				4
#define USART_EVENT_FE				5
#define USART_EVENT_NE				6
#define USART_EVENT_ORE				7
#define USART_ERREVENT_FE			8
#define USART_ERREVENT_NF			9
#define USART_ERREVENT_ORE			10

/*
 * USART FLAG Definition
 */

#define USART_FLAG_PARITY_ERROR		(1 << USART_SR_PE)
#define USART_FLAG_FRAMING_ERROR	(1 << USART_SR_FE)
#define USART_FLAG_NOISE_DECT		(1 << USART_SR_NF)
#define USART_FLAG_OVERRUN_ERR		(1 << USART_SR_ORE)
#define USART_FLAG_IDLE				(1 << USART_SR_IDLE)
#define USART_FLAG_RXNE				(1 << USART_SR_RXNE)
#define USART_FLAG_TRANS_SCMPLT		(1 << USART_SR_TC)
#define USART_FLAG_TXE				(1 << USART_SR_TXE)
#define USART_FLAG_LIN_BRK			(1 << USART_SR_LBD)
#define USART_FLAG_CTS				(1 << USART_SR_CTS)

/*
 * Other Misc Definitions
 */

#define USART_PARITY_DISABLE 	0
#define USART_PARITY_ENABLE		1



// Options for Configuration I/O
/*
 * Peripheral Clock Setup
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);

/*
 * Init and De-Init
 */
void UART_Init(USARTHandle_t *pUSARTHandle);
void USART_Deinit(USART_RegDef_t *pUSARTx);


/*
 * Data Send and Receive
 */
void USART_SendData(USARTHandle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void USART_ReceiveData(USARTHandle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

uint8_t USART_SendDataIT(USARTHandle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USARTHandle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t  IRQPriority);
void USART_IRQHandling(USARTHandle_t *pUSARTHandle);


/*
 * Other Peripheral Control APIs
 */

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi);
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);


/*
 * Application Callbacks
 */

void USART_ApplicationEventCallback(USARTHandle_t *pUSARTHandle, uint8_t ApEv);






#endif /* INC_STM32F407VG_USART_DRIVER_H_ */
