/*
 * stm32f407VG_gpio_driver.h
 *
 *  Created on: Jun 25, 2022
 *      Author: lamqi
 */

#include "stm32f407VG.h"


typedef struct
{
	uint8_t GPIO_PinNumber; 		/*!< possible values from @GPIO_PIN_NUMBERS >*/
	uint8_t GPIO_PinMode;   		/*!< possible values from @GPIO_PIN_MODES >*/
	uint8_t GPIO_PinSpeed; 			/*!< possible values from @GPIO_OSPEED_TYPE >*/
	uint8_t GPIO_PinPuPdControl;	/*!< possible values from @GPIO_PUPD_TYPE >*/
	uint8_t GPIO_PinOPType;			/*!< possible values from @GPIO_OUTPUT_TYPE >*/
	uint8_t GPIO_PinAltFunMode;		/*!< possible values from @GPIO_PIN_MODES >*/
}GPIO_PinConfig_t;



typedef struct
{
	// pointer to hold the base address of the GPIO peripheral

	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;

}GPIO_Handle_t;



/*
 * @GPIO_PIN_NUMBERS
 * GPIO Pin numbers
 */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15



/*
 * @GPIO_PIN_MODES
 * GPIo Pin Possible Modes
 */

#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6


/*
 * 	@GPIO_OUTPUT_TYPE
 *  GPIO Pin mode Possible Output types
 */

#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

/*
 * @GPIO_OSPEED_TYPE
 * GPIO Pin Mode Output Speed Register Types
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/*
 * @GPIO_PUPD_TYPE
 * GPIO Pin Pull up pull Down configuration macros
 */

#define GPIO_NO_PUPD			0
#define GPIO_PIN_PU				1
#define GPIO_PIN_PD				2









/**************************************************************************
 * 							APIS Supported by this driver
 *
 ***************************************************************************/
/*
 *  Peripheral clock setup, GPIO_RegDef_t = base address of GPIOA
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);
/*
 * GPIO init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx); //use Peripheral RST register
/*
 * Data Read and Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
/*
 * IRQ Configuration and ISR Handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQHandling(uint8_t PinNumber);
void GPIO_IRQPriorityHandling(uint8_t IRQNumber, uint32_t IRQPriority);



























