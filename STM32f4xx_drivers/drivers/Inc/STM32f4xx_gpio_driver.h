/*
 * STM32f4xx_gpio_driver.h
 *
 *  Created on: May 13, 2021
 *      Author: lamqi
 */

#include "STM32f407xx.h"
#include <stdint.h>

#ifndef INC_STM32F4XX_GPIO_DRIVER_H_
#define INC_STM32F4XX_GPIO_DRIVER_H_


/*
 * This is a Handle structure for a GPIO Pin
 */

typedef struct
{
	uint8_t GPIO_PinNumber;      /* < Possible values from @GPIO_PIN_NUMBER  >*/
	uint8_t GPIO_PinMode;        /* < Possible values from @GPIO_PIN_MODES   >*/
	uint8_t GPIO_PinSpeed;       /* < Possible values from @GPIO_PIN_SPEED   >*/
	uint8_t GPIO_PinPuPdControl; /* < Possible values from @GPIO_PIN_PUPD    >*/
	uint8_t GPIO_PinOPType;      /* < Possible values from @GPIO_PIN_OP_TYPE >*/
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;


typedef struct
{
	// pointer to hold the base address of the GPIO Peripheral

	GPIO_RegDef_t *pGPIOx; /* <This holds the base address of the GPIO port to which the pin belongs>*/
	GPIO_PinConfig_t GPIO_PinConfig;  /* < This holds GPIO pin configuration setttings >*/

}GPIO_Handle_t;


/*
 * @GPIO_PIN_NUMBER
 * GPIO Pin number
 */

#define GPIO_PIN_NO_0        0
#define GPIO_PIN_NO_1        1
#define GPIO_PIN_NO_2        2
#define GPIO_PIN_NO_3        3
#define GPIO_PIN_NO_4        4
#define GPIO_PIN_NO_5        5
#define GPIO_PIN_NO_6        6
#define GPIO_PIN_NO_7        7
#define GPIO_PIN_NO_8        8
#define GPIO_PIN_NO_9        9
#define GPIO_PIN_NO_10       10
#define GPIO_PIN_NO_11       11
#define GPIO_PIN_NO_12       12
#define GPIO_PIN_NO_13       13
#define GPIO_PIN_NO_14       14
#define GPIO_PIN_NO_15       15


/*
 * @GPIO_PIN_MODES
 * GPIO Pin possible modes
 */

#define GPIO_MODE_IN     0
#define GPIO_MODE_OUT    1
#define GPIO_MODE_ALTFN  2
#define GPIO_MODE_ANALOG 3
#define GPIO_MODE_IT_FT  4    // GPIO Input Falling edge
#define GPIO_MODE_IT_RT  5    // GPIO Input Rising edge
#define GPIO_MODE_IT_RFT 6   // GPIO Input Rising and Falling edge triggerd

/*
 * @GPIO_PIN_OP_TYPE
 * GPIO Pin possible output types
 */
#define GPIO_OP_TYPE_PP  0   // GPIO Push Pull
#define GPIO_OP_TYPE_OD  1   // GPIO Open Drain

/*
 * @GPIO_PIN_SPEED
 * GPIO Pin possible output speed
 */

#define GPIO_OP_LOW_SPEED     0
#define GPIO_OP_MED_SPEED     1
#define GPIO_OP_HIGH_SPEED    2
#define GPIO_OP_Very_HI_SPEED 3


/*
 * @GPIO_PIN_PUPD
 * GPIO Pin possible pull up and pull down configuration macros
 */

#define GPIO_NO_PU_PD  0  // No Pull up, Only Pull Down
#define GPIO_PU		   1  // Pull Up
#define GPIO_PD        2  // Pull Down


/*****************************************************************************************
 *							API's Supported by this driver
 * For more information aboutr the API's check the function definition
 *****************************************************************************************/
/*
 * Peripheral Clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t ENorDI);

/*
 * GPIO Initialization and De-initialization
 */
void GPIO_init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Data Read and Data write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx,uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR Handling
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t ENorDI);     // Interrupt configuration
void GPIO_IRQHandling(uint8_t PinNumber);   // Interrupt handling function



#endif /* INC_STM32F4XX_GPIO_DRIVER_H_ */
