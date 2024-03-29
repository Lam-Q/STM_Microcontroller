/*
 * 0022Led_Button.c
 *
 *  Created on: Jun 26, 2022
 *      Author: lamqi
 */

#include <string.h>
#include "stm32F407VG.h"
#include "stm32f407VG_gpio_driver.h"

void delay(void)
{
	for(uint32_t i=0; i <500000/2; i ++);
}
int main(void)
{

	// LED  Port D Input 12

	GPIO_Handle_t GpioLed;
	memset(&GpioLed,0,sizeof(GpioLed));
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;





	// BUTTON
	GPIO_Handle_t GpioBtn;

	// BTn does not use Output type, therefore If we dont reset it, random OP mode data will be written in.
	memset(&GpioBtn,0,sizeof(GpioBtn));

	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioBtn);
	//IRQ Configuration


	GPIO_IRQPriorityHandling(IRQ_NO_EXTI0, NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0, ENABLE);




	while(1)

	return 0;
}


void EXTI0_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_0);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
}
