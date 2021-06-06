/*
 * 002_led_button.c
 *
 *  Created on: May 16, 2021
 *      Author: lamqi
 */

#include "STM32f407xx.h"
#define HIGH 1
#define BTN_PRESSED HIGH

void delay(void)
{
	for (uint32_t i=0; i <500000 ; i++)
	{

	}
}
int main(void)
{

	GPIO_Handle_t GpioLed,GpioBtn;

	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_MED_SPEED;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PU_PD;


	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_init(&GpioLed);

// Configuration for button
	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_MED_SPEED;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PU_PD;


	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_init(&GpioBtn);


	while (1)
	{
		if (GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == BTN_PRESSED){
			delay();
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		}


	}

	return 0;
}
