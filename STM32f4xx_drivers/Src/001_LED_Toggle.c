/*
 * 001_LED_Toggle.c
 *
 *  Created on: May 16, 2021
 *      Author: lamqi
 */


#include "STM32f407xx.h"


void delay(void)
{
	for (uint32_t i=0; i <500000/2 ; i++)
	{

	}
}
int main(void)
{

	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_OP_MED_SPEED;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PU_PD;


	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_init(&GpioLed);


	GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_12, GPIO_PIN_SET);
	while (1)
	{
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
	delay();
	}

	return 0;
}
