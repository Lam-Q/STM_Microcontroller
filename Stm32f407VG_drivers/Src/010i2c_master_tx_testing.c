/*
 * 010i2c_master_tx_testing.c
 *
 *  Created on: Aug 7, 2022
 *      Author: lamqi
 */


#include "string.h"
#include "stm32f407VG.h"
#include "stm32f407VG_gpio_driver.h"
#include "stm32f407VG_i2c_driver.h"


void delay(void)
{
	for(uint32_t i=0; i <500000/2; i ++);
}



//Global variables/ declareation

uint8_t some_data[] = "We are testing i2C number!";
I2CHandle_t I2C1handle;
#define MY_ADDR 0x61
#define SLAVE_ADDR 0x68





/*
 * PB6 -> SCL
 * PB7 -> SDA
 *
 */
void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;    //assign the base address of pin
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	//SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);
}


void GPIO_ButtonInit(void)
{
	// BUTTON
	GPIO_Handle_t GpioBtn;

	GpioBtn.pGPIOx = GPIOA;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GpioBtn);
}

void I2C1_Inits(void)
{

	I2C1handle.pI2Cx = I2C1;
	I2C1handle.I2C_Config.I2C_AckControl = I2C_ACK_DISABLE;
	I2C1handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;
	I2C_Init(&I2C1handle);

}

int main(void)
{



	GPIO_ButtonInit();
	// This function initiates the MOSI, MISO , SCLK, and NSS pins on the GPIO B
	I2C1_GPIOInits();

	// This function initiates the Config Registers for SPI2
	I2C1_Inits();

	//enable the I2C1 Peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	//enable ACk after I2C Peripheral Enable
	if(I2C1handle.I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(I2C1handle.pI2Cx, I2C_ACK_ENABLE);
	}




	while(1)
	{
		while(!GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		delay();
		I2C_MasterSendData(&I2C1handle, some_data, strlen((char*)some_data), SLAVE_ADDR);

	}

}
