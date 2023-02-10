/*
 * 011i2c_master_rx_testing.c
 *
 *  Created on: Aug 27, 2022
 *      Author: lamqi
 */


#include "string.h"
#include "stm32f407VG.h"
#include "stm32f407VG_gpio_driver.h"
#include "stm32f407VG_i2c_driver.h"
#include <stdio.h>
extern void initialise_monitor_handles();






void delay(void)
{
	for(uint32_t i=0; i <500000/2; i ++);
}



//Global variables/ declareation

//rcv buffer
uint8_t rcv_buf[60];
uint8_t rxComplt = RESET;

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
	I2C1handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;
	I2C_Init(&I2C1handle);

}

int main(void)
{


	initialise_monitor_handles();

	printf("Application is running\n");

	uint8_t commandcode;
	uint8_t len;

	GPIO_ButtonInit();
	// This function initiates the MOSI, MISO , SCLK, and NSS pins on the GPIO B
	I2C1_GPIOInits();

	// This function initiates the Config Registers for SPI2
	I2C1_Inits();

	//I2C IRQ configurations
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV,ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER,ENABLE);

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
		commandcode=0x51;

		while(I2C_MasterSendDataIT(&I2C1handle, &commandcode, 1, SLAVE_ADDR,I2C_ENABLE_SR) != I2C_READY);

		while(I2C_MasterReceiveDataIT(&I2C1handle, &len, 1, SLAVE_ADDR,I2C_ENABLE_SR)!= I2C_READY);


		commandcode=0x52;

		while(I2C_MasterSendDataIT(&I2C1handle, &commandcode, 1, SLAVE_ADDR,I2C_ENABLE_SR)!= I2C_READY);
		while(I2C_MasterReceiveDataIT(&I2C1handle, rcv_buf, len, SLAVE_ADDR,I2C_NO_SR)!= I2C_READY);
		rxComplt = RESET;
		//wait till Rx complete
		while(rxComplt != SET);
		rcv_buf[len+1]='\0';
		printf("Done read.\n");
		printf("Data: %s \n", rcv_buf);
		rxComplt = RESET;
	}

}

void I2C1_EV_IRQHandler(void)
{
	I2C_EV_IRQHandling(&I2C1handle);
}

void I2C1_ER_IRQHandler(void)
{
	I2C_ER_IRQHandling(&I2C1handle);
}

void I2C_ApplicationEventCallback(I2CHandle_t *pI2CHandle,uint8_t AppEv)
{
	if (AppEv == I2C_EV_TX_CMPLT)
	{
		printf("Tx is completed\n");
	}else if (AppEv == I2C_EV_RX_CMPLT)
	{
		printf("Rx is completed\n");
		rxComplt = SET;
	}else if (AppEv == I2C_ERROR_AF)
	{
		printf("Error: Ack Failure\n");

		I2C_CloseSendData(pI2CHandle);

		I2C_GenerateStopCondition(I2C1);

		while(1);
	}

}







/*
#define I2C_EV_TX_NOT_CMPLT			0
#define I2C_EV_TX_CMPLT				1
#define I2C_EV_RX_NOT_CMPLT			0
#define I2C_EV_RX_CMPLT				1
#define	I2C_EV_STOP					0
#define I2C_ERROR_BERR  			3
#define I2C_ERROR_ARLO  			4
#define I2C_ERROR_AF    			5
#define I2C_ERROR_OVR   			6
#define I2C_ERROR_TIMEOUT			 7

*/












