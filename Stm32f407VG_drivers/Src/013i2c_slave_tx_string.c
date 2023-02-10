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
uint8_t Tx_buf[32] = "STM32 Slave mode testing.."; //arduino wire library allows 32 per xmission
uint8_t rxComplt = RESET;
I2CHandle_t I2C1handle;

#define SLAVE_ADDR 0x68
#define MY_ADDR SLAVE_ADDR




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
	GPIO_ButtonInit();
	// This function initiates the MOSI, MISO , SCLK, and NSS pins on the GPIO B
	I2C1_GPIOInits();

	// This function initiates the Config Registers for SPI2
	I2C1_Inits();

	//I2C IRQ configurations
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV,ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER,ENABLE);

	I2C_SlaveEnableDisableCallbackEvents(I2C1,ENABLE);

	//enable the I2C1 Peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	//enable ACk after I2C Peripheral Enable
	if(I2C1handle.I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(I2C1handle.pI2Cx, I2C_ACK_ENABLE);
	}




	while(1);

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

	static uint8_t commandCode =0;
	static uint8_t Cnt =0;
	if (AppEv == I2C_EV_DATA_REQ)
	{
		if (commandCode == 0x51)
		{
			//send the length information the master
			I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen((char*)Tx_buf));
		}else if (commandCode == 0x52)
		{

			I2C_SlaveSendData(pI2CHandle->pI2Cx, Tx_buf[Cnt++]);
		}
	}else if (AppEv == I2C_EV_DATA_RCV)
	{
		// Data is waiting for the slave to read. Slave has to read it.
		commandCode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);

	}else if (AppEv == I2C_ERROR_AF)
	{
		// This happens only during slave txing
		//Master has sent the NACK. So slave should understand that master doesnt need
		//more data.
		commandCode == 0xFF;
		Cnt = 0;
	}
	else if (AppEv == I2C_EV_STOP)
	{
		// slave should conclude master ended I2C Communication
	}
}












