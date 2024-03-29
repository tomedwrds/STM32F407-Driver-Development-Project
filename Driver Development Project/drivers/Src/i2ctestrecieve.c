/*
 * i2ctest.c
 *
 *  Created on: 16/01/2023
 *      Author: tomed
 */


#include "stm32f407xx.h"
#include <string.h>

void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}
void I2C1_GPIO_Inits(void)
{
	//GPIO pins must be configured for AF mode
	GPIO_Handle_t I2CPins;
	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	//MOSI
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);

}

I2C_Handle_t I2C1Handle;
uint8_t rcv_buff[32];


#define MY_ADDR 0x61
#define SLAVE_ADDR 0x68

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
}

void Btn_GPIO_Inits(void)
{
	GPIO_Handle_t GPIOBtn;

	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_Init(&GPIOBtn);
}


int main(void)
{

	uint8_t commandcode;
	uint8_t len;

	//Intalize peripheals
	Btn_GPIO_Inits();
	I2C1_GPIO_Inits();
	I2C1_Inits();

	//I2c irq configurations
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

	I2C_PeripheralControl(I2C1, ENABLE);

	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);
	while(1)
	{
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		delay();

		//Send the command code that will cause slave to send length of data
		commandcode = 0x51;
		//Application hangs until i2c is ready
		while( I2C_MasterSendDataIT(&I2C1Handle, &commandcode, 1, SLAVE_ADDR,I2C_ENABLE_SR) != I2C_READY);

		//Read the length of the data from the slave
		while( I2C_MasterReceiveDataIT(&I2C1Handle, &len, 1, SLAVE_ADDR,I2C_ENABLE_SR) != I2C_READY);

		//Send the command to slave to send data
		commandcode = 0x52;
		while(I2C_MasterSendDataIT(&I2C1Handle, &commandcode, 1, SLAVE_ADDR,I2C_ENABLE_SR)!=I2C_READY);

		//Read the  data from the slave
		while(I2C_MasterReceiveDataIT(&I2C1Handle, rcv_buff, len, SLAVE_ADDR,I2C_DISABLE_SR) !=I2C_READY);

	}

}

void I2C1_EV_IRQHandler (void)
{
	I2C_EV_IRQHandling(&I2C1Handle);
}

void I2C1_ER_IRQHandler (void)
{
	I2C_ER_IRQHandling(&I2C1Handle);
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv)
{
	if(AppEv == I2C_ERROR_AF)
	{
		//if master ack error occurs when slave fails to send ack for the byte
		I2C_CloseSendData(pI2CHandle);

		//generate stop condition to relase the bus
		I2C_GenerateStopCondition(I2C1);

		//Hang in infinite loop
		 while(1);
	}
}
