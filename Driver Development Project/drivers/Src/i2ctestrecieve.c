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

	I2C_PeripheralControl(I2C1, ENABLE);

	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);
	while(1)
	{
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		delay();

		//Send the command code that will cause slave to send length of data
		commandcode = 0x51;
		I2C_MasterSendData(&I2C1Handle, &commandcode, 1, SLAVE_ADDR,I2C_ENABLE_SR);

		//Read the length of the data from the slave
		I2C_MasterReceiveData(&I2C1Handle, &len, 1, SLAVE_ADDR,I2C_ENABLE_SR);

		//Send the command to slave to send data
		commandcode = 0x52;
		I2C_MasterSendData(&I2C1Handle, &commandcode, 1, SLAVE_ADDR,I2C_ENABLE_SR);

		//Read the  data from the slave
		I2C_MasterReceiveData(&I2C1Handle, rcv_buff, len, SLAVE_ADDR,I2C_DISABLE_SR);

	}



}
