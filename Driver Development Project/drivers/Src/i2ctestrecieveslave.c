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


uint8_t Tx_buf[32] = "STM32 Slave mode testing.";

#define MY_ADDR 0x68
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

	//Irq configs
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

	I2C_SlaveEnableDisableCallbackEvents(I2C1,ENABLE);

	I2C_PeripheralControl(I2C1, ENABLE);

	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

	//The slave rx and tx is handeled through interupts so program can hang
	while(1);
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
	//Static keywords makes the variables appear in global memory
	static uint8_t commandCode = 0;
	static uint8_t cnt = 0;


	if(AppEv == I2C_EV_DATA_REQ)
	{
		//master wants some data. slave has to send it
		if(commandCode ==  0x51)
		{
			// send length
			I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen((char*)Tx_buf));
		}
		else if(commandCode ==  0x52)
		{
			//send data or contets of tx buff
			I2C_SlaveSendData(pI2CHandle->pI2Cx, Tx_buf[cnt++]);
		}
	}
	else if(AppEv == I2C_EV_DATA_RCV)
	{
		//Data is wating for the slave to read
		commandCode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);
	}
	else if(AppEv == I2C_ERROR_AF)
	{
		//NACK generated at end of slave tx
		//Command code is invalidated and cnt is reset as no mrore data needs to be sent
		commandCode = 0xff;
		cnt = 0;
	}
	else if(AppEv == I2C_EV_STOP)
	{
		//hapenns during slave reception
		//master has ended the i2c communciation with the slave
	}
}
