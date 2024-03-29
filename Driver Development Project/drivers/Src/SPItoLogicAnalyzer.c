/*
 * main.c
 *
 *  Created on: 12/01/2023
 *      Author: tomed
 */

#include "stm32f407xx.h"
#include <string.h>


//command codes
#define COMMAND_LED_CTRL      		0x50
#define COMMAND_SENSOR_READ      	0x51
#define COMMAND_LED_READ      		0x52
#define COMMAND_PRINT      			0x53
#define COMMAND_ID_READ      		0x54

#define LED_ON     1
#define LED_OFF    0

//arduino analog pins
#define ANALOG_PIN0 	0
#define ANALOG_PIN1 	1
#define ANALOG_PIN2 	2
#define ANALOG_PIN3 	3
#define ANALOG_PIN4 	4

//arduino led

#define LED_PIN  9

//PB14 MISO
//PB15 MOSI
//PB13 SCLK
//PB12 NSS
//Alt function mode 5
void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}
void SPI_GPIO_Inits(void)
{
	//GPIO pins must be configured for AF mode
	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);
}




void SPI2_Inits(void)
{
	SPI_Handle_t SPI2handle;
	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI;

	SPI_Init(&SPI2handle);
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

uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{
	if(ackbyte == 0xF5)
	{
		//ack
		return 1;
	}
	else
	{
		//nack
		return 0;
	}
}

int main(void)
{
	uint8_t dummyByte = 0xff;
	uint8_t dummyRead;

	//Enable clock for GPIOB and SPI2
	SPI_GPIO_Inits();
	SPI2_Inits();
	Btn_GPIO_Inits();


	SPI_SSOEConfig(SPI2, ENABLE);
	while(1)
	{

		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		delay();

		//Enable the SPE bit. It is recommended to only enable this bit after the settings have been made on the peripheal
		SPI_PeripheralControl(SPI2, ENABLE);

		//Send the first command
		uint8_t commndcode = COMMAND_LED_CTRL;
		uint8_t ackbyte;
		uint8_t args[2];
		//send command
		SPI_SendData(SPI2, &commndcode, 1);

		//dummy read is required to clear the RXNE of junk data that is received
		//by reading the value the RX buffer is cleared
		SPI_RecieveData(SPI2, &dummyRead, 1);

		//send dummy bits to fetch the respone from slave. This allows for the Tx buffer of slave to enter Rx of Master
		SPI_SendData(SPI2, &dummyByte, 1);
		SPI_RecieveData(SPI2, &ackbyte, 1);

		//Determine wetehr receive ack or nack
		if(SPI_VerifyResponse(ackbyte))
		{
			args[0] = LED_PIN;
			args[1] = LED_ON;
			//send data
			SPI_SendData(SPI2, args, 2);
			//dummy read
			SPI_RecieveData(SPI2,args,2);
		}

		//CMD sensor read wait for button press again
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));
		delay();

		commndcode = COMMAND_SENSOR_READ;

		//send command
		SPI_SendData(SPI2, &commndcode, 1);

		//dummy read is required to clear the RXNE of junk data that is received
		//by reading the value the RX buffer is cleared
		SPI_RecieveData(SPI2, &dummyRead, 1);

		//send dummy bits to fetch the respone from slave. This allows for the Tx buffer of slave to enter Rx of Master
		SPI_SendData(SPI2, &dummyByte, 1);
		SPI_RecieveData(SPI2, &ackbyte, 1);

		//Determine wetehr receive ack or nack
		if(SPI_VerifyResponse(ackbyte))
		{
			args[0] = ANALOG_PIN0;
			SPI_SendData(SPI2, args, 1);

			//Read the response of the sensor

			//dummy read is required to clear the RXNE of junk data that is received
			//by reading the value the RX buffer is cleared
			SPI_RecieveData(SPI2, &dummyRead, 1);

			//Inset some delay so slave can be ready with the data from adc conversion
			delay();


			//send dummy bits to fetch the respone from slave. This allows for the Tx buffer of slave to enter Rx of Master
			SPI_SendData(SPI2, &dummyByte, 1);

			uint8_t anlg_read;

			SPI_RecieveData(SPI2, &anlg_read, 1);
		}



		//Waits until spi is no longer busy and commuication has finished
		while(SPI_Get_FlagStatus(SPI2, SPI_BUSY_FLAG));

		SPI_SSIConfig(SPI2,DISABLE);
	}

	return 0;
}
