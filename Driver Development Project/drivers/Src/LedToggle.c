/*
 * LedToggle.c
 *
 *  Created on: Jan 10, 2023
 *      Author: tomed
 */


#include "stm32f407xx.h"
#include <string.h>


void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

int main(void)
{
	GPIO_Handle_t GpioLed;
	//Memset is used to prevent unset fields being set to junk values
	memset(&GpioLed,0,sizeof(GpioLed));
//
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
//
	GPIO_PeriClockControl(GPIOA,ENABLE);
	GPIO_Init(&GpioLed);

	GPIO_Handle_t GpioBtn;
	memset(&GpioBtn,0,sizeof(GpioBtn));

	GpioBtn.pGPIOx = GPIOB;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOB,ENABLE);
	GPIO_Init(&GpioBtn);


	//IRQ config
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, 15);
	GPIO_IRQConfig(IRQ_NO_EXTI15_10, ENABLE);




//	while(1)
//	{
//		if(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_12) == 0)
//		{
//			delay();
//			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
//		}
//
//
//	}
	while(1);


}



void EXTI15_10_IRQHandler(void)
{
	delay();
	GPIO_IRQHandling(GPIO_PIN_NO_12);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
}
