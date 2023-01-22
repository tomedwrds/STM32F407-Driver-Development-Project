/*
 * timertest.c
 *
 *  Created on: 20/01/2023
 *      Author: tomed
 */
#include "stm32f407xx.h"
#include <string.h>

void HSI_Config(void);

void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}
int main(void)
{
	//Enable the irq config
	TIM_IRQConfig(IRQ_NO_TIM3, ENABLE);
	TIM_Handle_t timer;
	timer.pTIMx = TIM3;

	//-1 nesscary as extra clock cycle is spent at Auto reload value
	timer.TIMConfig.TIM_AutoReload = 64000-1;
	timer.TIMConfig.TIM_Prescalar = 24;
	timer.TIMConfig.TIM_CounterMode = TIM_COUNTER_UP;
	timer.TIMICConfig.IC_Channel = TIM_IC_Channel_IP1;
	timer.TIMICConfig.IC_Filter = TIM_IC_IF_NONE;
	timer.TIMICConfig.IC_Polarity = TIM_IC_POL_RISING;
	timer.TIMICConfig.IC_Prescaler = TIM_IC_PSC_1;

	TIM_Init(&timer);




	//Iniatlize the LED
	GPIO_Handle_t GpioLed, GPIOClk;
	//Memset is used to prevent unset fields being set to junk values
	memset(&GpioLed,0,sizeof(GpioLed));

	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GpioLed.GPIO_PinConfig.GPIO_PinAltFunMode = 2;

	GPIO_Init(&GpioLed);


	memset(&GpioLed,0,sizeof(GpioLed));

	GPIOClk.pGPIOx = GPIOA;
	GPIOClk.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GPIOClk.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GPIOClk.GPIO_PinConfig.GPIO_PinAltFunMode = 0;
	GPIOClk.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOClk.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOClk.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

		GPIO_Init(&GPIOClk);

	HSI_Config();

	//Enable the CC on the time
	TIM_CC_Toggle(&timer, ENABLE);
	//Enable the timer
	TIM_Toggle(TIM3, ENABLE);

	//Enable

	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_0);
	delay();
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_0);
		delay();
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_0);
			delay();
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_0);
				delay();



	while(1);

	return 0;


}

void HSI_Config(void)
{
	RCC->CFGR |= (1<<21);
	//Enable PWR clock
	PWR_PCLK_EN();
	PWR->CR |= (1<< 8);
	RCC->BDCR |= 1;

}

void TIM3_IRQHandler(void)
{
	TIM_Clear_Update(TIM2);
}
