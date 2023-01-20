/*
 * timertest.c
 *
 *  Created on: 20/01/2023
 *      Author: tomed
 */
#include "stm32f407xx.h"

int main(void)
{
	TIM_Handle_t timer;
	timer.pTIMx = TIM6;
	//-1 nesscary as extra clock cycle is spent at Auto reload value
	timer.TIM_Config_t.TIM_AutoReload = 64000-1;
	timer.TIM_Config_t.TIM_Prescalar = 24;
	TIM_Init(&timer);
}
