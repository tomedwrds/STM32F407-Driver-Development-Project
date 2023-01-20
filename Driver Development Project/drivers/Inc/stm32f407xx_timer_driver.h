/*
 * stm32f407x_timer_driver.h
 *
 *  Created on: 20/01/2023
 *      Author: tomed
 */

#ifndef INC_STM32F407XX_TIMER_DRIVER_H_
#define INC_STM32F407XX_TIMER_DRIVER_H_

#include "stm32f407xx.h"

/*
 * This is a config structure for a Timer
 * */

typedef struct
{
	uint16_t TIM_Prescalar;
	uint16_t TIM_AutoReload;

}TIM_Config_t;

/*
 * This is a handle structure for a GPIO pins
 * */

typedef struct
{
	TIM_RegDef_t *pTIMx;
	TIM_Config_t TIM_Config_t;

}TIM_Handle_t;

//Peripheal clock setup
void TIM_PeriClockControl(TIM_RegDef_t *pTIMx, uint8_t EnorDi);

//Init/Deinit
void TIM_Init(TIM_Handle_t *TIM_Handle_t);
void TIM_DeInit(TIM_RegDef_t *pTIMx);

//IRQ APIs
void TIM_IRQConfig(uint8_t IRQ_Number, uint8_t EnorDi);
void TIM_IRQPriorityConfig(uint8_t IRQ_Number, uint8_t IRQ_Priority);

#endif /* INC_STM32F407XX_TIMER_DRIVER_H_ */
