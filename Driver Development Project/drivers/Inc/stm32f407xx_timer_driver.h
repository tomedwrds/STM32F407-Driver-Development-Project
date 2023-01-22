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
	uint32_t TIM_Prescalar; // This size of this value depedends on size of this register for each tiemr
	uint16_t TIM_AutoReload;
	uint8_t TIM_CounterMode;

}TIM_Config_t;

typedef struct
{
	uint8_t IC_Filter;
	uint8_t IC_Polarity;
	uint8_t IC_Prescaler;
	uint8_t IC_Channel;
}TIM_IC_Config_t;

typedef struct
{
	TIM_RegDef_t *pTIMx;
	TIM_Config_t TIMConfig;
	TIM_IC_Config_t TIMICConfig;

}TIM_Handle_t;




/*
 * 	Counter mode options
 */

#define TIM_COUNTER_UP 				0
#define TIM_COUNTER_DOWN 			1
#define TIM_COUNTER_CMS_IT_DOWN 	2
#define TIM_COUNTER_CMS_IT_UP 		3
#define TIM_COUNTER_CMS_IT_UPDOWN 	4



/*
 * Input capture options
 */

#define TIM_IC_Channel_OP				0
#define TIM_IC_Channel_IP1				1
#define TIM_IC_Channel_IP2				2
#define TIM_IC_Channel_IP3				3
#define TIM_IC_Channel_IP4				4

#define TIM_IC_PSC_1				0
#define TIM_IC_PSC_2				1
#define TIM_IC_PSC_4				2
#define TIM_IC_PSC_8				3

#define TIM_IC_IF_NONE				0
#define TIM_IC_IF_2					1
#define TIM_IC_IF_4					2
#define TIM_IC_IF_8					3

#define TIM_IC_POL_RISING				0
#define TIM_IC_POL_FALLING				1
#define TIM_IC_POL_BOTH				3
/*
 * TIM related status flags definitions
 */
#define TIM_FLAG_UIF   		( 1 << TIM_SR_UIF)


//Peripheal clock setup
void TIM_PeriClockControl(TIM_RegDef_t *pTIMx, uint8_t EnorDi);

//Init/Deinit
void TIM_Init(TIM_Handle_t *TIM_Handle_t);
void TIM_DeInit(TIM_RegDef_t *pTIMx);
void TIM_Toggle(TIM_RegDef_t *pTIMx,uint8_t EnorDi);
void TIM_CC_Toggle(TIM_Handle_t *TIM_Handle_t,uint8_t EnorDi);

//IRQ APIs
void TIM_IRQConfig(uint8_t IRQ_Number, uint8_t EnorDi);
void TIM_IRQPriorityConfig(uint8_t IRQ_Number, uint8_t IRQ_Priority);

uint8_t TIM_Get_FlagStatus(TIM_RegDef_t *pTIMx , uint32_t FlagName);
void TIM_Clear_Update(TIM_RegDef_t *pTIMx);
#endif /* INC_STM32F407XX_TIMER_DRIVER_H_ */
