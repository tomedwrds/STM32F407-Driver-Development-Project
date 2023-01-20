/*
 * stm32f407xx_timer_driver.c
 *
 *  Created on: 20/01/2023
 *      Author: tomed
 */

#include "stm32f407xx_timer_driver.h"

/*********************************************************************
 * @fn      		  - TIM_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for a TIMER
 *
 * @param[in]         - base address of the timer peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              - none
 *********************************************************************/

void TIM_PeriClockControl(TIM_RegDef_t *pTIMx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pTIMx == TIM1)
		{
			TIM1_PCLK_EN();
		}else if (pTIMx == TIM2)
		{
			TIM2_PCLK_EN();
		}else if (pTIMx == TIM3)
		{
			TIM3_PCLK_EN();
		}else if (pTIMx == TIM4)
		{
			TIM4_PCLK_EN();
		}else if (pTIMx == TIM5)
		{
			TIM5_PCLK_EN();
		}else if (pTIMx == TIM6)
		{
			TIM6_PCLK_EN();
		}else if (pTIMx == TIM7)
		{
			TIM7_PCLK_EN();
		}else if (pTIMx == TIM8)
		{
			TIM8_PCLK_EN();
		}else if (pTIMx == TIM9)
		{
			TIM9_PCLK_EN();
		}else if (pTIMx == TIM10)
		{
			TIM10_PCLK_EN();
		}else if (pTIMx == TIM11)
		{
			TIM11_PCLK_EN();
		}else if (pTIMx == TIM12)
		{
			TIM12_PCLK_EN();
		}else if (pTIMx == TIM13)
		{
			TIM13_PCLK_EN();
		}else if (pTIMx == TIM14)
		{
			TIM14_PCLK_EN();
		}
	}
}

/*********************************************************************
 * @fn      		  - TIM_Init
 *
 * @brief             - Initalizes the timer peripheal
 *
 * @param[in]         - handle strucutre of the timer
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              - none
 *********************************************************************/
void TIM_Init(TIM_Handle_t *TIM_Handle_t)
{
	//Enable the clock
	TIM_PeriClockControl(TIM_Handle_t->pTIMx, ENABLE);

	//Set the prescalar value
	TIM_Handle_t->pTIMx->PSC = TIM_Handle_t->TIM_Config_t.TIM_Prescalar;

	//Set the autoreload value
	TIM_Handle_t->pTIMx->ARR = TIM_Handle_t->TIM_Config_t.TIM_AutoReload;

}
/*********************************************************************
 * @fn      		  - TIM_DeInit
 *
 * @brief             - Resets and deinizliaes the timer peripheal
 *
 * @param[in]         - timer base address
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              - none
 *********************************************************************/
void TIM_DeInit(TIM_RegDef_t *pTIMx)
{

	if(pTIMx == TIM1)
	{
		TIM1_REG_RESET();
	}else if (pTIMx == TIM2)
	{
		TIM2_REG_RESET();
	}else if (pTIMx == TIM3)
	{
		TIM3_REG_RESET();
	}else if (pTIMx == TIM4)
	{
		TIM4_REG_RESET();
	}else if (pTIMx == TIM5)
	{
		TIM5_REG_RESET();
	}else if (pTIMx == TIM6)
	{
		TIM6_REG_RESET();
	}else if (pTIMx == TIM7)
	{
		TIM7_REG_RESET();
	}else if (pTIMx == TIM8)
	{
		TIM8_REG_RESET();
	}else if (pTIMx == TIM9)
	{
		TIM9_REG_RESET();
	}else if (pTIMx == TIM10)
	{
		TIM10_REG_RESET();
	}else if (pTIMx == TIM11)
	{
		TIM11_REG_RESET();
	}else if (pTIMx == TIM12)
	{
		TIM12_REG_RESET();
	}else if (pTIMx == TIM13)
	{
		TIM13_REG_RESET();
	}else if (pTIMx == TIM14)
	{
		TIM14_REG_RESET();
	}

}


/*********************************************************************
 * @fn      		  - TIM_IRQConfig
 *
 * @brief             - enable and configure the IRQ interupt
 *
 * @param[in]         - irq number
 * @param[in]         - enable or disable
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              - none
 *********************************************************************/
void TIM_IRQConfig(uint8_t IRQ_Number, uint8_t EnorDi)
{
	//Enable or disable pin
	if(EnorDi == ENABLE)
	{
		//81 total inteturpts on STM32 there enable is spread across three reigsters
		if(IRQ_Number <= 31)
		{
			*NVIC_ISER0_BASE_ADDR |= (1 << IRQ_Number);
		}
		else if(IRQ_Number > 31 && IRQ_Number<64)
		{
			*NVIC_ISER1_BASE_ADDR |= (1 << IRQ_Number%32);
		}
	}
	else
	{
		//81 total inteturpts on STM32 there enable is spread across three reigsters
		if(IRQ_Number <= 31)
		{
			*NVIC_ICER0_BASE_ADDR &= ~(1 << IRQ_Number);
		}
		else if(IRQ_Number > 31 && IRQ_Number<64)
		{
			*NVIC_ICER1_BASE_ADDR &= ~(1 << IRQ_Number%32);
		}
	}

}


/*********************************************************************
 * @fn      		  - TIM_IRQPriorityConfig
 *
 * @brief             - enable and configure the IRQ priort
 *
 * @param[in]         - irq number
 * @param[in]         - priority level
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              - none
 *********************************************************************/

void TIM_IRQPriorityConfig(uint8_t IRQ_Number, uint8_t IRQ_Priority)
{
	//There exists multiple IPR registers with four IRQ priorties in each therefore we must find the reigster
	uint8_t IPRRegister = IRQ_Number /4;
	//A plus 4 is nesscary on the offset as only the higher four bits of the register are implemented
	uint8_t offset = (IRQ_Number % 4)*8 +4;

	//We are incrementing the pointer by 32 bits for each value to get to next register
	*(NVIC_PR_BASE_ADDR + IPRRegister) |= (IRQ_Priority << offset) ;
}
