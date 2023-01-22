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
	TIM_Handle_t->pTIMx->PSC = TIM_Handle_t->TIMConfig.TIM_Prescalar;

	//Set the autoreload value
	TIM_Handle_t->pTIMx->ARR = TIM_Handle_t->TIMConfig.TIM_AutoReload;

	//Set the counter mode
	if(TIM_Handle_t->TIMConfig.TIM_CounterMode != TIM_COUNTER_UP)
	{
		if(TIM_Handle_t->TIMConfig.TIM_CounterMode == TIM_COUNTER_DOWN)
		{
			TIM_Handle_t->pTIMx->CR1 |= (1<<TIM_CR1_DIR);
		}
		else if(TIM_Handle_t->TIMConfig.TIM_CounterMode == TIM_COUNTER_CMS_IT_DOWN)
		{
			TIM_Handle_t->pTIMx->CR1 &= ~(0x3<<TIM_CR1_CMS);
			TIM_Handle_t->pTIMx->CR1 |= (0x1<<TIM_CR1_CMS);
		}
		else if(TIM_Handle_t->TIMConfig.TIM_CounterMode == TIM_COUNTER_CMS_IT_UP)
		{
			TIM_Handle_t->pTIMx->CR1 &= ~(0x3<<TIM_CR1_CMS);
			TIM_Handle_t->pTIMx->CR1 |= (0x2<<TIM_CR1_CMS);
		}
		else if(TIM_Handle_t->TIMConfig.TIM_CounterMode == TIM_COUNTER_CMS_IT_UPDOWN)
		{
			TIM_Handle_t->pTIMx->CR1 |= (0x3<<TIM_CR1_CMS);
		}
	}

	//Configure the Capture Compare
	if(TIM_Handle_t->TIMICConfig.IC_Channel == TIM_IC_Channel_OP)
	{
		//output mode
	}
	else if(TIM_Handle_t->TIMICConfig.IC_Channel == TIM_IC_Channel_IP1)
	{
		//input mode
		//configure CC into input mode and select TI channel
		TIM_Handle_t->pTIMx->CCMR1 |= (1<< TIM_CCMR1_CC1S);

		//configure the input capture filter
		TIM_Handle_t->pTIMx->CCMR1 |= (TIM_Handle_t->TIMICConfig.IC_Filter << TIM_CCMR1_IC1F);

		//set the polarity CCNP holds MSB CCP holds LSB
		//CCNP and CCP are seperated by a bit
		//To set CCNP LSB is masked to set CCP MSB is masked
		TIM_Handle_t->pTIMx->CCER |= ((TIM_Handle_t->TIMICConfig.IC_Polarity &= ~(0x1)) << TIM_CCER_CC1NP);
		TIM_Handle_t->pTIMx->CCER |= ((TIM_Handle_t->TIMICConfig.IC_Polarity &= ~(0x2)) << TIM_CCER_CC1P);

		//SEt the input prescalar
		TIM_Handle_t->pTIMx->CCMR1 |= (TIM_Handle_t->TIMICConfig.IC_Prescaler << TIM_CCMR1_IC1PSC);

	}
	else if(TIM_Handle_t->TIMICConfig.IC_Channel == TIM_IC_Channel_IP2)
	{
		//configure CC into input mode and select TI channel
		TIM_Handle_t->pTIMx->CCMR1 |= (1<< TIM_CCMR1_CC2S);

		//configure the input capture filter
		TIM_Handle_t->pTIMx->CCMR1 |= (TIM_Handle_t->TIMICConfig.IC_Filter << TIM_CCMR1_IC2F);

		//set the polarity CCNP holds MSB CCP holds LSB
		//CCNP and CCP are seperated by a bit
		//To set CCNP LSB is masked to set CCP MSB is masked
		TIM_Handle_t->pTIMx->CCER |= ((TIM_Handle_t->TIMICConfig.IC_Polarity &= ~(0x1)) << TIM_CCER_CC2NP);
		TIM_Handle_t->pTIMx->CCER |= ((TIM_Handle_t->TIMICConfig.IC_Polarity &= ~(0x2)) << TIM_CCER_CC2P);

		//SEt the input prescalar
		TIM_Handle_t->pTIMx->CCMR1 |= (TIM_Handle_t->TIMICConfig.IC_Prescaler << TIM_CCMR1_IC2PSC);

	}
	else if(TIM_Handle_t->TIMICConfig.IC_Channel == TIM_IC_Channel_IP3)
	{
		//input mode
		//configure CC into input mode and select TI channel
		TIM_Handle_t->pTIMx->CCMR2 |= (1<< TIM_CCMR2_CC3S);

		//configure the input capture filter
		TIM_Handle_t->pTIMx->CCMR2 |= (TIM_Handle_t->TIMICConfig.IC_Filter << TIM_CCMR2_IC3F);

		//set the polarity CCNP holds MSB CCP holds LSB
		//CCNP and CCP are seperated by a bit
		//To set CCNP LSB is masked to set CCP MSB is masked
		TIM_Handle_t->pTIMx->CCER |= ((TIM_Handle_t->TIMICConfig.IC_Polarity &= ~(0x1)) << TIM_CCER_CC3NP);
		TIM_Handle_t->pTIMx->CCER |= ((TIM_Handle_t->TIMICConfig.IC_Polarity &= ~(0x2)) << TIM_CCER_CC3P);

		//SEt the input prescalar
		TIM_Handle_t->pTIMx->CCMR2 |= (TIM_Handle_t->TIMICConfig.IC_Prescaler << TIM_CCMR2_IC3PSC);

	}
	else if(TIM_Handle_t->TIMICConfig.IC_Channel == TIM_IC_Channel_IP4)
	{
		//input mode
		//configure CC into input mode and select TI channel
		TIM_Handle_t->pTIMx->CCMR2 |= (1<< TIM_CCMR2_CC4S);

		//configure the input capture filter
		TIM_Handle_t->pTIMx->CCMR2 |= (TIM_Handle_t->TIMICConfig.IC_Filter << TIM_CCMR2_IC4F);

		//set the polarity CCNP holds MSB CCP holds LSB
		//CCNP and CCP are seperated by a bit
		//To set CCNP LSB is masked to set CCP MSB is masked
		TIM_Handle_t->pTIMx->CCER |= ((TIM_Handle_t->TIMICConfig.IC_Polarity &= ~(0x1)) << TIM_CCER_CC4NP);
		TIM_Handle_t->pTIMx->CCER |= ((TIM_Handle_t->TIMICConfig.IC_Polarity &= ~(0x2)) << TIM_CCER_CC4P);

		//SEt the input prescalar
		TIM_Handle_t->pTIMx->CCMR2 |= (TIM_Handle_t->TIMICConfig.IC_Prescaler << TIM_CCMR2_IC4PSC);
	}



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

/*********************************************************************
 * @fn      		  - TIM_Toggle
 *
 * @brief             - toggle the timer and enable the interupt generation
 *
 * @param[in]         - timer address
 * @param[in]         - enable or disable
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              - none
 *********************************************************************/

void TIM_Toggle(TIM_RegDef_t *pTIMx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		//pTIMx->DIER |= 1;
		pTIMx->CR1 |= 1;
	}
	else
	{
		//pTIMx->DIER &= ~1;
		pTIMx->CR1 &= ~1;
	}
}


/*********************************************************************
 * @fn      		  - TIM_CC_Toggle
 *
 * @brief             - enable the CC
 *
 * @param[in]         - timer address
 * @param[in]         - enable or disable
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              - none
 *********************************************************************/

void TIM_CC_Toggle(TIM_Handle_t *TIM_Handle_t,uint8_t EnorDi)
{
	if(TIM_Handle_t->TIMICConfig.IC_Channel == TIM_IC_Channel_IP1)
	{
		if(EnorDi == ENABLE)
		{
			TIM_Handle_t->pTIMx->DIER |= (1<< TIM_DIER_CCIE1);
			TIM_Handle_t->pTIMx->CCER |= (1<<TIM_CCER_CC1E);
		}
		else
		{
			TIM_Handle_t->pTIMx->DIER &= ~(1<< TIM_DIER_CCIE1);
			TIM_Handle_t->pTIMx->CCER &= ~(1<<TIM_CCER_CC1E);
		}
	}
	else if(TIM_Handle_t->TIMICConfig.IC_Channel == TIM_IC_Channel_IP2)
	{
		if(EnorDi == ENABLE)
		{
			TIM_Handle_t->pTIMx->DIER |= (1<< TIM_DIER_CCIE2);
			TIM_Handle_t->pTIMx->CCER |= (1<<TIM_CCER_CC2E);
		}
		else
		{
			TIM_Handle_t->pTIMx->DIER &= ~(1<< TIM_DIER_CCIE2);
			TIM_Handle_t->pTIMx->CCER &= ~(1<<TIM_CCER_CC2E);
		}
	}
	else if(TIM_Handle_t->TIMICConfig.IC_Channel == TIM_IC_Channel_IP3)
	{
		if(EnorDi == ENABLE)
		{
			TIM_Handle_t->pTIMx->DIER |= (1<< TIM_DIER_CCIE3);
			TIM_Handle_t->pTIMx->CCER |= (1<<TIM_CCER_CC3E);
		}
		else
		{
			TIM_Handle_t->pTIMx->DIER &= ~(1<< TIM_DIER_CCIE3);
			TIM_Handle_t->pTIMx->CCER &= ~(1<<TIM_CCER_CC3E);
		}
	}
	else if(TIM_Handle_t->TIMICConfig.IC_Channel == TIM_IC_Channel_IP4)
	{
		if(EnorDi == ENABLE)
		{
			TIM_Handle_t->pTIMx->DIER |= (1<< TIM_DIER_CCIE4);
			TIM_Handle_t->pTIMx->CCER |= (1<<TIM_CCER_CC4E);
		}
		else
		{
			TIM_Handle_t->pTIMx->DIER &= ~(1<< TIM_DIER_CCIE4);
			TIM_Handle_t->pTIMx->CCER &= ~(1<<TIM_CCER_CC4E);
		}
	}
}

/*********************************************************************
 * @fn      		  - TIM_GetFlagStatus
 *
 * @brief             - inspects the state of a flag in the status register
 *
 * @param[in]         - timer peripheal
 * @param[in]         - flag name
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              - none
 *********************************************************************/

uint8_t TIM_Get_FlagStatus(TIM_RegDef_t *pTIMx , uint32_t FlagName)
{
	if(pTIMx->SR & FlagName)
	{
		return SET;
	}
	else
	{
		return RESET;
	}
}

/*********************************************************************
 * @fn      		  - TIM_Clear_Update
 *
 * @brief             - clears the update flag for a timer
 *
 * @param[in]         - timer peripheal
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              - none
 *********************************************************************/
void TIM_Clear_Update(TIM_RegDef_t *pTIMx)
{
	pTIMx->SR &= ~1;
}





