/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Jan 9, 2023
 *      Author: tomed
 */


#include "stm32f407xx_gpio_driver.h"

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the gpio peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              - the base address of gpio peripheal is used as a parameter as it allows the inuse GPIO port to be distnigused
 *********************************************************************/

//Peripheal clock setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		//In header file for GPIO macros are set for all GPIO addresses allowing a comparinson to be made.
		//Switch isnt used as pointers cannot be passed as an arugment
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}
}

//Init/Deinit

/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - Intalize all parts of the GPIO pin such as mode, speed, pull up pull down, outtype, alternate func
 *
 * @param[in]         - handle struct that contains pointer to gpio and struct of pointer config
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              - none
 *********************************************************************/

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{

	//Enable peripheal clock
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
	//Set mode of GPIO some modes are interupt mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANLG)
	{
		//Bit shifted by two for each pin number
		pGPIOHandle->pGPIOx->MODER |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));
	}
	else
	{
		//EXTI can trigger an interupt on GPIO on falling edge, rising edge or both.
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IN_FT)
		{
			//Set the FTSR and clear the correpsoning RSTR bit
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IN_FT)
		{
			//Set the RTSR and clear the correpsoning FTSR bit
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IN_RFT)
		{
			//Set both RTSR and FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//Configure the GPIO port selection in SYSCF_EXTICR
		//Before using the peripheal it must be enabled
		SYSCFG_PCLK_EN();

		//There are four registers each storing 4EXTI therefore we must divide by four to find this register
		uint8_t subRegister = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber /4;
		uint8_t offset = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber %4;

		//Port code stores the value for the resptive port ie a, b, c ,d
		uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);


		SYSCFG->EXTICR[subRegister] |= portCode << offset*4;

		//Enable the exti interupt deilever
		EXTI->IMR |= (1<< pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}

	//Set speed - bits must be cleared as they may have a random unexpected value
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));
	pGPIOHandle->pGPIOx->OSPEEDR |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));

	//Set PUPD
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));
	pGPIOHandle->pGPIOx->PUPDR |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));

	//configure the output type
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << (2* pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));
	pGPIOHandle->pGPIOx->OTYPER |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ));

	//configure alternate fuctionality mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//Alternate functionality register has a higher and lower which we must determine what to use
		uint8_t HorLReg = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber /8;
		uint8_t relativeBitPos = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		pGPIOHandle->pGPIOx->AFR[HorLReg] &= ~(0xF << (4*relativeBitPos ));
		pGPIOHandle->pGPIOx->AFR[HorLReg] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*relativeBitPos ));
	}

}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - Uses an rcc reigster to full reset the GPIO
 *
 * @param[in]         - register name
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              - none
 *********************************************************************/

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}else if (pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - retrieves the value on the input pin
 *
 * @param[in]         - register
 * @param[in]         - pin number
 * @param[in]         -
 *
 * @return            -  input value
 *
 * @Note              - none
 *********************************************************************/

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	//Shift the value to the LSB then mask all other bits
	uint8_t value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}

/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - retrieves the value on the input port
 *
 * @param[in]         - register
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  input value
 *
 * @Note              - none
 *********************************************************************/

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	return (uint16_t)pGPIOx->IDR;
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             - sets a pin value
 *
 * @param[in]         - register
 * @param[in]         - pin number
 * @param[in]         - value (0 or 1)
 *
 * @return            -  none
 *
 * @Note              - none
 *********************************************************************/

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
	if(value == GPIO_PIN_SET)
	{
		//write 1 to set the bit
		pGPIOx->ODR |= (1<< PinNumber);
	}
	else
	{
		//clear the bit register
		pGPIOx->ODR |= ~(1<< PinNumber);
	}
}

/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             - allows setting of the lowest 16 bits of a register
 *
 * @param[in]         - register
 * @param[in]         - value
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              - only lower 16 bits of output data register are used.
 *********************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}

/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             - toggle a given pin
 *
 * @param[in]         - register
 * @param[in]         - pin number
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              - only lower 16 bits of output data register are used.
 *********************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1<< PinNumber);
}


/*********************************************************************
 * @fn      		  - GPIO_IRQConfig
 *
 * @brief             - toggles the gpio interupt
 *
 * @param[in]         - irq nunber
 * @param[in]         - enable or disable irq
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              - none
 *********************************************************************/

void GPIO_IRQConfig(uint8_t IRQ_Number, uint8_t EnorDi)
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
		else if(IRQ_Number >= 64 && IRQ_Number<96)
		{
			*NVIC_ISER2_BASE_ADDR |= (1 << IRQ_Number%32);
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
		else if(IRQ_Number >= 64 && IRQ_Number<96)
		{
			*NVIC_ICER2_BASE_ADDR &= ~(1 << IRQ_Number%32);
		}
	}




}


/*********************************************************************
 * @fn      		  - GPIO_IRQPriorityConfig
 *
 * @brief             - allows for setting of the priortiy of the GPIO IRQ
 *
 * @param[in]         - irq nunber
 * @param[in]         - priority level
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              - none
 *********************************************************************/
void GPIO_IRQPriorityConfig(uint8_t IRQ_Number, uint8_t IRQ_Priority)
{
	//There exists multiple IPR registers with four IRQ priorties in each therefore we must find the reigster
	uint8_t IPRRegister = IRQ_Number /4;
	//A plus 4 is nesscary on the offset as only the higher four bits of the register are implemented
	uint8_t offset = (IRQ_Number % 4)*8 +4;

	//We are incrementing the pointer by 32 bits for each value to get to next register
	*(NVIC_PR_BASE_ADDR + IPRRegister) |= (IRQ_Priority << offset) ;
}


/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             - sets the pending register to 1 causing the interupt to be cleared
 *
 * @param[in]         - pin number
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              - none
 *********************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pr numebr
	if(EXTI->PR & (1<<PinNumber))
	{
		//clear
		EXTI->PR |= (1<<PinNumber);
	}
}
