/*
 * stm32f407xx_rcc_driver.c
 *
 *  Created on: 18/01/2023
 *      Author: tomed
 */

#include "stm32f407xx_rcc_driver.h"


uint16_t AHBPreScalarValues[8] = {2,4,8,16,64,128,256,512};
uint16_t APB1PreScalarValues[8] = {2,4,8,16};

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pClk1,SystemClk;

	//Get the clock source HSI, HSE, PLL
	uint8_t clkSrc = (RCC->CFGR >> 2) & 0x3;
	if(clkSrc == 0)
	{
		//HSI
		SystemClk = 16000000;
	}
	else if (clkSrc == 1)
	{
		//HSE
		SystemClk = 8000000;
	} else if(clkSrc == 2)
	{
		//PLL not used
	}

	//Get the AHB prescalar value
	uint8_t AHBSetPreScalar = (RCC->CFGR >> 4) & 0xF;
	uint16_t ahbp;

	if(AHBSetPreScalar < 8)
	{
		ahbp = 1;
	}
	else
	{
		//First value is 1000 next is 1001. Therefore -8 is nesscary to index array correctly
		ahbp = AHBPreScalarValues[AHBSetPreScalar-8];
	}

	//Get the APB1 prescalar value
	uint8_t APB1SetPreScalar = (RCC->CFGR >> 10) & 0x7;
	uint16_t apb1p;

	if(APB1SetPreScalar < 4)
	{
		apb1p = 1;
	}
	else
	{
		apb1p = APB1PreScalarValues[APB1SetPreScalar-4];
	}

	pClk1 = SystemClk/ahbp/apb1p;

	return pClk1;
}

uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t pClk,SystemClk;

	//Get the clock source HSI, HSE, PLL
	uint8_t clkSrc = (RCC->CFGR >> 2) & 0x3;
	if(clkSrc == 0)
	{
		//HSI
		SystemClk = 16000000;
	}
	else if (clkSrc == 1)
	{
		//HSE
		SystemClk = 8000000;
	} else if(clkSrc == 2)
	{
		//PLL not used
	}

	//Get the AHB prescalar value
	uint8_t AHBSetPreScalar = (RCC->CFGR >> 4) & 0xF;
	uint16_t ahbp;

	if(AHBSetPreScalar < 8)
	{
		ahbp = 1;
	}
	else
	{
		//First value is 1000 next is 1001. Therefore -8 is nesscary to index array correctly
		ahbp = AHBPreScalarValues[AHBSetPreScalar-8];
	}

	//Get the APB1 prescalar value
	uint8_t APB2SetPreScalar = (RCC->CFGR >> 13) & 0x7;
	uint16_t apb2p;

	if(APB2SetPreScalar < 4)
	{
		apb2p = 1;
	}
	else
	{
		apb2p = APB1PreScalarValues[APB2SetPreScalar-4];
	}

	pClk = SystemClk/ahbp/apb2p;

	return pClk;
}



