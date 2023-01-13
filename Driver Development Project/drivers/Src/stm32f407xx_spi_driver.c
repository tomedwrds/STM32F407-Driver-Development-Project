/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Jan 9, 2023
 *      Author: tomed
 */


#include "stm32f407xx_spi_driver.h"

/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -none
 *********************************************************************/

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
		{
			//Enables the clock for the given SPI peripheal
			if(pSPIx == SPI1)
			{
				SPI1_PCLK_EN();
			}else if (pSPIx == SPI2)
			{
				SPI2_PCLK_EN();
			}else if (pSPIx == SPI3)
			{
				SPI3_PCLK_EN();
			}
		}
}

/*********************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             - Initalize a given SPI port for use
 *
 * @param[in]         - spi config structure
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -none
 *********************************************************************/


void SPI_Init(SPI_Handle_t *pSPIHandle)
{

	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//First the SPI_CR1 is configured
	//As a large amount of bits need to be set a temp register is used. This will then be copied into the actual register.
	uint16_t tempReg = 0;

	//Configure the device mode mater or slave
	tempReg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

	//Configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//Enable BIDI by clearing bit
		tempReg &= ~(1 << SPI_CR1_BIDIMODE);

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//Disable BIDI by setting bit
		tempReg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//Enable BIDI by clearing bit and set RX only for recieve only
		tempReg &= ~(1 << SPI_CR1_BIDIMODE);
		tempReg |= (1 << SPI_CR1_RXONLY);
	}

	//Set the SCLK divisor
	tempReg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//Set the DFF
	tempReg |= pSPIHandle->SPIConfig.SPI_DFF<< SPI_CR1_DFF;

	//Set the CPOL
	tempReg |= pSPIHandle->SPIConfig.SPI_CPOL<< SPI_CR1_CPOL;

	//Set the CPHA
	tempReg |= pSPIHandle->SPIConfig.SPI_CPHA<< SPI_CR1_CPHA;

	//Set the SSM
	tempReg |= pSPIHandle->SPIConfig.SPI_SSM<< SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempReg;




}

/*********************************************************************
 * @fn      		  - SPI_DeInit
 *
 * @brief             - Clears the SPI peripheal register
 *
 * @param[in]         - spi port
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -none
 *********************************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	//Enables the clock for the given SPI peripheal
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}else if (pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}else if (pSPIx == SPI3)
	{
		SPI2_REG_RESET();
	}
}

uint8_t SPI_Get_FlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName)
{
	if(pSPIx->SR & flagName)
	{
		return SET;
	}
	else
	{
		return RESET;
	}
}

//Blocking call. While loop present so fuction does return until all data sent.


void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t* dataToSend,uint32_t Len)
{
	while(Len > 0)
	{
		//Check if the TXE is set and Tx buffer empty. Program hangs until this happens
		while(SPI_Get_FlagStatus(pSPIx,SPI_TXE_FLAG) == RESET);

		//Check dff
		if(pSPIx->CR1 & (1<<SPI_CR1_DFF))
		{
			//16bit
			pSPIx->DR = *((uint16_t*)dataToSend);
			Len -=2;

			//Increment the dataTosend pointer
			dataToSend+=2;

		}
		else
		{
			//8bit
			pSPIx->DR = *dataToSend;
			Len --;
			dataToSend++;
		}

	}

}



void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t* dataReceived,uint32_t Len)
{
	while(Len > 0)
	{
		//Check if the RXE is not empty
		while(SPI_Get_FlagStatus(pSPIx,SPI_RXNE_FLAG) == SET);

		//Check dff
		if(pSPIx->CR1 & (1<<SPI_CR1_DFF))
		{
			//16bit
			 *((uint16_t*)dataReceived) = pSPIx->DR;
			Len -=2;

			//Increment the dataTosend pointer
			dataReceived+=2;

		}
		else
		{
			//8bit
			*dataReceived = pSPIx->DR;
			Len --;
			dataReceived++;
		}

	}

}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1<< SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1<< SPI_CR1_SPE);
	}
}


void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1<< SPI_CR1_SSI);
	}
	else
	{
		pSPIx->CR1 &= ~(1<< SPI_CR1_SSI);
	}
}

void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
		{
			pSPIx->CR2 |= (1<< SPI_CR2_SSOE);
		}
		else
		{
			pSPIx->CR2 &= ~(1<< SPI_CR2_SSOE);
		}
}


void SPI_IRQConfig(uint8_t IRQ_Number, uint8_t EnorDi)
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
 * @fn      		  - SPI_IRQPriorityConfig
 *
 * @brief             - allows for setting of the priortiy of the SPI IRQ
 *
 * @param[in]         - irq nunber
 * @param[in]         - priority level
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              - none
 *********************************************************************/
void SPI_IRQPriorityConfig(uint8_t IRQ_Number, uint8_t IRQ_Priority)
{
	//There exists multiple IPR registers with four IRQ priorties in each therefore we must find the reigster
	uint8_t IPRRegister = IRQ_Number /4;
	//A plus 4 is nesscary on the offset as only the higher four bits of the register are implemented
	uint8_t offset = (IRQ_Number % 4)*8 +4;

	//We are incrementing the pointer by 32 bits for each value to get to next register
	*(NVIC_PR_BASE_ADDR + IPRRegister) |= (IRQ_Priority << offset) ;
}