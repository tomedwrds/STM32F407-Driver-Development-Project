/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Jan 9, 2023
 *      Author: tomed
 */


#include "stm32f407xx_spi_driver.h"

//Static to make them private as we only want them used in this file
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);


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


/*********************************************************************
 * @fn      		  - SPI_Get_FlagStatus
 *
 * @brief             - Checks the SR of SPI to see if a flag is set. Ie TXE flag for when transmit buffer empty
 *
 * @param[in]         - spi port
 * @param[in]         - flag to check for
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -none
 *********************************************************************/
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

/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             - moves data byte by byte into the Tx data to be sent.
 *
 * @param[in]         - spi port
 * @param[in]         - data to send
 * @param[in]         - length of data to send (bytes)
 *
 * @return            -  none
 *
 * @Note              - This send api makes use of polling so temporarily blocks the thread while waiting for Tx to be empty
 *********************************************************************/

void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t* dataToSend,uint32_t Len)
{
	while(Len > 0)
	{
		//Checks if all data has been moved out of the transmission buffer
		//This is to prevent data being over written in Tx
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

/*********************************************************************
 * @fn      		  - SPI_RecieveData
 *
 * @brief             - reads data byte by byte from the Rx buffer.
 *
 * @param[in]         - spi port
 * @param[in]         - location to store data
 * @param[in]         - length of data to send (bytes)
 *
 * @return            -  none
 *
 * @Note              - This send api makes use of polling so temporarily blocks the thread while waiting for Rx to be full
 *********************************************************************/

void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t* dataReceived,uint32_t Len)
{
	while(Len > 0)
	{
		//Hangs the programme until the buffer has data in it
		while(SPI_Get_FlagStatus(pSPIx,SPI_RXNE_FLAG) == RESET);

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

/*********************************************************************
 * @fn      		  - SPI_SendDataIT
 *
 * @brief             - enables and confugres interupt allowing all data to be sent through it
 *
 * @param[in]         - global spi handle structure
 * @param[in]         - data to send
 * @param[in]         - length of data to send (bytes)
 *
 * @return            -  none
 *
 * @Note              - This function configures then enables the interupt. This interupt will continue to be called until all data is sent.
 *********************************************************************/

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t* dataToSend,uint32_t Len)
{
	//Checks wether the SPI is in the process of sending data if so the function returns false and will be called again
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX)
	{
		//The SPIHandle should be globally inatlized to allow access to it in the interupt functions
		pSPIHandle->pTxBuffer = dataToSend;
		pSPIHandle->TxLen = Len;

		//Mark the spi state as busy in transmission so no other code can take over same SPI peripheal until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//Allows for an iterupt to occur when TXE bit is set in SR
		//With this set an interupt will be called when the Tx buffer is empty allowing data to be passed into
		pSPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_TXEIE);
	}
	return state;
}

/*********************************************************************
 * @fn      		  - SPI_RecieveDataIT
 *
 * @brief             - enables and confugres interupt allowing all data to be receieved through it
 *
 * @param[in]         - global spi handle structure
 * @param[in]         - data to send
 * @param[in]         - length of data to send (bytes)
 *
 * @return            -  none
 *
 * @Note              - none
 *********************************************************************/
uint8_t SPI_RecieveDataIT(SPI_Handle_t *pSPIHandle, uint8_t* dataReceived,uint32_t Len)
{
	//Checks wether the SPI is in the process of sending data if so the function returns false and will be called again
	uint8_t state = pSPIHandle->RxState;
	if(state != SPI_BUSY_IN_RX)
	{
		//The SPIHandle should be globally inatlized to allow access to it in the interupt functions
		pSPIHandle->pRxBuffer = dataReceived;
		pSPIHandle->RxLen = Len;

		//mark the spi state as busy in transmission so no other code can take over same SPI peripheal until transmission is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//Allows for an iterupt to occur when RNXE bit is set in SR
		//With this set an interupt will be called when the Rx buffer is empty allowing data to be recieved from it
		pSPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_RXNEIE);
	}
	return state;
}

/*********************************************************************
 * @fn      		  - SPI_PeripheralControl
 *
 * @brief             - turns on or off the spi
 *
 * @param[in]         - spi port
 * @param[in]         - enable or disable
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              - THIS SHOULD ONLY BE CALLED AFTER THE SPI HAS BEEN CONFIGURED
 *********************************************************************/
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

/*********************************************************************
 * @fn      		  - SPI_SSIConfig
 *
 * @brief             - if SSM (software slave managment) is used the value of this bit is forced on to the NSS pin
 *
 * @param[in]         - spi port
 * @param[in]         - enable or disable
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              - none
 *********************************************************************/
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
/*********************************************************************
 * @fn      		  - SPI_SSOEConfig
 *
 * @brief             - allows for toggling of a multi master confuration
 *
 * @param[in]         - spi port
 * @param[in]         - enable or disable
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              - THIS SHOULD BE SET TO 1 WHEN USING HARDWARE SLAVE MANAGMENT BUT NOT USING A MULTI MASTER SYSTEM
 *********************************************************************/
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


/*********************************************************************
 * @fn      		  - SPI_IRQConfig
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

void SPI_IRQPriorityConfig(uint8_t IRQ_Number, uint8_t IRQ_Priority)
{
	//There exists multiple IPR registers with four IRQ priorties in each therefore we must find the reigster
	uint8_t IPRRegister = IRQ_Number /4;
	//A plus 4 is nesscary on the offset as only the higher four bits of the register are implemented
	uint8_t offset = (IRQ_Number % 4)*8 +4;

	//We are incrementing the pointer by 32 bits for each value to get to next register
	*(NVIC_PR_BASE_ADDR + IPRRegister) |= (IRQ_Priority << offset) ;
}

/*********************************************************************
 * @fn      		  - SPI_IRQHandling
 *
 * @brief             - SPI can be interupted for multiple reasons. This function determines the reason of interupt and calls the approiate function.
 *
 * @param[in]         - SPI handle structure
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              - none
 *********************************************************************/
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	//Check for TXE to be true in status register and for interupts to be enabled for TXE
	if(pHandle->pSPIx->SR & (1<< SPI_SR_TXE) && pHandle->pSPIx->CR2 & (1<< SPI_CR2_TXEIE))
	{
		spi_txe_interrupt_handle(pHandle);
	}
	//Receveiving
	else if(pHandle->pSPIx->SR & (1<< SPI_SR_RXNE) && pHandle->pSPIx->CR2 & (1<< SPI_CR2_RXNEIE))
	{
		spi_rxe_interrupt_handle(pHandle);
	}
	//Overun flag
	else if(pHandle->pSPIx->SR & (1<< SPI_SR_OVR) && pHandle->pSPIx->CR2 & (1<< SPI_CR2_ERRIE))
	{
		spi_ovr_err_interrupt_handle (pHandle);
	}

}

/*********************************************************************
 * @fn      		  - spi_txe_interrupt_handle
 *
 * @brief             - interupt called to transmit txe data
 *
 * @param[in]         - SPI handle structure
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              - function is a helper function and is private to this header so carries a static typing
 *********************************************************************/
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//Determines size of data frame
	if(pSPIHandle->pSPIx->CR1 & (1<<SPI_CR1_DFF))
	{
		//16bit
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen -=2;

		//Increment the dataTosend pointer
		pSPIHandle->pTxBuffer+=2;

	}
	else
	{
		//8bit
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen  --;
		pSPIHandle->pTxBuffer++;
	}

	if(! pSPIHandle->TxLen)
	{
		//if tx len is not 0 close the spi transimmison and infor the application that tx is over
		SPI_CloseTransmisson(pSPIHandle);
	}
}

/*********************************************************************
 * @fn      		  - spi_rxe_interrupt_handle
 *
 * @brief             - interupt called to receive rxe data
 *
 * @param[in]         - SPI handle structure
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              - function is a helper function and is private to this header so carries a static typing
 *********************************************************************/
static void spi_rxe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->CR1 & (1<<SPI_CR1_DFF))
	{
		//16bit
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -=2;

		//Increment the dataTosend pointer
		pSPIHandle->pRxBuffer+=2;

	}
	else
	{
		//8bit
		*pSPIHandle->pRxBuffer = (uint8_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen  --;
		pSPIHandle->pRxBuffer++;
	}

	if(! pSPIHandle->RxLen)
	{
		//if tx len is not 0 close the spi transimmison and infor the application that tx is over
		SPI_CloseReception(pSPIHandle);
	}
}

/*********************************************************************
 * @fn      		  - spi_ovr_err_interrupt_handle
 *
 * @brief             -  handles ovr error
 *
 * @param[in]         - SPI handle structure
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              - function is a helper function and is private to this header so carries a static typing
 *********************************************************************/
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//clear the ovr flag by first reading DR then read the status register

	//This checks if TX is active. IF so flag should not be cleared otherwise transmission will be cancelled
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		SPI_ClearOVRFlag(pSPIHandle->pSPIx);
	}
	//inform the application of error an application error can be implemented below if user wishes
}

/*********************************************************************
 * @fn      		  - SPI_ClearOVRFlag
 *
 * @brief             -  clears ovr flag by reading both DR and SR
 *
 * @param[in]         - SPI port
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              - none
 *********************************************************************/
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t dummy;
	dummy = pSPIx->DR;
	dummy = pSPIx->SR;
	(void) dummy;
}

/*********************************************************************
 * @fn      		  - SPI_CloseTransmisson
 *
 * @brief             - closes transmisson of SPI port by disabling interupt
 *
 * @param[in]         - SPI port
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              - none
 *********************************************************************/
void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle)
{
	//TX interupt flag is cleared
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	//Pointer is reset to prevent a dangling pointer to no loner existing memory
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxState = SPI_READY;
	//SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
}

/*********************************************************************
 * @fn      		  - SPI_CloseReception
 *
 * @brief             - closes reception of SPI port by disabling interupt
 *
 * @param[in]         - SPI port
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              - none
 *********************************************************************/
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	//RX interupt flag is cleared
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	//Pointer is reset to prevent a dangling pointer to no loner existing memory
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxState = SPI_READY;
}

