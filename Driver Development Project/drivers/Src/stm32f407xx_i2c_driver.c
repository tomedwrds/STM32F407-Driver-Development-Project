/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: 15/01/2023
 *      Author: tomed
 */

#include "stm32f407xx_i2c_driver.h"
uint16_t AHBPreScalarValues[8] = {2,4,8,16,64,128,256,512};
uint16_t APB1PreScalarValues[8] = {2,4,8,16};

static void  I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle );
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle );



static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1<< I2C_CR1_START);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	//Slave address is 7 bits from 7:1. The first bit must be cleared as its r/w bit and we are wirting
	SlaveAddr <<= 1;
	SlaveAddr &= ~(1);
	pI2Cx->DR=SlaveAddr;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	//Slave address is 7 bits from 7:1. The first bit must be set as its r/w bit and we are readin
	SlaveAddr <<= 1;
	SlaveAddr |= 1;
	pI2Cx->DR=SlaveAddr;
}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle )
{
	if(pI2CHandle->RxSize == 1)
	{
		//read data in to the buffer
		*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen --;

		//wait until RXNE becomes 1
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));


	}

	if(pI2CHandle->RxSize > 1)
	{
		//There exists a delay between data being sent and read by 1 byte. Tehrefore acking must be disaled when the second byte is being read as the last byte is sent at this point
		if(pI2CHandle->RxLen == 2) // if last 2 bytes are remaning
		{
			//disable acking
			I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
		}

		//read the data from data register into the buffer
		*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;

		//increment the buffer address
		pI2CHandle->pRxBuffer++;

		//Decrement the length
		pI2CHandle->RxLen --;
	}

	if(pI2CHandle->RxLen == 0)
	{
		//close the i2c data reception and notify the application

		//generate stop condition
		if(pI2CHandle->Sr == I2C_DISABLE_SR)
			//Generate the stop condtion
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		I2C_CloseReceiveData(pI2CHandle);

		//Notfiy the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}

}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle )
{

	if (pI2CHandle->TxLen > 0)
	{
		//load data into dr increment buffer and decrement len
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
		pI2CHandle->TxLen --;
		pI2CHandle->pTxBuffer ++;
	}

}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummyRead = pI2CHandle->pI2Cx->SR1;
	//check device mode
	if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
	{
		//master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			//if data sent is only 1 byte ack must be cleared on first byte transmisson
			if(pI2CHandle->RxSize == 1)
			{

				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
			}
			dummyRead = pI2CHandle->pI2Cx->SR1;
			dummyRead = pI2CHandle->pI2Cx->SR2;
			(void)dummyRead; //dummy read needs to be type cast to void to prevent unused variable warning
		}
		else
		{
			//addr b
			dummyRead = pI2CHandle->pI2Cx->SR1;
			dummyRead = pI2CHandle->pI2Cx->SR2;
			(void)dummyRead; //dummy read needs to be type cast to void to prevent unused variable warning
		}
	}
	else
	{
		//device is in slave mode
		dummyRead = pI2CHandle->pI2Cx->SR1;
		dummyRead = pI2CHandle->pI2Cx->SR2;
		(void)dummyRead; //dummy read needs to be type cast to void to prevent unused variable warning
	}




}

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1<< I2C_CR1_STOP);
}

/*
 * Peripheral Clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		//In header file for I2C macros are set for all I2C addresses allowing a comparinson to be made.
		//Switch isnt used as pointers cannot be passed as an arugment
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}else if (pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}else if (pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
}

uint32_t RCC_Get_PCLK1Value(void)
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


/*
 * Init and De-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	//Enable the clock
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);



	//Set the freq field
	pI2CHandle->pI2Cx->CR2 |= (RCC_Get_PCLK1Value())/1000000;

	//Configure the device address
	pI2CHandle->pI2Cx->OAR1 |= (pI2CHandle->I2C_Config.I2C_DeviceAddress) << 1;

	//The 14th bit of the OAR1 register must always be mainted by software as 1
	pI2CHandle->pI2Cx->OAR1 |= (1<<14);

	//CCR calculation
	uint16_t ccr_value = 0;

	//First mode is configured
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//CCR = Tscl / 2TpClk1 | CCR = FPclk1 / 2Fscl
		ccr_value = RCC_Get_PCLK1Value() / (2*pI2CHandle->I2C_Config.I2C_SCLSpeed);
		//Mask out bits beyond 13 bits of ccr_value
		pI2CHandle->pI2Cx->CCR |=(ccr_value & 0xFFF);
	}
	else
	{
		//fast mode
		pI2CHandle->pI2Cx->CCR |= (1 << I2C_CCR_FS);
		pI2CHandle->pI2Cx->CCR |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY );

		//Configure the duty cycle based on value user provided
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			//Tlow : Thigh is 2:1
			ccr_value = RCC_Get_PCLK1Value() / (3*pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		else
		{
			//Tlow : Thigh is 16:9
			ccr_value = RCC_Get_PCLK1Value() / (25*pI2CHandle->I2C_Config.I2C_SCLSpeed);
		}
		pI2CHandle->pI2Cx->CCR |=(ccr_value & 0xFFF);
	}

	//Configure the rise time for I2C pin
	uint8_t tRise;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		//standard mode
		//trise = trisemax / tpclk1
		//trise = Fpclk / Frisemax
		tRise = (RCC_Get_PCLK1Value() / 1000000U) + 1;
	}
	else
	{
		tRise = ((RCC_Get_PCLK1Value()  *300)/ 1000000U) + 1;
	}
	pI2CHandle->pI2Cx->TRISE |=(tRise & 0x3F);
}

void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{

	//Enables the clock for the given SPI peripheal
	if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}else if (pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}else if (pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}
}


/*
 * Data Send and Receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	//Generate the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//Read SR1 register to check if SB has been set. Not operator is nesscary to hang the code until flag = 1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//Write the address of the slave to DR register
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	//Confirm the address phase is compelted by checking if ADDR bit set in SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));

	//Clear the ADDR flag by reading SR1 and SR2
	I2C_ClearADDRFlag(pI2CHandle);

	//Send all bytes of data until len is = 0
	while(Len > 0)
	{
		//When txe is set to 1 data register is empty and ready to recieve data
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
		pI2CHandle->pI2Cx->DR = *pTxbuffer;
		pTxbuffer ++;
		Len --;
	}

	//Wait for Txe and BTF to be set as they signal end of data transfer
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	//Generate stop condition
	if(Sr == I2C_DISABLE_SR)
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);


}
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr)
{
	//Generate the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//Confirm the start generation is completed by checking the SB flag in SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB));

	//Send the address of the slave with the r/nw bit to R(1)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	//Wait until the address phase is completed by checking the ADDR flag in the SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR));


	//Procedure to handle last byte and ending communcation
	if(Len == 1)
	{
		//Disable acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);


		//clear the addr flag
		I2C_ClearADDRFlag(pI2CHandle);

		//wait until RXNE becomes 1
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

		if(Sr == I2C_DISABLE_SR)
			//Generate the stop condtion
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		//read data in to the buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}

	if(Len > 1)
	{
		//Clear the addr flag
		I2C_ClearADDRFlag(pI2CHandle);
		//read the data until len becomes 0
		for(uint32_t i = Len; i > 0; i--)
		{
			//wait until rxne becomes 1. hang the program until read buffer is full
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

			//There exists a delay between data being sent and read by 1 byte. Tehrefore acking must be disaled when the second byte is being read as the last byte is sent at this point
			if(i == 2) // if last 2 bytes are remaning
			{
				//disable acking
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
				if(Sr == I2C_DISABLE_SR)
					//generate stop condition
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			//read the data from data register into the buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			//increment the buffer address
			pRxBuffer++;
		}
	}

	//re-enable accking
	I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
}
/*********************************************************************
 * @fn      		  - I2C_MasterSendDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -  Complete the below code . Also include the function prototype in header file

 */
uint8_t  I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxBuffer, uint32_t Len,uint8_t SlaveAddr,uint8_t Sr)
{

	//Get the current state of I2C to check if its already in use
	uint8_t busystate = pI2CHandle->TxRxState;

	//Check if the state isnt busy recieving or transmitting
	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);


	}

	return busystate;

}
/*********************************************************************
 * @fn      		  - I2C_MasterReceiveDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Complete the below code . Also include the fn prototype in header file

 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr)
{

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = Len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);

	}

	return busystate;
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	//Reset the values
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL; // set to null to prevent dangling pointer
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	//Acking has to be renabled after being disabled to close reception
	if(pI2CHandle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(pI2CHandle->pI2Cx,ENABLE);
	}
}
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	//Implement the code to disable ITBUFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

	//Implement the code to disable ITEVFEN Control Bit
	pI2CHandle->pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

	//Reseting the values
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL; // set to null to prevent dangling pointer
	pI2CHandle->TxLen = 0;
}


void I2C_SlaveSendData(I2C_RegDef_t *pI2C,uint8_t data)
{
	//only single byte sent at a time so byte loaded into DR
	pI2C->DR = data;
}
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C)
{
	//Value of DR is returned
	return (uint8_t) pI2C->DR;
}

/*
 * IRQ Configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQ_Number, uint8_t EnorDi)
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
		else if(IRQ_Number >= 64)
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
		else if(IRQ_Number >= 64)
		{
			*NVIC_ICER2_BASE_ADDR |= (1 << IRQ_Number%32);
		}
	}
}
void I2C_IRQPriorityConfig(uint8_t IRQ_Number, uint32_t IRQ_Priority)
{
	//There exists multiple IPR registers with four IRQ priorties in each therefore we must find the reigster
	uint8_t IPRRegister = IRQ_Number /4;
	//A plus 4 is nesscary on the offset as only the higher four bits of the register are implemented
	uint8_t offset = (IRQ_Number % 4)*8 +4;

	//We are incrementing the pointer by 32 bits for each value to get to next register
	*(NVIC_PR_BASE_ADDR + IPRRegister) |= (IRQ_Priority << offset) ;
}
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

	uint32_t eventEREnabled,BERRSet, ARLOSet,AFSet,OVRSet,TIMEOUTSet;

    //Know the status of  ITERREN control bit in the CR2
	eventEREnabled = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);
	BERRSet = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BERR);
	ARLOSet = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ARLO);
	AFSet = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_AF);
	OVRSet = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_OVR);
	TIMEOUTSet = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TIMEOUT);


	if(eventEREnabled  && BERRSet )
	{
		//Bus error

		//Implement the code to clear the buss error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

		//Implement the code to notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

	if(eventEREnabled  && ARLOSet)
	{
		//arbitration lost error
		//Implement the code to clear the arbitration lost error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);
	}

	if(eventEREnabled  && AFSet)
	{
		// ACK failure error
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
	}


	if(eventEREnabled  && OVRSet)
	{
		//Overrun/underrun

	    //Implement the code to clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
	}


	if(eventEREnabled  && TIMEOUTSet)
	{
		//Time out error

	    //Implement the code to clear the Time out error flag
		pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);
		//Implement the code to notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
	}

}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t eventITEnabled,buffITEnabled,SBSet,ADDRSet,BTFSet,STOPFSet,TxESet,RxNESet;

	//Check if the respective interupts handlers are enabled
	eventITEnabled = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	buffITEnabled = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

	//Flags from SR
	SBSet = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB);
	ADDRSet = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR);
	BTFSet = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF);
	STOPFSet = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_STOPF);
	TxESet = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE);
	RxNESet = I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE);

	//Handle interupt for SB event
	if(eventITEnabled && SBSet)
	{
		//SB flag is set
		//This will never be executed in slave mode as SB is always 0 in slave mode
		//Address of slave must be set. Depedning on Tx or Rx a diffrent function must be used to correctly set the r/w bit
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	if(eventITEnabled && ADDRSet)
	{
		//ADDR flag is set
		//Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);
	}

	if(eventITEnabled && BTFSet)
	{
		//BTF flag is set both shift and data register are empty or full
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX && TxESet && pI2CHandle->TxLen == 0)
		{
			//Wait for Txe and BTF to be set as they signal end of data transfer

			//Generate stop condition
			if(pI2CHandle->Sr == I2C_DISABLE_SR)
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

			//reset all member elements of the handle structure
			I2C_CloseSendData(pI2CHandle);

			//call the callback
			I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
		}
		//Closing of communication is not done through BTF for Rx

	}
	if(eventITEnabled && STOPFSet)
	{
		//STOPF flag is set
		//This flag is only set by a slave
		//The STOPF flag is cleared by reading SR1 and writing CR1
		pI2CHandle->pI2Cx->CR1 |= 0x00000;

		//call the callback
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}
	if(buffITEnabled && eventITEnabled && TxESet)
	{
		//Data can only be sent if device is master
		if(pI2CHandle->pI2Cx->SR2 & (1<< I2C_SR2_MSL))
		{
			//TxE flag is set
			//Transmission buffer empty so we have to do data transmisson
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}
		else
		{
			//Device is slave and must send a byte check if slave is in trasmitter mode
			if(pI2CHandle->pI2Cx->SR2 & ( 1<< I2C_SR2_TRA))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);

			}
		}
	}
	if(buffITEnabled && eventITEnabled && RxNESet)
	{
		//Data can only be sent if device is master
		if(pI2CHandle->pI2Cx->SR2 & (1<< I2C_SR2_MSL))
		{
			//RxNE flag is set
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}
		else
		{
			//Device is slave and must send a byte check if slave is in reciever mode
			if(!(pI2CHandle->pI2Cx->SR2 & ( 1<< I2C_SR2_TRA)))
			{
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);

			}
		}
}

}


/*
 * Other Peripheral Control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= (1<< I2C_CR1_PE);
	}
	else
	{
		pI2Cx->CR1 &= ~(1<< I2C_CR1_PE);
	}
}
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
	{
		return SET;
	}
	else
	{
		return RESET;
	}
}
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
	else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}



void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx,uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		//Implement the code to enable ITBUFEN Control Bit
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}
	else
	{
		//Implement the code to enable ITBUFEN Control Bit
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

		//Implement the code to enable ITERREN Control Bit
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITERREN);
	}
}

/*
 * Application callback
 */

