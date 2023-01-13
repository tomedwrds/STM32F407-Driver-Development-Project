/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Jan 9, 2023
 *      Author: tomed
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

/*
 *  Configuration structure for SPIx peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;


/*
 *Handle structure for SPIx peripheral
 */
typedef struct
{
	SPI_RegDef_t 	*pSPIx;
	SPI_Config_t 	SPIConfig;
//	uint8_t 		*pTxBuffer;
//	uint8_t 		*pRxBuffer;
//	uint32_t 		TxLen;
//	uint32_t 		RxLen;
//	uint8_t 		TxState;
//	uint8_t 		RxState;
}SPI_Handle_t;


/*
 * SPI application states
 */
#define SPI_READY 					0
#define SPI_BUSY_IN_RX 				1
#define SPI_BUSY_IN_TX 				2

/*
 * Possible SPI Application events
 */
#define SPI_EVENT_TX_CMPLT   		1
#define SPI_EVENT_RX_CMPLT   		2
#define SPI_EVENT_OVR_ERR    		3
#define SPI_EVENT_CRC_ERR    		4



/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER    	1
#define SPI_DEVICE_MODE_SLAVE     	0


/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD           	1
#define SPI_BUS_CONFIG_HD               2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY   3

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2             0
#define SPI_SCLK_SPEED_DIV4            	1
#define SPI_SCLK_SPEED_DIV8            	2
#define SPI_SCLK_SPEED_DIV16           	3
#define SPI_SCLK_SPEED_DIV32          	4
#define SPI_SCLK_SPEED_DIV64          	5
#define SPI_SCLK_SPEED_DIV128          	6
#define SPI_SCLK_SPEED_DIV256          	7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS 	0
#define SPI_DFF_16BITS  1

/*
 * @CPOL
 */
#define SPI_CPOL_HIGH 1
#define SPI_CPOL_LOW 0

/*
 * @CPHA
 */
#define SPI_CPHA_HIGH 1
#define SPI_CPHA_LOW 0

/*
 * @SPI_SSM
 */
#define SPI_SSM_EN     1
#define SPI_SSM_DI     0

/*
 * SPI related status flags definitions
 */
#define SPI_TXE_FLAG    ( 1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG   ( 1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG   ( 1 << SPI_SR_BSY)

/*
 * API function protoypes
 * */

//Peripheal clock setup
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

//Init/Deinit
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

//Data send and recieve
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t* dataToSend,uint32_t Len);
void SPI_RecieveData(SPI_RegDef_t *pSPIx, uint8_t* dataReceived,uint32_t Len);
uint8_t SPI_Get_FlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName);

//IRQ config and ISR handling
void SPI_IRQConfig(uint8_t IRQ_Number, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQ_Number, uint8_t IRQ_Priority);





/*
 * Other Peripheral Control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);
//uint8_t I2C_DeviceMode(I2C_RegDef_t *I2Cx);



#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
