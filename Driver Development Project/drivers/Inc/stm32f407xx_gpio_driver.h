/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Jan 9, 2023
 *      Author: tomed
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"


/*
 * This is a config structure for a GPIO pins
 * */

typedef struct
{
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;

/*
 * This is a handle structure for a GPIO pins
 * */

typedef struct
{
	GPIO_RegDef_t *pGPIOx; //pointer, points to the base address of the gpio port to which the pin belongs too
	GPIO_PinConfig_t GPIO_PinConfig; //Holds the gpio pin config settings

}GPIO_Handle_t;


/*
 * GPIO Register specific macros
 * */

//Pin mode
#define GPIO_MODE_IN 			0
#define GPIO_MODE_OUT 			1
#define GPIO_MODE_ALTFN 		2
#define GPIO_MODE_ANLG 			3
#define GPIO_MODE_IN_FT			4
#define GPIO_MODE_IN_RT			5
#define GPIO_MODE_IN_RFT		6

//Output type
#define GPIO_OP_TYPE_PP			0
#define GPIO_OP_TYPE_OD			1

//Speed mode
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_HIGH			3

//Pull up pull down config
#define GPIO_NO_PUPD   			0
#define GPIO_PIN_PU				1
#define GPIO_PIN_PD				2

//Pin Numbers
#define GPIO_PIN_NO_0  				0
#define GPIO_PIN_NO_1  				1
#define GPIO_PIN_NO_2  				2
#define GPIO_PIN_NO_3  				3
#define GPIO_PIN_NO_4  				4
#define GPIO_PIN_NO_5  				5
#define GPIO_PIN_NO_6  				6
#define GPIO_PIN_NO_7  				7
#define GPIO_PIN_NO_8  				8
#define GPIO_PIN_NO_9  				9
#define GPIO_PIN_NO_10  			10
#define GPIO_PIN_NO_11 				11
#define GPIO_PIN_NO_12  			12
#define GPIO_PIN_NO_13 				13
#define GPIO_PIN_NO_14 				14
#define GPIO_PIN_NO_15 				15


/*
 * API function protoypes
 * */

//Peripheal clock setup
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

//Init/Deinit
void GPIO_Init(GPIO_Handle_t *GPIO_Handle_t);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

//Data read write
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

//IRQ config and ISR handling
void GPIO_IRQConfig(uint8_t IRQ_Number, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQ_Number, uint8_t IRQ_Priority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
