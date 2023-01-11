/*
 * stm32f407xx.h
 *
 *  Created on: Jan 9, 2023
 *      Author: tomed
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define __vo volatile


/*
 *  base addresses of flash and SRAM memories
 */

#define FLASH_BASEADDR			0x08000000U
#define SRAM1_BASEADDR			0x20000000U
#define SRAM2_BASEADDR			0x2001C000U
#define ROM						0x1FFF0000U
#define SRAM					SRAM1_BASEADDR

/*
 * base addresses of bus domains
 */

#define PERIPH_BASEADDR			0x40000000U
#define APB1PERIPH_BASE_ADDR	PERIPH_BASEADDR
#define APB2PERIPH_BASE_ADDR	0x40010000U
#define AHB1PERIPH_BASE_ADDR	0x40020000U
#define AHB2PERIPH_BASE_ADDR	0x50000000U

/*
 * base addresses of peripheals attached to AHB1 bus
 */

#define GPIOA_BASEADDR			(AHB1PERIPH_BASE_ADDR + 0x0000)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASE_ADDR + 0x0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASE_ADDR + 0x0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASE_ADDR + 0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASE_ADDR + 0x1000)
#define GPIOF_BASEADDR			(AHB1PERIPH_BASE_ADDR + 0x1400)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASE_ADDR + 0x1800)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASE_ADDR + 0x1C00)
#define GPIOI_BASEADDR			(AHB1PERIPH_BASE_ADDR + 0x2000)

#define RCC_BASEADDR			(AHB1PERIPH_BASE_ADDR + 0x3800)


/*
 * base addresses of peripheals attached to APB1 bus
 */

#define I2C1_BASEADDR			(APB1PERIPH_BASE_ADDR + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASE_ADDR + 0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASE_ADDR + 0x5C00)

#define SPI2_BASEADDR			(APB1PERIPH_BASE_ADDR + 0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASE_ADDR + 0x3C00)

#define USART2_BASEADDR			(APB1PERIPH_BASE_ADDR + 0x4400)
#define USART3_BASEADDR			(APB1PERIPH_BASE_ADDR + 0x4800)
#define UART4_BASEADDR			(APB1PERIPH_BASE_ADDR + 0x4C00)
#define UART5_BASEADDR			(APB1PERIPH_BASE_ADDR + 0x5000)


/*
 * base addresses of peripheals attached to APB2 bus
 */
#define EXTI_BASEADDR			(APB2PERIPH_BASE_ADDR + 0x3C00)
#define SPI1_BASEADDR			(APB2PERIPH_BASE_ADDR + 0x3000)
#define SYSCFG_BASEADDR			(APB2PERIPH_BASE_ADDR + 0x3800)
#define USART1_BASEADDR			(APB2PERIPH_BASE_ADDR + 0x1000)
#define USART6_BASEADDR			(APB2PERIPH_BASE_ADDR + 0x1400)


/*
 * peripheal register defention structures
 * __vo is used as short hand for volatile. Values in register can change therefore are volatile
 */

//GPIO registers
typedef struct
{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDR;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];
}GPIO_RegDef_t;



//RCC registers
typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
	uint32_t      RESERVED0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t      RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	uint32_t      RESERVED2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t      RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	uint32_t      RESERVED4;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t      RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	uint32_t      RESERVED6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;
	__vo uint32_t CKGATENR;
	__vo uint32_t DCKCFGR2;

} RCC_RegDef_t;

/*
 * peripheral register definition structure for EXTI
 */
typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;

}EXTI_RegDef_t;

/*
 * peripheral register definition structure for SYSCFG
 */
typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	uint32_t      RESERVED1[2];
	__vo uint32_t CMPCR;
	uint32_t      RESERVED2[2];
	__vo uint32_t CFGR;
} SYSCFG_RegDef_t;




/*
 * peripheal defentions (Peripheal base addresses typed casted to xxx_regdef_t)
 * */

#define GPIOA 					((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 					((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 					((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 					((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 					((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 					((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG					((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 					((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI 					((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC						((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI					((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG					((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)


/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()    		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()			(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()			(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()			(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()			(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()			(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()			(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()			(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()			(RCC->AHB1ENR |= (1 << 8))


/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN() 			(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() 			(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() 			(RCC->APB1ENR |= (1 << 23))


/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN() 			(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() 			(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() 			(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN() 			(RCC->APB2ENR |= (1 << 13))


/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCCK_EN() 		(RCC->APB2ENR |= (1 << 4))
#define USART2_PCCK_EN() 		(RCC->APB1ENR |= (1 << 17))
#define USART3_PCCK_EN() 		(RCC->APB1ENR |= (1 << 18))
#define UART4_PCCK_EN()  		(RCC->APB1ENR |= (1 << 19))
#define UART5_PCCK_EN()  		(RCC->APB1ENR |= (1 << 20))
#define USART6_PCCK_EN() 		(RCC->APB1ENR |= (1 << 5))

/*
 * Clock Enable Macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))


/*
 * Macros to reset GPIO Peripheals
 * */

//Do while allows for multiple code in a single line

#define GPIOA_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)

/*
 * This macro returns a code( between 0 to 7) for a given GPIO base address(x)
 * The macro checks if GPIO value matches and if so returns value if not checks another value
 */
#define GPIO_BASEADDR_TO_CODE(x)      ( (x == GPIOA)?0:\
										(x == GPIOB)?1:\
										(x == GPIOC)?2:\
										(x == GPIOD)?3:\
								        (x == GPIOE)?4:\
								        (x == GPIOF)?5:\
								        (x == GPIOG)?6:\
								        (x == GPIOH)?7: \
								        (x == GPIOI)?8:0)

/*
 * IRQ numer for EXTI interupts, SPI interupts, I2C interupts and U(S)ART interupts
 * */
#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2         36
#define IRQ_NO_SPI3         51
#define IRQ_NO_SPI4
#define IRQ_NO_I2C1_EV     31
#define IRQ_NO_I2C1_ER     32
#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39
#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53
#define IRQ_NO_USART6	    71

//Generic macros
#define ENABLE 					1
#define DISABLE 				0
#define SET						ENABLE
#define RESET					DISABLE
#define GPIO_PIN_SET			SET
#define GPIO_PIN_RESET			RESET


#include "stm32f407xx_gpio_driver.h"

#endif /* INC_STM32F407XX_H_ */
