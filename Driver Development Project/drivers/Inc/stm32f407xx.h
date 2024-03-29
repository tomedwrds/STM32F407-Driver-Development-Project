/*
 * stm32f407xx.h
 *
 *  Created on: Jan 9, 2023
 *      Author: tomed
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>
#include <stddef.h>

#define __vo volatile


/*
 * PROCESSOR SPECIFIC DETAILS AND MACROS
 * */

#define NVIC_ISER0_BASE_ADDR        ( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1_BASE_ADDR        ( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2_BASE_ADDR        ( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3_BASE_ADDR        ( (__vo uint32_t*)0xE000E10c )


/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0_BASE_ADDR 		((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1_BASE_ADDR		((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2_BASE_ADDR  		((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3_BASE_ADDR		((__vo uint32_t*)0XE000E18C)


/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 			((__vo uint32_t*)0xE000E400)

/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED  4



/*
 * MICROCONTROLLER MACROS
 * */

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

#define TIM2_BASEADDR			(APB1PERIPH_BASE_ADDR +0x0)
#define TIM3_BASEADDR			(APB1PERIPH_BASE_ADDR +0x400)
#define TIM4_BASEADDR			(APB1PERIPH_BASE_ADDR +0x800)
#define TIM5_BASEADDR			(APB1PERIPH_BASE_ADDR +0xC00)
#define TIM6_BASEADDR			(APB1PERIPH_BASE_ADDR +0x1000)
#define TIM7_BASEADDR			(APB1PERIPH_BASE_ADDR +0x1400)
#define TIM12_BASEADDR			(APB1PERIPH_BASE_ADDR +0x1800)
#define TIM13_BASEADDR			(APB1PERIPH_BASE_ADDR +0x1C00)
#define TIM14_BASEADDR			(APB1PERIPH_BASE_ADDR +0x2000)

#define I2C1_BASEADDR			(APB1PERIPH_BASE_ADDR + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASE_ADDR + 0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASE_ADDR + 0x5C00)

#define SPI2_BASEADDR			(APB1PERIPH_BASE_ADDR + 0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASE_ADDR + 0x3C00)

#define USART2_BASEADDR			(APB1PERIPH_BASE_ADDR + 0x4400)
#define USART3_BASEADDR			(APB1PERIPH_BASE_ADDR + 0x4800)
#define UART4_BASEADDR			(APB1PERIPH_BASE_ADDR + 0x4C00)
#define UART5_BASEADDR			(APB1PERIPH_BASE_ADDR + 0x5000)

#define PWR_BASEADDR			(APB1PERIPH_BASE_ADDR + 0x7000)


/*
 * base addresses of peripheals attached to APB2 bus
 */

#define TIM1_BASEADDR			(APB2PERIPH_BASE_ADDR + 0x0)
#define TIM8_BASEADDR			(APB2PERIPH_BASE_ADDR + 0x0400)
#define TIM9_BASEADDR			(APB2PERIPH_BASE_ADDR + 0x4000)
#define TIM10_BASEADDR			(APB2PERIPH_BASE_ADDR + 0x4400)
#define TIM11_BASEADDR			(APB2PERIPH_BASE_ADDR + 0x4800)


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

//EXTI registers
typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;

}EXTI_RegDef_t;

//Sysconfig registers
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


//SPI registers
typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;
} SPI_RegDef_t;


//I2C register
typedef struct
{
  __vo uint32_t CR1;
  __vo uint32_t CR2;
  __vo uint32_t OAR1;
  __vo uint32_t OAR2;
  __vo uint32_t DR;
  __vo uint32_t SR1;
  __vo uint32_t SR2;
  __vo uint32_t CCR;
  __vo uint32_t TRISE;
  __vo uint32_t FLTR;
}I2C_RegDef_t;

//USART
typedef struct
{
	__vo uint32_t SR;         /*!< TODO,     										Address offset: 0x00 */
	__vo uint32_t DR;         /*!< TODO,     										Address offset: 0x04 */
	__vo uint32_t BRR;        /*!< TODO,     										Address offset: 0x08 */
	__vo uint32_t CR1;        /*!< TODO,     										Address offset: 0x0C */
	__vo uint32_t CR2;        /*!< TODO,     										Address offset: 0x10 */
	__vo uint32_t CR3;        /*!< TODO,     										Address offset: 0x14 */
	__vo uint32_t GTPR;       /*!< TODO,     										Address offset: 0x18 */
} USART_RegDef_t;


//Timer registers
typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SMCR;
	__vo uint32_t DIER;
	__vo uint32_t SR;
	__vo uint32_t EGR;
	__vo uint32_t CCMR1;
	__vo uint32_t CCMR2;
	__vo uint32_t CCER;
	__vo uint32_t CNT;
	__vo uint32_t PSC;
	__vo uint32_t ARR;
	__vo uint32_t RCR;
	__vo uint32_t CCR1;
	__vo uint32_t CCR2;
	__vo uint32_t CCR3;
	__vo uint32_t CCR4;
	__vo uint32_t BTDR;
	__vo uint32_t DMAR;
}TIM_RegDef_t;


//PWR Register
typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t CSR;
}PWR_RegDef_t;

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

#define SPI1  				((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2  				((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3  				((SPI_RegDef_t*)SPI3_BASEADDR)

#define I2C1  				((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2  				((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3  				((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1  			((USART_RegDef_t*)USART1_BASEADDR)
#define USART2  			((USART_RegDef_t*)USART2_BASEADDR)
#define USART3  			((USART_RegDef_t*)USART3_BASEADDR)
#define UART4  				((USART_RegDef_t*)UART4_BASEADDR)
#define UART5  				((USART_RegDef_t*)UART5_BASEADDR)
#define USART6  			((USART_RegDef_t*)USART6_BASEADDR)

#define TIM1  				((TIM_RegDef_t*)TIM1_BASEADDR)
#define TIM2  				((TIM_RegDef_t*)TIM2_BASEADDR)
#define TIM3  				((TIM_RegDef_t*)TIM3_BASEADDR)
#define TIM4  				((TIM_RegDef_t*)TIM4_BASEADDR)
#define TIM5  				((TIM_RegDef_t*)TIM5_BASEADDR)
#define TIM6  				((TIM_RegDef_t*)TIM6_BASEADDR)
#define TIM7  				((TIM_RegDef_t*)TIM7_BASEADDR)
#define TIM8  				((TIM_RegDef_t*)TIM8_BASEADDR)
#define TIM9  				((TIM_RegDef_t*)TIM9_BASEADDR)
#define TIM10  				((TIM_RegDef_t*)TIM10_BASEADDR)
#define TIM11				((TIM_RegDef_t*)TIM11_BASEADDR)
#define TIM12 				((TIM_RegDef_t*)TIM12_BASEADDR)
#define TIM13 				((TIM_RegDef_t*)TIM13_BASEADDR)
#define TIM14 				((TIM_RegDef_t*)TIM14_BASEADDR)

#define RCC						((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI					((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG					((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define PWR						((PWR_RegDef_t*) PWR_BASEADDR)


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
#define USART1_PCLK_EN() 		(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN() 		(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN() 		(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()  		(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()  		(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN() 		(RCC->APB2ENR |= (1 << 5))

/*
 * Clock Enable Macros for TIMx peripherals
 */
#define TIM1_PCLK_EN() 			(RCC->APB2ENR |= (1 << 0))
#define TIM2_PCLK_EN() 			(RCC->APB1ENR |= (1 << 0))
#define TIM3_PCLK_EN() 			(RCC->APB1ENR |= (1 << 1))
#define TIM4_PCLK_EN() 			(RCC->APB1ENR |= (1 << 2))
#define TIM5_PCLK_EN() 			(RCC->APB1ENR |= (1 << 3))
#define TIM6_PCLK_EN() 			(RCC->APB1ENR |= (1 << 4))
#define TIM7_PCLK_EN() 			(RCC->APB1ENR |= (1 << 5))
#define TIM8_PCLK_EN() 			(RCC->APB2ENR |= (1 << 1))
#define TIM9_PCLK_EN() 			(RCC->APB2ENR |= (1 << 16))
#define TIM10_PCLK_EN() 		(RCC->APB2ENR |= (1 << 17))
#define TIM11_PCLK_EN() 		(RCC->APB2ENR |= (1 << 18))
#define TIM12_PCLK_EN() 		(RCC->APB1ENR |= (1 << 6))
#define TIM13_PCLK_EN() 		(RCC->APB1ENR |= (1 << 7))
#define TIM14_PCLK_EN() 		(RCC->APB1ENR |= (1 << 8))


/*
 * Clock Enable Macros for SYSCFG peripherals
 */
#define SYSCFG_PCLK_EN() (RCC->APB2ENR |= (1 << 14))



/*
 * Clock Enable Macros for PWR peripherals
 */
#define PWR_PCLK_EN() (RCC->APB1ENR |= (1 << 28))



/*
 * Macros to reset GPIO and SPI Peripheals
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

#define SPI1_REG_RESET()               do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); }while(0)

#define I2C1_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21)); }while(0)
#define I2C2_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22)); }while(0)
#define I2C3_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR &= ~(1 << 23)); }while(0)

#define USART1_REG_RESET()               do{ (RCC->APB2RSTR |= (1 << 4)); (RCC->APB2RSTR &= ~(1 << 4)); }while(0)
#define USART2_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 17)); (RCC->APB1RSTR &= ~(1 << 17)); }while(0)
#define USART3_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 18)); (RCC->APB1RSTR &= ~(1 << 18)); }while(0)
#define USART4_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 19)); (RCC->APB1RSTR &= ~(1 << 19)); }while(0)
#define USART5_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 20)); (RCC->APB1RSTR &= ~(1 << 20)); }while(0)
#define USART6_REG_RESET()               do{ (RCC->APB2RSTR |= (1 << 5)); (RCC->APB2RSTR &= ~(1 << 5)); }while(0)

#define TIM1_REG_RESET()               	do{ (RCC->APB2RSTR |= (1 << 0)); (RCC->APB2RSTR &= ~(1 << 0)); }while(0)
#define TIM2_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 0)); (RCC->APB1RSTR &= ~(1 << 0)); }while(0)
#define TIM3_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 1)); (RCC->APB1RSTR &= ~(1 << 1)); }while(0)
#define TIM4_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 2)); (RCC->APB1RSTR &= ~(1 << 2)); }while(0)
#define TIM5_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 3)); (RCC->APB1RSTR &= ~(1 << 3)); }while(0)
#define TIM6_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 4)); (RCC->APB1RSTR &= ~(1 << 4)); }while(0)
#define TIM7_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 5)); (RCC->APB1RSTR &= ~(1 << 5)); }while(0)
#define TIM8_REG_RESET()               do{ (RCC->APB2RSTR |= (1 << 1)); (RCC->APB2RSTR &= ~(1 << 1)); }while(0)
#define TIM9_REG_RESET()               do{ (RCC->APB2RSTR |= (1 << 16)); (RCC->APB2RSTR &= ~(1 << 16)); }while(0)
#define TIM10_REG_RESET()               do{ (RCC->APB2RSTR |= (1 << 17)); (RCC->APB2RSTR &= ~(1 << 17)); }while(0)
#define TIM11_REG_RESET()               do{ (RCC->APB2RSTR |= (1 << 18)); (RCC->APB2RSTR &= ~(1 << 18)); }while(0)
#define TIM12_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 6)); (RCC->APB1RSTR &= ~(1 << 6)); }while(0)
#define TIM13_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 7)); (RCC->APB1RSTR &= ~(1 << 7)); }while(0)
#define TIM14_REG_RESET()               do{ (RCC->APB1RSTR |= (1 << 8)); (RCC->APB1RSTR &= ~(1 << 8)); }while(0)




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
#define IRQ_NO_EXTI0 				6
#define IRQ_NO_EXTI1 				7
#define IRQ_NO_EXTI2 				8
#define IRQ_NO_EXTI3 				9
#define IRQ_NO_EXTI4 				10
#define IRQ_NO_EXTI9_5 				23
#define IRQ_NO_EXTI15_10 			40
#define IRQ_NO_SPI1					35
#define IRQ_NO_SPI2         		36
#define IRQ_NO_SPI3         		51
#define IRQ_NO_SPI4
#define IRQ_NO_I2C1_EV     			31
#define IRQ_NO_I2C1_ER     			32
#define IRQ_NO_I2C2_EV     			33
#define IRQ_NO_I2C2_ER     			34
#define IRQ_NO_I2C3_EV     			72
#define IRQ_NO_I2C3_ER     			73
#define IRQ_NO_USART1	    		37
#define IRQ_NO_USART2	    		38
#define IRQ_NO_USART3	    		39
#define IRQ_NO_UART4	    		52
#define IRQ_NO_UART5	    		53
#define IRQ_NO_USART6	    		71
#define IRQ_NO_TIM1_BRK_TIM9 		24
#define IRQ_NO_TIM1_UP_TIM10 		25
#define IRQ_NO_TIM1_TRG_COM_TIM11 	26
#define IRQ_NO_TIM1_CC 				27
#define IRQ_NO_TIM2 				28
#define IRQ_NO_TIM3 				29
#define IRQ_NO_TIM4 				30
#define IRQ_NO_TIM8_BRK_TIM12 		43
#define IRQ_NO_TIM8_UP_TIM13		44
#define IRQ_NO_TIM8_TRG_COM_TIM14	45
#define IRQ_NO_TIM2_TIM8_CC			46
#define IRQ_NO_TIM5 				50
#define IRQ_NO_TIM6_DAC				54
#define IRQ_NO_TIM7 				55


//Generic macros
#define ENABLE 					1
#define DISABLE 				0
#define SET						ENABLE
#define RESET					DISABLE
#define GPIO_PIN_SET			SET
#define GPIO_PIN_RESET			RESET


/******************************************************************************************
 *Bit position definitions of SPI peripheral
 ******************************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	10
#define SPI_CR1_DFF     			 	11
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDIMODE      			15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7


/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8

/******************************************************************************************
 *Bit position definitions of I2C peripheral
 ******************************************************************************************/
/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH  				7
#define I2C_CR1_START 					8
#define I2C_CR1_STOP  				 	9
#define I2C_CR1_ACK 				 	10
#define I2C_CR1_SWRST  				 	15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 	15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15

/******************************************************************************************
 *Bit position definitions of USART peripheral
 ******************************************************************************************/

/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15



/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/*
 * Bit position definitions USART_SR
 */

#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9


/******************************************************************************************
 *Bit position definitions of TIMER peripheral
 ******************************************************************************************/
/*
 * Bit position definitions TIM_CR
 */
#define TIM_CR1_CEN						0
#define TIM_CR1_UDIS						1
#define TIM_CR1_URS						2
#define TIM_CR1_OPM						3
#define TIM_CR1_DIR						4
#define TIM_CR1_CMS						5

/*
 * Bit positions for TIM_CCMR1 (input mode)
 */

#define TIM_CCMR1_CC1S					0
#define TIM_CCMR1_IC1PSC				2
#define TIM_CCMR1_IC1F					4
#define TIM_CCMR1_CC2S					8
#define TIM_CCMR1_IC2PSC				10
#define TIM_CCMR1_IC2F					12

#define TIM_CCMR2_CC3S					0
#define TIM_CCMR2_IC3PSC				2
#define TIM_CCMR2_IC3F					4
#define TIM_CCMR2_CC4S					8
#define TIM_CCMR2_IC4PSC				10
#define TIM_CCMR2_IC4F					12
/*
 * Bit position definitions TIM_SR
 */

#define TIM_SR_UIF						0

/*
 * Bit position definitions TIM_CCER
 */

#define TIM_CCER_CC1E					0
#define TIM_CCER_CC1P					1
#define TIM_CCER_CC1NP					3
#define TIM_CCER_CC2E					4
#define TIM_CCER_CC2P					5
#define TIM_CCER_CC2NP					7
#define TIM_CCER_CC3E					8
#define TIM_CCER_CC3P					9
#define TIM_CCER_CC3NP					11
#define TIM_CCER_CC4E					12
#define TIM_CCER_CC4P					13
#define TIM_CCER_CC4NP					15

/*
 * BIt position of TIM_DIER
 */

#define TIM_DIER_UIE					0
#define TIM_DIER_CCIE1					1
#define TIM_DIER_CCIE2					2
#define TIM_DIER_CCIE3					3
#define TIM_DIER_CCIE4					4


#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_i2c_driver.h"
#include "stm32f407xx_usart_driver.h"
#include "stm32f407xx_rcc_driver.h"
#include "stm32f407xx_timer_driver.h"

#endif /* INC_STM32F407XX_H_ */
