/*
 * stm32f303xx.h
 *
 *  Created on: Nov 27, 2022
 *      Author: Aniqu
 */

#ifndef INC_STM32F303XX_H_
#define INC_STM32F303XX_H_

#include <stdint.h>

// ARM Cortex Mx NVIC ISERx register addresses

#define NVIC_ISER0					((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1					((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2					((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3					((volatile uint32_t*)0xE000E10C)

// ARM Cortex Mx NVIC ICERx register addresses

#define NVIC_ICER0					((volatile uint32_t*)0xE000E180)
#define NVIC_ICER1					((volatile uint32_t*)0xE000E184)
#define NVIC_ICER2					((volatile uint32_t*)0xE000E188)
#define NVIC_ICER3					((volatile uint32_t*)0xE000E18C)

// ARM Cortex Mx Interrupt Priority register addresses

#define NVIC_PR_BASE_ADDR			((volatile uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED		4

// Memories Base Addresses

#define FLASH_BASEADDR				0x08000000U
#define SRAM_BASEADDR				0x20000000U
#define ROM_BASEADDR				0x1FFFD800U

// Buses Base Addresses

#define PERIPH_BASEADDR				0x40000000U
#define APB1PERIPH_BASEADDR			PERIPH_BASE
#define APB2PERIPH_BASEADDR 		0x40010000U
#define AHB1PERIPH_BASEADDR 		0x40020000U
#define AHB2PERIPH_BASEADDR 		0x48000000U
#define AHB3PERIPH_BASEADDR 		0x50000000U

// Peripherals Base Addresses From APB1 Bus

#define SPI2_BASEADDR				(APB1PERIPH_BASEADDR + 0x3800U)
#define SPI3_BASEADDR				(APB1PERIPH_BASEADDR + 0x3C00U)
#define USART2_BASEADDR				(APB1PERIPH_BASEADDR + 0x4400U)
#define USART3_BASEADDR				(APB1PERIPH_BASEADDR + 0x4800U)
#define UART4_BASEADDR				(APB1PERIPH_BASEADDR + 0x4C00U)
#define UART5_BASEADDR				(APB1PERIPH_BASEADDR + 0x5000U)
#define I2C1_BASEADDR				(APB1PERIPH_BASEADDR + 0x5400U)
#define I2C2_BASEADDR				(APB1PERIPH_BASEADDR + 0x5800U)

// Peripherals Base Addresses From APB2 Bus

#define SYSCFG_BASEADDR				(APB2PERIPH_BASEADDR + 0x0000U)
#define EXTI_BASEADDR				(APB2PERIPH_BASEADDR + 0x0400U)
#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3000U)
#define USART1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3800U)

// Peripherals Base Addresses From AHB1 Bus

#define RCC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1000U)

// Peripherals Base Addresses From AHB2 Bus

#define GPIOA_BASEADDR				(AHB2PERIPH_BASEADDR + 0x0000U)
#define GPIOB_BASEADDR				(AHB2PERIPH_BASEADDR + 0x0400U)
#define GPIOC_BASEADDR				(AHB2PERIPH_BASEADDR + 0x0800U)
#define GPIOD_BASEADDR				(AHB2PERIPH_BASEADDR + 0x0C00U)
#define GPIOE_BASEADDR				(AHB2PERIPH_BASEADDR + 0x1000U)
#define GPIOF_BASEADDR				(AHB2PERIPH_BASEADDR + 0x1400U)

// Peripheral Register Definition Structures

typedef struct {
	volatile uint32_t MODER;		// Port Mode Register
	volatile uint32_t OTYPER;		// Output Type Register
	volatile uint32_t OSPEEDR;		// Output Speed Register
	volatile uint32_t PUPDR;		// Pull Up Pull Down Register
	volatile uint32_t IDR;			// Input Data Register
	volatile uint32_t ODR;			// Output Data Register
	volatile uint32_t BSRR;			// Port Bit Set Reset Register
	volatile uint32_t LCKR;			// Port Configuration Lock Register
	volatile uint32_t AFR[2];		// Alternate Function Registers (AFR[0]: Low, AFR[1]: High)
	volatile uint32_t BRR;			// Port Bit Reset Register
} GPIO_RegDef_t;

typedef struct {
	volatile uint32_t CR;			// Clock Control Register
	volatile uint32_t CFGR;			// Clock Configuration Register
	volatile uint32_t CIR;			// Clock Interrupt Register
	volatile uint32_t APB2RSTR;		// APB2 Peripheral Reset Register
	volatile uint32_t APB1RSTR;		// APB1 Peripheral Reset Register
	volatile uint32_t AHBENR;		// AHB Peripheral Clock Enable Register
	volatile uint32_t APB2ENR;		// APB2 Peripheral Clock Enable Register
	volatile uint32_t APB1ENR;		// APB1 Peripheral Clock Enable Register
	volatile uint32_t BDCR;			// RTC Domain Control Register
	volatile uint32_t CSR;			// Control/Status Register
	volatile uint32_t AHBRSTR;		// AHB Peripheral Reset Register
	volatile uint32_t CFGR2;		// Clock Configuration Register 2
	volatile uint32_t CFGR3;		// Clock Configuration Register 3
} RCC_RefDef_t;

typedef struct {
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;
} EXTI_RefDef_t;

typedef struct {
	volatile uint32_t 	MEMRMP;
	volatile uint32_t 	PMC;
	volatile uint32_t 	EXTICR[4];
	uint32_t 			RESERVED1[2];
	volatile uint32_t 	CMPCR;
	uint32_t 			RESERVED2[2];
	volatile uint32_t 	CFGR;
} SYSCFG_RefDef_t;

// Peripheral Definitions

#define GPIOA						((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB						((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC						((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD						((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE						((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF						((GPIO_RegDef_t*)GPIOF_BASEADDR)

#define RCC							((RCC_RefDef_t*)RCC_BASEADDR)

#define EXTI						((EXTI_RefDef_t*)EXTI_BASEADDR)

#define SYSCFG						((SYSCFG_RefDef_t*)SYSCFG_BASEADDR)

// Clock Enable Macros for GPIOx Peripherals

#define GPIOA_PCLK_EN() 			(RCC->AHBENR |= (1 << 17))
#define GPIOB_PCLK_EN() 			(RCC->AHBENR |= (1 << 18))
#define GPIOC_PCLK_EN() 			(RCC->AHBENR |= (1 << 19))
#define GPIOD_PCLK_EN() 			(RCC->AHBENR |= (1 << 20))
#define GPIOE_PCLK_EN() 			(RCC->AHBENR |= (1 << 21))
#define GPIOF_PCLK_EN() 			(RCC->AHBENR |= (1 << 22))

// Clock Enable Macros for I2Cx Peripherals

#define I2C1_PCLK_EN()				(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()				(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()				(RCC->APB1ENR |= (1 << 30))

// Clock Enable Macros for SPIx Peripherals

#define SPI1_PCLK_EN()				(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()				(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()				(RCC->APB1ENR |= (1 << 15))

// Clock Enable Macros for USARTx Peripherals

#define USART1_PCLK_EN()			(RCC->APB2ENR |= (1 << 14))
#define USART2_PCLK_EN()			(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()			(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()				(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()				(RCC->APB1ENR |= (1 << 20))

// Clock Enable Macros for SYSCFG Peripheral

#define SYSCFG_PCLK_EN()			(RCC->APB2ENR |= (1 << 0))

// Clock Disable Macros for GPIOx Peripherals

#define GPIOA_PCLK_DI() 			(RCC->AHBENR &= ~(1 << 17))
#define GPIOB_PCLK_DI() 			(RCC->AHBENR &= ~(1 << 18))
#define GPIOC_PCLK_DI() 			(RCC->AHBENR &= ~(1 << 19))
#define GPIOD_PCLK_DI() 			(RCC->AHBENR &= ~(1 << 20))
#define GPIOE_PCLK_DI() 			(RCC->AHBENR &= ~(1 << 21))
#define GPIOF_PCLK_DI() 			(RCC->AHBENR &= ~(1 << 22))

// Clock Disable Macros for I2Cx Peripherals

#define I2C1_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 30))

// Clock Disable Macros for SPIx Peripherals

#define SPI1_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 15))

// Clock Disable Macros for USARTx Peripherals

#define USART1_PCLK_DI()			(RCC->APB2ENR &= ~(1 << 14))
#define USART2_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()			(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 20))

// Clock Disable Macros for SYSCFG Peripheral

#define SYSCFG_PCLK_DI()			(RCC->APB2ENR |= ~(1 << 0))

// Macros to reset GPIOx peripherals

#define GPIOA_REG_RESET()			do { (RCC->AHBRSTR |= (1 << 17)); (RCC->AHBRSTR &= ~(1 << 17)); } while(0)
#define GPIOB_REG_RESET()			do { (RCC->AHBRSTR |= (1 << 18)); (RCC->AHBRSTR &= ~(1 << 18)); } while(0)
#define GPIOC_REG_RESET()			do { (RCC->AHBRSTR |= (1 << 19)); (RCC->AHBRSTR &= ~(1 << 19)); } while(0)
#define GPIOD_REG_RESET()			do { (RCC->AHBRSTR |= (1 << 20)); (RCC->AHBRSTR &= ~(1 << 20)); } while(0)
#define GPIOE_REG_RESET()			do { (RCC->AHBRSTR |= (1 << 21)); (RCC->AHBRSTR &= ~(1 << 21)); } while(0)
#define GPIOF_REG_RESET()			do { (RCC->AHBRSTR |= (1 << 22)); (RCC->AHBRSTR &= ~(1 << 22)); } while(0)

#define GPIO_BASEADDR_TO_CODE(x)	( (x == GPIOA) ? 0 : \
									(x == GPIOB) ? 1 : \
									(x == GPIOC) ? 2 : \
									(x == GPIOD) ? 3 : \
									(x == GPIOE) ? 4 : \
									(x == GPIOF) ? 5 : 0 )

// IRQ Number for EXTI lines

#define IRQ_NO_EXTI0				6
#define IRQ_NO_EXTI1				7
#define IRQ_NO_EXTI2				8
#define IRQ_NO_EXTI3				9
#define IRQ_NO_EXTI4				10
#define IRQ_NO_EXTI9_1				23
#define IRQ_NO_EXTI15_10			40

// Generic Macros

#define ENABLE 						1
#define DISABLE 					0
#define SET 						ENABLE
#define RESET 						DISABLE
#define GPIO_PIN_SET				SET
#define GPIO_PIN_RESET				RESET

#endif /* INC_STM32F303XX_H_ */
