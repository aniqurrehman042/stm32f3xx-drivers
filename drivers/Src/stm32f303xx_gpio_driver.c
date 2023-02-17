/*
 * stm32f303xx_gpio.c
 *
 *  Created on: Nov 28, 2022
 *      Author: Aniqu
 */

#include "stm32f303xx_gpio_driver.h"

/*
 * @fn				- GPIO_PeriClockControl
 *
 * @brief			- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		- pGPIOx: Base address of the GPIO peripheral
 * @param[in]		- EnorDi: ENABLE or DISABLE macro
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_PeriClockControl(GPIO_RegDef_t* pGPIOx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
			GPIOA_PCLK_EN();
		if (pGPIOx == GPIOB)
			GPIOB_PCLK_EN();
		if (pGPIOx == GPIOC)
			GPIOC_PCLK_EN();
		if (pGPIOx == GPIOD)
			GPIOD_PCLK_EN();
		if (pGPIOx == GPIOE)
			GPIOE_PCLK_EN();
		if (pGPIOx == GPIOF)
			GPIOF_PCLK_EN();
	}
	else
	{
		if (pGPIOx == GPIOA)
			GPIOA_PCLK_DI();
		if (pGPIOx == GPIOB)
			GPIOB_PCLK_DI();
		if (pGPIOx == GPIOC)
			GPIOC_PCLK_DI();
		if (pGPIOx == GPIOD)
			GPIOD_PCLK_DI();
		if (pGPIOx == GPIOE)
			GPIOE_PCLK_DI();
		if (pGPIOx == GPIOF)
			GPIOF_PCLK_DI();
	}
}

/*
 * @fn				- GPIO_Init
 *
 * @brief			- Initializes the given GPIO peripheral
 *
 * @param[in]		- pGPIOHandle: Handle of the GPIO peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_Init(GPIO_Handle_t* pGPIOHandle)
{
	uint32_t temp = 0;

	// Configure the mode of the GPIO pin

	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// Non interrupt mode
		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clearing bits
		pGPIOHandle->pGPIOx->MODER |= temp; // Setting bits
	}
	else
	{
		// Interrupt mode
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// Configure the Falling Edge Trigger Selection Register
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			// Reset corresponding Rising Edge Trigger Selection bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// Configure the Rising Edge Trigger Selection Register
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			// Reset corresponding Falling Edge Trigger Selection bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			// Configure both the Rising and Falling Edge Trigger Selection Registers
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// Configure GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = (portcode << (temp2 * 2));

		// Enable EXTI interrupt delivery using Interrupt Mask Register
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;

	// Configure the speed

	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clearing bits
	pGPIOHandle->pGPIOx->OSPEEDR |= temp; // Setting bits

	temp = 0;

	// Configure the pull up/pull down settings

	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clearing bits
	pGPIOHandle->pGPIOx->PUPDR |= temp; // Setting bits

	temp = 0;

	// Configure the output type setting

	temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // Clearing bits
	pGPIOHandle->pGPIOx->OTYPER |= temp; // Setting bits

	temp = 0;

	// Configure the alt functionality

	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2)); // Clearing bits
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2)); // Setting bits
	}

}

/*
 * @fn				- GPIO_DeInit
 *
 * @brief			- De-Initializes the given GPIO peripheral
 *
 * @param[in]		- pGPIOx: Base address of the GPIO peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_DeInit(GPIO_RegDef_t* pGPIOx)
{
	if (pGPIOx == GPIOA)
		GPIOA_REG_RESET();
	if (pGPIOx == GPIOB)
		GPIOB_REG_RESET();
	if (pGPIOx == GPIOC)
		GPIOC_REG_RESET();
	if (pGPIOx == GPIOD)
		GPIOD_REG_RESET();
	if (pGPIOx == GPIOE)
		GPIOE_REG_RESET();
	if (pGPIOx == GPIOF)
		GPIOF_REG_RESET();
}

/*
 * @fn				- GPIO_ReadFromInputPin
 *
 * @brief			- Reads the value from the input pin of the GPIO port
 *
 * @param[in]		- pGPIOx: Base address of the GPIO peripheral
 * @param[in]		- PinNumber: Pin number of the GPIO peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber)
{
	return (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x01);
}

/*
 * @fn				- GPIO_ReadFromInputPort
 *
 * @brief			- Reads the value from the GPIO input port
 *
 * @param[in]		- pGPIOx: Base address of the GPIO peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx)
{
	return (uint16_t)(pGPIOx->IDR);
}

/*
 * @fn				- GPIO_WriteToOutputPin
 *
 * @brief			- Writes a value to the output pin of the GPIO port
 *
 * @param[in]		- pGPIOx: Base address of the GPIO peripheral
 * @param[in]		- PinNumber: Pin number of the GPIO peripheral
 * @param[in]		- Value: Value to be written to the GPIO peripheral's pin
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/*
 * @fn				- GPIO_WriteToOutputPort
 *
 * @brief			- Writes a value to the output GPIO port
 *
 * @param[in]		- pGPIOx: Base address of the GPIO peripheral
 * @param[in]		- Value: Value to be written to the GPIO port
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/*
 * @fn				- GPIO_WriteToOutputPort
 *
 * @brief			- Toggles the value of the GPIO peripheral's pin
 *
 * @param[in]		- pGPIOx: Base address of the GPIO peripheral
 * @param[in]		- PinNumber: Pin number of the GPIO port
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * @fn				- GPIO_IRQInterruptConfig
 *
 * @brief			- Configures the IRQ Interrupt
 *
 * @param[in]		- IRQNumber: IRQ number to be configured
 * @param[in]		- EnorDi: ENABLE or DISABLE macro
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (IRQNumber <= 31)
		{
			// Update ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64)
		{
			// Update ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			// Update ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
	else
	{
		if (IRQNumber <= 31)
		{
			// Update ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64)
		{
			// Update ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}
		else if (IRQNumber >= 64 && IRQNumber < 96)
		{
			// Update ICER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}
}

/*
 * @fn				- GPIO_IRQPriorityConfig
 *
 * @brief			- Configures the IRQ Priority
 *
 * @param[in]		- IRQNumber: IRQ number whose priority is to be configured
 * @param[in]		- IRQPriority: Priority to be assigned to the IRQ number
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// Find IPR register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}

/*
 * @fn				- GPIO_IRQHandling
 *
 * @brief			- Sets the IRQ Handler
 *
 * @param[in]		- PinNumber: PinNumber whose IRQ handler is to be set
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	// Clear the EXTI PR register bit for the pin number
	if (EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR |= (1 << PinNumber);
	}
}
