/*
 * stm32f407xx_gpio.c
 *
 *  Created on: Jun 19, 2020
 *      Author: javi
 */

#include "stm32f407xx_gpio.h"


/*
 * Peripheral clock control
 */
/******************************************************************************
 * @fn 				- GPIO_ClockControl
 *
 * @brief			- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		- GPIO port
 * @param[in]		- ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_ClockControl(GPIO_port_t GPIOx_port, uint8_t status)
{
	if(status == ENABLE)
		GPIOx_PCLK_EN(GPIOx_port);
	else
		GPIOx_PCLK_DI(GPIOx_port);
}

/******************************************************************************
 * @fn 				- GPIO_Init
 *
 * @brief			- This function initializes the given GPIO port
 *
 * @param[in]		- GPIO handle containing the GPIO base address and the pin configuration
 *
 * @return			- none
 *
 * @Note			- Steps for GPIO interrupt configuration:
 * 						1. Pin must be in input mode
 * 						2. Configure the edge trigger in EXTI registers
 * 						3. Enable interrupt delivery from peripheral to processor (unmask in EXTI registers)
 * 						4. Identify the IRQ number on which the processor accepts the int. for that pin.
 * 						5. Configure the IRQ priority for the identified IRQ number (processor side, NVIC)
 * 						6. Enable interrupt reception on that IRQ number (processor side, NVIC)
 * 						7. Implement IRQ handler
 *
 */
void GPIO_Init(GPIO_handle_t *pGPIOHandle)
{

	// 0. Set the GPIO port number
	pGPIOHandle->GPIOx_port = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

	// Enable the peripheral clock
	GPIO_ClockControl(pGPIOHandle->GPIOx_port, ENABLE);

	// 1. Configure mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		// Non-interrupt mode
		pGPIOHandle->pGPIOx->MODER &= ~(0x03 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	}else{
		// Interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			// 0. Configure as input
			pGPIOHandle->pGPIOx->MODER &= ~(0x03 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			// 1. Configure FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			// 1. Configure RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT){
			// 1. Configure both FTSR and RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		// 2. configure the GPIO port selection in SYSCFG_EXTICR
		// Enable SYSCFG peripheral clock
		SYSCFG_PCLK_EN();
		uint8_t EXTICR_idx = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t PIN_idx = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;

		SYSCFG->EXTICR[EXTICR_idx] &= ~(0x0F << (4 * PIN_idx));
		SYSCFG->EXTICR[EXTICR_idx] |= (pGPIOHandle->GPIOx_port << (4 * PIN_idx));

		// 3. Enable the EXTI interrupt delivery (unmask) using IMR
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;


	}
	// 2. Configure speed
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x03 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

	// 3. Configure PU/PD
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x03 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

	// 4. Configure output type (only if in output mode)
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_OUT){
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType == GPIO_OP_TYPE_PP)
			pGPIOHandle->pGPIOx->OTYPER &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType == GPIO_OP_TYPE_OD)
			pGPIOHandle->pGPIOx->OTYPER |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	// 5. Configure Alternate Function
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0x0F << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}

}
/******************************************************************************
 * @fn 				- GPIO_DeInit
 *
 * @brief			- This function de-initializes the given GPIO port
 *
 * @param[in]		- Base address of the GPIO peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_DeInit(GPIO_port_t GPIOx_portx)
{
	GPIOx_RST(GPIOx_portx);
}


/******************************************************************************
 * @fn 				- GPIO_ReadFromInputPin
 *
 * @brief			- This function reads the pin state of the given GPIO port
 *
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		- Pin to read
 *
 * @return			- uint8_t containing the pin state (0 or 1)
 *
 * @Note			- none
 *
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	return (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x01);
}
/******************************************************************************
 * @fn 				- GPIO_ReadFromInputPort
 *
 * @brief			- This function reads the given GPIO
 *
 * @param[in]		- Base address of the GPIO peripheral
 *
 * @return			- uint16_t containing the port state
 *
 * @Note			- none
 *
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	return (uint16_t)(pGPIOx->IDR & 0xFFFF);
}
/******************************************************************************
 * @fn 				- GPIO_WriteFromInputPin
 *
 * @brief			- This function writes the pin of the given GPIO port
 *
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		- Pin to write
 * @param[in]		- Value to write
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
	if(value)
		pGPIOx->ODR |= (1 << PinNumber);
	else
		pGPIOx->ODR = ~(1 << PinNumber);
}
/******************************************************************************
 * @fn 				- GPIO_WriteFromInputPort
 *
 * @brief			- This function writes the given GPIO
 *
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		- Value to write
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = (uint32_t)value;
}
/******************************************************************************
 * @fn 				- GPIO_ToggleOutputPin
 *
 * @brief			- This function toggles the pin of the given GPIO port
 *
 * @param[in]		- Base address of the GPIO peripheral
 * @param[in]		- Pin to write
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/******************************************************************************
 * @fn 				- GPIO_IRQInterruptConfig
 *
 * @brief			- This function configures the IRQ Interrupt
 *
 * @param[in]		- Number of the IRQ to configure
 * @param[in]		- ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t status)
{
	if(status == ENABLE){
		if(IRQNumber < 32){
			// NVIC_ISER0  (0 - 31)
			NVIC_ISER0 |= (1 << IRQNumber);
		}else if(IRQNumber < 64){
			// NVIC_ISER1  (32 - 63)
			NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}else if(IRQNumber < 96){
			// NVIC_ISER2  (64 - 95)
			NVIC_ISER2 |= (1 << (IRQNumber % 64));
		}
	}else if(status == DISABLE){
		if(IRQNumber < 32){
			// NVIC_ICER0  (0 - 31)
			NVIC_ICER0 |= (1 << IRQNumber);
		}else if(IRQNumber < 64){
			// NVIC_ICER1  (32 - 63)
			NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}else if(IRQNumber < 96){
			// NVIC_ICER2  (64 - 95)
			NVIC_ICER2 |= (1 << (IRQNumber % 64));
		}
	}



}

/******************************************************************************
 * @fn 				- GPIO_IRQPriorityConfig
 *
 * @brief			- This function configures the IRQ priority
 *
 * @param[in]		- Number of the IRQ to configure
 * @param[in]		- IRQ priority
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// Set priority in NVIC_IPRx registers
	uint8_t NVIC_IPRx_idx = IRQNumber / 4;
	uint8_t IRQx_idx = IRQNumber % 4;
	__vo uint32_t *NVIC_IPRx = (uint32_t *)(NVIC_IPR_BASEADDR + 4*NVIC_IPRx_idx);

	*NVIC_IPRx &= ~(0xFF << (IRQx_idx * 8));
	*NVIC_IPRx |= (IRQPriority << (IRQx_idx * 8 + (8 - NVIC_NUM_OF_PR_BITS_IMPLEMENTED)));
}

/******************************************************************************
 * @fn 				- GPIO_IRQHandling
 *
 * @brief			- This function handles the GPIO IQR
 *
 * @param[in]		- Pin number that triggered the interrupt
 *
 * @return			- none
 *
 * @Note			- The ISR must be implemented by the application and call this function
 *
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{
	// Clear the EXTI PR register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber)){
		// Clear flag
		EXTI->PR |= (1 << PinNumber);
	}
}




