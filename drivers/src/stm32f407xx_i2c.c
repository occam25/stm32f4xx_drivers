/*
 * stm32f407xx_i2c.c
 *
 *  Created on: Oct 17, 2020
 *      Author: javi
 */

#include <stm32f407xx_i2c.h>


uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint8_t flag)
{
	if((flag == I2C_SR2_MSL) || (flag == I2C_SR2_BUSY) || (flag == I2C_SR2_TRA) ||
			(flag == I2C_SR2_GENCALL) || (flag == I2C_SR2_SMBDEFAULT) || (flag == I2C_SR2_SMBHOST) ||
			(flag == I2C_SR2_DUALF) || (flag == I2C_SR2_PEC)){

	}
//+++
	if(pI2Cx->SR2 & flag)
		return SET;

	return RESET;
}


/******************************************************************************
 * @fn 				- I2C_ClockControl
 *
 * @brief			- This function enables or disables peripheral clock for the given I2C
 *
 * @param[in]		- I2Cx peripheral
 * @param[in]		- ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void I2C_ClockControl(I2C_RegDef_t *pI2Cx, uint8_t status)
{
	if(status == ENABLE){
		if(pI2Cx == I2C1){
			I2C1_PCLK_EN();
		}else if(pI2Cx == I2C2){
			I2C2_PCLK_EN();
		}else if(pI2Cx == I2C3){
			I2C3_PCLK_EN();
		}
	}else if(status == DISABLE){
		if(pI2Cx == I2C1){
			I2C1_PCLK_DI();
		}else if(pI2Cx == I2C2){
			I2C2_PCLK_DI();
		}else if(pI2Cx == I2C3){
			I2C3_PCLK_DI();
		}
	}
}

uint32_t RCC_GetPLLOutputClock(void)
{
	// TODO
	return 0;
}

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t system_clk;
	uint32_t pclk1;
	uint8_t clksrc;

	clksrc = ((RCC->CFGR >> 2) & 0x03);

	/* System clock */
	if(clksrc == 0){
		// HSI
		system_clk = HSI_FREQ;
	}else if(clksrc == 1){
		// HSE
		system_clk = HSE_FREQ;
	}else if(clksrc == 2){
		// PLL
		system_clk = RCC_GetPLLOutputClock();
	}else{

	}

	/* Buses' clocks */
	// AHB
	uint32_t AHB_preescaler_bits = ((RCC->CFGR >> 4) & 0x0f);
	uint8_t AHB_preescaler;

	if(AHB_preescaler_bits < 8){
		AHB_preescaler = 1;
	}else{
		AHB_preescaler = 2^(AHB_preescaler_bits - 8 + 1);
	}

	// APB1
	uint32_t APB1_preescaler_bits = ((RCC->CFGR >> 10) & 0x07);
	uint8_t APB1_preescaler;
	if(APB1_preescaler_bits < 4){
		APB1_preescaler = 1;
	}else{
		APB1_preescaler = 2^(APB1_preescaler_bits - 4 + 1);
	}

	pclk1 = (system_clk / AHB_preescaler) / APB1_preescaler;

	return pclk1;
}

/******************************************************************************
 * @fn 				- I2C_Init
 *
 * @brief			- This function initializes the given I2C peripheral
 *
 * @param[in]		- I2Cx handle containing the I2Cx base address and the I2C configuration
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void I2C_Init(I2C_handle_t *pI2CHandle)
{

	uint32_t tempreg = 0;

	// ACK control bit
	pI2CHandle->pI2Cx->CR1 |= (pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK);


	////////////////////////////////////////////
	/* Disable peripheral */

	/* Speed */
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed > I2C_SCL_SPEED_SM){
		// Fast Mode
		pI2CHandle->pI2Cx->CCR = 0; // reset register
		pI2CHandle->pI2Cx->CCR |= (1 << I2C_CCR_FS); // set fast mode bit

		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2){
			pI2CHandle->pI2Cx->CCR &= ~(1 << I2C_CCR_DUTY); // clear duty bit
			pI2CHandle->pI2Cx->CCR |= (0x0fff & (8000000/(3*pI2CHandle->I2C_Config.I2C_SCLSpeed)));

		}else if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_19_9){
			pI2CHandle->pI2Cx->CCR |= (1 << I2C_CCR_DUTY); // set duty bit
			pI2CHandle->pI2Cx->CCR |= (0x0fff & (8000000/(25*pI2CHandle->I2C_Config.I2C_SCLSpeed)));
		}
	}else{
		// Standard mode
		pI2CHandle->pI2Cx->CCR = 0; // reset register
		pI2CHandle->pI2Cx->CCR &= ~(1 << I2C_CCR_FS); // clear fast mode bit

		pI2CHandle->pI2Cx->CCR |= (0x0fff & (8000000/(2*pI2CHandle->I2C_Config.I2C_SCLSpeed)));
	}

	/* Address */
	if(pI2CHandle->I2C_Config.I2C_DeviceAddress > 0x7f){
		// 10-bit address
		pI2CHandle->pI2Cx->OAR1 = 0x0000; // reset register
		pI2CHandle->pI2Cx->OAR1 |= (1 << I2C_OAR1_ADDMODE); // set 10-bit slave address bit
		pI2CHandle->pI2Cx->OAR1 |= (0x3f & pI2CHandle->I2C_Config.I2C_DeviceAddress);
		pI2CHandle->pI2Cx->OAR1 |= (1 << 14); // kept at 1 by software
	}else{
		// 7-bit address
		pI2CHandle->pI2Cx->OAR1 = 0x0000; // reset register
		pI2CHandle->pI2Cx->OAR1 &= ~(1 << I2C_OAR1_ADDMODE); // clear 10-bit slave address bit
		pI2CHandle->pI2Cx->OAR1 |= (0x80 & pI2CHandle->I2C_Config.I2C_DeviceAddress) << 1;
		pI2CHandle->pI2Cx->OAR1 |= (1 << 14); // kept at 1 by software
	}
	/* ACK */
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE){
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}else if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_DISABLE){
		pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}

	/* Enable peripheral */

}

/******************************************************************************
 * @fn 				- I2C_DeInit
 *
 * @brief			- This function de-initializes the given I2C peripheral
 *
 * @param[in]		- I2Cx handle containing the I2Cx base address
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1){
		I2C1_RST();
	}else if(pI2Cx == I2C2){
		I2C2_RST();
	}else if(pI2Cx == I2C3){
		I2C3_RST();
	}
}

/******************************************************************************
 * @fn 				- I2C_PeripheralControl
 *
 * @brief			- This function enables or disables the I2C peripheral
 *
 * @param[in]		- Base address of the I2C peripheral
 * @param[in]		- ENABLE or DISABLE
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t status)
{
	if(status == ENABLE){
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}else{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

/******************************************************************************
 * @fn 				- I2C_IRQInterruptConfig
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
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t status)
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
 * @fn 				- I2C_IRQPriorityConfig
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
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// Set priority in NVIC_IPRx registers
	uint8_t NVIC_IPRx_idx = IRQNumber / 4;
	uint8_t IRQx_idx = IRQNumber % 4;
	__vo uint32_t *NVIC_IPRx = (uint32_t *)(NVIC_IPR_BASEADDR + 4*NVIC_IPRx_idx);

	*NVIC_IPRx &= ~(0xFF << (IRQx_idx * 8));
	*NVIC_IPRx |= (IRQPriority << (IRQx_idx * 8 + (8 - NVIC_NUM_OF_PR_BITS_IMPLEMENTED)));
}
