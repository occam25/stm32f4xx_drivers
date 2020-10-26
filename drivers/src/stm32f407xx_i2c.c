/*
 * stm32f407xx_i2c.c
 *
 *  Created on: Oct 17, 2020
 *      Author: javi
 */

#include <stm32f407xx_i2c.h>


/******************************************************************************
 * @fn 				- I2C_ACKingControl
 *
 * @brief			- This function controls the ACKing in the I2C peripheral
 *
 * @param[in]		- Base address of the I2C peripheral
 * @param[in]		- ACK status (ENABLE or DISABLE)
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
static void I2C_ACKingControl(I2C_RegDef_t *pI2Cx, uint8_t status)
{
	if(status == ENABLE)
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	else
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
}

/******************************************************************************
 * @fn 				- I2C_GenerateStartCondition
 *
 * @brief			- This function generates a start condition in the I2C peripheral
 *
 * @param[in]		- Base address of the I2C peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

/******************************************************************************
 * @fn 				- I2C_GenerateSTOPCondition
 *
 * @brief			- This function generates a stop condition in the I2C peripheral
 *
 * @param[in]		- Base address of the I2C peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
static void I2C_GenerateSTOPCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

/******************************************************************************
 * @fn 				- I2C_WriteAddress
 *
 * @brief			- This function sets the I2C address
 *
 * @param[in]		- Base address of the I2C peripheral
 * @param[in]		- Slave I2C address
 * @param[in]		- R/W mode
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
static void I2C_WriteAddress(I2C_RegDef_t *pI2Cx, uint8_t slaveAddr, uint8_t rw_mode)
{
	// Write 7-bit address + r/w bit
	if(rw_mode == I2C_WRITE)
		pI2Cx->DR = (slaveAddr << 1) & 0xfe;
	else
		pI2Cx->DR = (slaveAddr << 1) | 0x01;
}

/******************************************************************************
 * @fn 				- I2C_ClearADDRFlag
 *
 * @brief			- This function clears the ADDR flag in SR1 status register
 *
 * @param[in]		- Base address of the I2C peripheral
 *
 * @return			- none
 *
 * @Note			- This flag is cleared after reading the SR1 and SR2 registers in that order
 *
 */
void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummy;
	dummy = pI2Cx->SR1;
	dummy = pI2Cx->SR2;
	(void) dummy;
}

/******************************************************************************
 * @fn 				- I2C_GetFlagStatus
 *
 * @brief			- This function gets the status of an I2C flag
 *
 * @param[in]		- Base address of the I2C peripheral
 * @param[in]		- Flag to check
 * @param[in]		- Status register that contains the flag (pI2Cx->SR1 or pI2Cx->SR2)
 *
 * @return			- Flag status
 *
 * @Note			- none
 *
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint8_t flag, uint32_t SR)
{
	if(SR == pI2Cx->SR1){
		// SR1 register
		if(pI2Cx->SR1 & flag)
			return SET;
	}else if(SR == pI2Cx->SR2){
		// SR2 register
		if(pI2Cx->SR2 & flag)
			return SET;
	}

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
	uint32_t pclk1;

	/* Enable peripheral clock */
	I2C_ClockControl(pI2CHandle->pI2Cx, ENABLE);

	pclk1 = RCC_GetPCLK1Value();

	/* I2C_CR1 */
	// ACK control bit in CR1
	/* Note: ACK bit in CR1 register will not be set if the peripheral is disabled (PE = 0)
	 * but it is ok because in the enabling API the ACKing configuration will be set again */
	tempreg |= (pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK);
	pI2CHandle->pI2Cx->CR1 = tempreg;

	/* I2C_CR2 */
	// FREQ in CR2
	tempreg = pclk1/1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3f);

	/* I2C_OAR1 */
	// Address
	tempreg = 0;
	tempreg |= (1 << 14); // This bit must be kept at 1 by software
	if(pI2CHandle->I2C_Config.I2C_DeviceAddress > 0x7f){
		// 10-bit address
		tempreg |= (1 << I2C_OAR1_ADDMODE); // set 10-bit slave address bit
		tempreg |= (0x3f & pI2CHandle->I2C_Config.I2C_DeviceAddress);
	}else{
		// 7-bit address
		tempreg &= ~(1 << I2C_OAR1_ADDMODE); // clear 10-bit slave address bit
		tempreg |= (0x7f & pI2CHandle->I2C_Config.I2C_DeviceAddress) << 1;
	}
	pI2CHandle->pI2Cx->OAR1 = tempreg;
	/* I2C_OAR2 */
	// Dual address not supported

	/* I2C_CCR */
	tempreg = 0;
	/* Speed */
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed > I2C_SCL_SPEED_SM){
		// Fast Mode
		tempreg |= (1 << I2C_CCR_FS); // set fast mode bit
		tempreg |= pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY;

		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2){
			tempreg |= ((pclk1/(3*pI2CHandle->I2C_Config.I2C_SCLSpeed)) & 0x0fff);

		}else if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_19_9){
			tempreg |= ((pclk1/(25*pI2CHandle->I2C_Config.I2C_SCLSpeed)) & 0x0fff);
		}
	}else{
		// Standard mode
		tempreg &= ~(1 << I2C_CCR_FS); // clear fast mode bit
		tempreg |= ((pclk1/(2*pI2CHandle->I2C_Config.I2C_SCLSpeed)) & 0x0fff);
	}
	pI2CHandle->pI2Cx->CCR = tempreg;


	/* TRISE configuration */
	tempreg = 0;
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed > I2C_SCL_SPEED_SM){
		// Fast Mode
		tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;
	}else{
		// Standard mode
		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = tempreg & 0x3f;
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
 * @fn 				- I2C_MasterSendData
 *
 * @brief			- This function sends data as Master through I2C peripheral
 *
 * @param[in]		- I2Cx handle containing the I2Cx base address and the I2C configuration
 * @param[in]		- Tx buffer with the data to send
 * @param[in]		- Length of Tx buffer
 * @param[in]		- Slave address
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void I2C_MasterSendData(I2C_handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t start_repetition)
{
	// 1. Generate START
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. Confirm that start generation is completed by checking the SB flag in the SR1 register
	//    Note: until SB is cleared, SCL will be stretched (pulled to LOW)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SB_FLAG, pI2CHandle->pI2Cx->SR1));

	// 3. Send the address of the slave with r/nw bit set to w (0). 8 bits in total
	I2C_WriteAddress(pI2CHandle->pI2Cx, slaveAddr, I2C_WRITE);

	// 4. Confirm that address phase is completed by checking the ADDR flag in the SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR_FLAG, pI2CHandle->pI2Cx->SR1));

	// 5. Clear the ADDR flag according to its software sequences
	// Note: Until ADDR is cleared SCL will be stretched (pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	// 6. Send the data until len becomes 0
	while(len){
		while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG, pI2CHandle->pI2Cx->SR1));
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		len--;
	}

	// 7. When len becomes zero, wait fot TXE = 1 and BTF = 1 before generating tue STOP condition.
	// Note: TXE = 1 and BTF = 1 means that both SR and DR are empty and next transmission should begin.
	// When BTF = 1 SCL will be stretched (pulled to LOW)
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG, pI2CHandle->pI2Cx->SR1));
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_BTF_FLAG, pI2CHandle->pI2Cx->SR1));

	// 8. Generate STOP condition and master need not to wait for the completion of STOP condition
	// Note: generating STOP, automatically clears the BTF
	if(start_repetition == I2C_NO_SR)
		I2C_GenerateSTOPCondition(pI2CHandle->pI2Cx);
}

/******************************************************************************
 * @fn 				- I2C_MasterReceiveData
 *
 * @brief			- This function receives data as Master through I2C peripheral
 *
 * @param[in]		- I2Cx handle containing the I2Cx base address and the I2C configuration
 * @param[in]		- Rx buffer to hold received bytes
 * @param[in]		- Length of Rx buffer
 * @param[in]		- Slave address
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void I2C_MasterReceiveData(I2C_handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t start_repetition)
{
	// TODO
	if(len == 0)
		return;

	// 1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. Confirm that start generation is completed by checking the SB flag in SR1
	//    Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SB_FLAG, pI2CHandle->pI2Cx->SR1));

	// 3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits)
	I2C_WriteAddress(pI2CHandle->pI2Cx, slaveAddr, I2C_READ);

	// 4. Wait until address phase is completed by checking the ADDR flag in SR1
	while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_ADDR_FLAG, pI2CHandle->pI2Cx->SR1));

	if(len == 1){
		// Procedure to read only 1 byte from slave
		// 1. Disable ACKing
		I2C_ACKingControl(pI2CHandle->pI2Cx, DISABLE);

		// 2. Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		// 3. Wait until RXNE is set
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RXNE_FLAG, pI2CHandle->pI2Cx->SR1));

		// 4. Generate STOP condition
		if(start_repetition == I2C_NO_SR)
			I2C_GenerateSTOPCondition(pI2CHandle->pI2Cx);

		// 5. Read data into buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;

	}else{
		// Procedure to read more than 1 byte from slave
		// 1. Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

		// 2. Read the data until len is zero
		for(uint32_t i = len; i > 0; i--){
			// Wait until RXNE is set
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RXNE_FLAG, pI2CHandle->pI2Cx->SR1));

			if(i == 2){
				// Last 2 bytes remaining:
				// Disable ACKing
				I2C_ACKingControl(pI2CHandle->pI2Cx, DISABLE);

				// Generate STOP condition
				if(start_repetition == I2C_NO_SR)
					I2C_GenerateSTOPCondition(pI2CHandle->pI2Cx);
			}

			// Read data register into buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			// Increment the buffer address
			pRxBuffer++;
		}
	}

	// re-enable ACKing if configured
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
		I2C_ACKingControl(pI2CHandle->pI2Cx, ENABLE);
}

/******************************************************************************
 * @fn 				- I2C_PeripheralControl
 *
 * @brief			- This function enables or disables the I2C peripheral
 *
 * @param[in]		- I2Cx handle containing the I2Cx base address and the I2C configuration
 * @param[in]		- ENABLE or DISABLE
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void I2C_PeripheralControl(I2C_handle_t *pI2CHandle, uint8_t status)
{
	if(status == ENABLE){
		pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_PE);
		/* Note: if the ACKing was enabled with the peripheral disabled (PE = 0)
		 * the ACK bit in CR1 was not set properly, so do it here again */
		if(pI2CHandle->I2C_Config.I2C_ACKControl == ENABLE)
			I2C_ACKingControl(pI2CHandle->pI2Cx, ENABLE);
	}else{
		pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
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
