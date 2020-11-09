/*
 * stm32f407xx_i2c.c
 *
 *  Created on: Oct 17, 2020
 *      Author: javi
 */

#include <stm32f407xx_i2c.h>
#include <stm32f407xx_rcc.h>

static void I2C_MasterHandleRXNEInterrupt(I2C_handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_handle_t *pI2CHandle);

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
void I2C_GenerateSTOPCondition(I2C_RegDef_t *pI2Cx)
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
 * @param[in]		- I2Cx handle containing the I2Cx base address and the I2C configuration
 *
 * @return			- none
 *
 * @Note			- This flag is cleared after reading the SR1 and SR2 registers in that order
 *
 */
void I2C_ClearADDRFlag(I2C_handle_t *pI2CHandle)
{
	uint32_t dummy_read;

	// check device mode
	if(I2C_IN_MASTER_MODE(pI2CHandle)){
		// device in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
			if(pI2CHandle->RxSize == 1){
				// fist disable ACK
				I2C_ACKingControl(pI2CHandle->pI2Cx, DISABLE);
				// clear the ADDR flag (read SR1, read SR2)
				dummy_read = pI2CHandle->pI2Cx->SR1;
				dummy_read = pI2CHandle->pI2Cx->SR2;
				(void)dummy_read;
			}
		}else{
			// clear the ADDR flag (read SR1, read SR2)
			dummy_read = pI2CHandle->pI2Cx->SR1;
			dummy_read = pI2CHandle->pI2Cx->SR2;
			(void)dummy_read;
		}
	}else{
		// device in slave mode
		// clear the ADDR flag (read SR1, read SR2)
		dummy_read = pI2CHandle->pI2Cx->SR1;
		dummy_read = pI2CHandle->pI2Cx->SR2;
		(void)dummy_read;
	}
}

/******************************************************************************
 * @fn 				- I2C_CloseReceiveData
 *
 * @brief			- This function closes the I2C receive communication and
 * 					  resets all the parameters
 *
 * @param[in]		- I2Cx handle containing the I2Cx base address and the I2C configuration
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void I2C_CloseReceiveData(I2C_handle_t *pI2CHandle)
{
	// Disable ITBUREN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	// Disable ITEVFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	// Reset receive parameters
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->TxRxState = I2C_READY;

	// Re-enable ACKing if needed
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
		I2C_ACKingControl(pI2CHandle->pI2Cx, ENABLE);
}

/******************************************************************************
 * @fn 				- I2C_CloseSendData
 *
 * @brief			- This function closes the I2C send communication and
 * 					  resets all the parameters
 *
 * @param[in]		- I2Cx handle containing the I2Cx base address and the I2C configuration
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void I2C_CloseSendData(I2C_handle_t *pI2CHandle)
{
	// Disable ITBUREN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	// Disable ITEVFEN control bit
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	// Reset receive parameters
	pI2CHandle->TxLen = 0;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxRxState = I2C_READY;

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
 * @param[in]		- Base address of the I2C peripheral
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
 * @param[in]		- Base address of the I2C peripheral
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
 * @param[in]		- Flag to use or not START repetition
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
	I2C_ClearADDRFlag(pI2CHandle);

	// 6. Send the data until len becomes 0
	while(len){
		while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG, pI2CHandle->pI2Cx->SR1));
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		len--;
	}

	// 7. When len becomes zero, wait fot TXE = 1 and BTF = 1 before generating the STOP condition.
	// Note: TXE = 1 and BTF = 1 means that both SR and DR are empty and next transmission should begin.
	// When BTF = 1 SCL will be stretched (pulled to LOW)
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_TXE_FLAG, pI2CHandle->pI2Cx->SR1));
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_BTF_FLAG, pI2CHandle->pI2Cx->SR1));

	// 8. Generate STOP condition and master need not to wait for the completion of STOP condition
	// Note: generating STOP, automatically clears the BTF
	if(start_repetition == I2C_DISABLE_SR)
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
 * @param[in]		- Flag to use or not START repetition
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
		I2C_ClearADDRFlag(pI2CHandle);

		// 3. Wait until RXNE is set
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RXNE_FLAG, pI2CHandle->pI2Cx->SR1));

		// 4. Generate STOP condition
		if(start_repetition == I2C_DISABLE_SR)
			I2C_GenerateSTOPCondition(pI2CHandle->pI2Cx);

		// 5. Read data into buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;

	}else{
		// Procedure to read more than 1 byte from slave
		// 1. Clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		// 2. Read the data until len is zero
		for(uint32_t i = len; i > 0; i--){
			// Wait until RXNE is set
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_RXNE_FLAG, pI2CHandle->pI2Cx->SR1));

			if(i == 2){
				// Last 2 bytes remaining:
				// Disable ACKing
				I2C_ACKingControl(pI2CHandle->pI2Cx, DISABLE);

				// Generate STOP condition
				if(start_repetition == I2C_DISABLE_SR)
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
 * @fn 				- I2C_MasterSendDataIT
 *
 * @brief			- This function sends data as Master through I2C peripheral using interrupts
 *
 * @param[in]		- I2Cx handle containing the I2Cx base address and the I2C configuration
 * @param[in]		- Rx buffer to hold received bytes
 * @param[in]		- Length of Rx buffer
 * @param[in]		- Slave address
 * @param[in]		- Flag to use or not START repetition
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
uint8_t I2C_MasterSendDataIT(I2C_handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t start_repetition)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = len;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddr = slaveAddr;
		pI2CHandle->Sr = start_repetition;

		// Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// Enable ITBUFEN (buffer interrupt enable) Control Bit to trigger interrupts for TXNE and RXE
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		// Enable ITEVFEN (event interrupt enable) Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		// Enable ITERREN (error interrupt enable) Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

/******************************************************************************
 * @fn 				- I2C_MasterReceiveDataIT
 *
 * @brief			- This function receives data as Master through I2C peripheral using interrupts
 *
 * @param[in]		- I2Cx handle containing the I2Cx base address and the I2C configuration
 * @param[in]		- Rx buffer to hold received bytes
 * @param[in]		- Length of Rx buffer
 * @param[in]		- Slave address
 * @param[in]		- Flag to use or not START repetition
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
uint8_t I2C_MasterReceiveDataIT(I2C_handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t start_repetition)
{
	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = len;
		pI2CHandle->RxSize = len; //+++
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->DevAddr = slaveAddr;
		pI2CHandle->Sr = start_repetition;

		// Generate START Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// Enable ITBUFEN (buffer interrupt enable) Control Bit to trigger interrupts for TXNE and RXE
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		// Enable ITEVFEN (event interrupt enable) Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		// Enable ITERREN (error interrupt enable) Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}

	return busystate;
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

void I2C_SlaveCallbackEventsControl(I2C_RegDef_t *pI2Cx, uint8_t status)
{
	if(status == ENABLE){
		// Enable ITBUFEN (buffer interrupt enable) Control Bit to trigger interrupts for TXNE and RXE
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		// Enable ITEVFEN (event interrupt enable) Control Bit
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITEVTEN);

		// Enable ITERREN (error interrupt enable) Control Bit
		pI2Cx->CR2 |= ( 1 << I2C_CR2_ITERREN);
	}else{
		// Disable ITBUFEN (buffer interrupt enable) Control Bit to trigger interrupts for TXNE and RXE
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITBUFEN);

		// Disable ITEVFEN (event interrupt enable) Control Bit
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITEVTEN);

		// Disable ITERREN (error interrupt enable) Control Bit
		pI2Cx->CR2 &= ~( 1 << I2C_CR2_ITERREN);
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


/******************************************************************************
 * @fn 				- I2C_SlaveSendData
 *
 * @brief			- This function sends one byte through the given I2C peripheral
 *
 * @param[in]		- I2Cx handle containing the I2Cx base address
 * @param[in]		- Byte to send
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data)
{
	pI2Cx->DR = data;
}

/******************************************************************************
 * @fn 				- I2C_SlaveReceiveData
 *
 * @brief			- This function reads one byte from the data register of the given I2C peripheral
 *
 * @param[in]		- I2Cx handle containing the I2Cx base address
 *
 * @return			- uint8_t data byte read from I2C data register
 *
 * @Note			- none
 *
 */
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
	return (uint8_t)pI2Cx->DR;
}

/******************************************************************************
 * @fn 				- I2C_EV_IRQHandling
 *
 * @brief			- This function handles the I2C event interrupts
 *
 * @param[in]		- I2Cx handle containing the I2Cx base address and the I2C configuration
 *
 * @return			- none
 *
 * @Note			- Interrupt handling for both master and slave mode of a device
 *
 */
void I2C_EV_IRQHandling(I2C_handle_t *pI2CHandle)
{
	uint32_t SR_flag, ITBUFEN_flag, ITEVTEN_flag;

	ITEVTEN_flag = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	ITBUFEN_flag = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

	SR_flag = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);

	// 1. Handle for interrupt generated by SB (Start Bit) event
	//    Note: SB flag is only applicable in Master mode (only set in Master mode)
	if(ITEVTEN_flag && SR_flag){
		// SB event: the master successfully created the START condition
		// the address can be sent
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			I2C_WriteAddress(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, I2C_WRITE);
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			I2C_WriteAddress(pI2CHandle->pI2Cx, pI2CHandle->DevAddr, I2C_READ);
		else //+++
			SR_flag = pI2CHandle->pI2Cx->DR;
	}

	// 2. Handle for interrupt generated by ADDR event
	//    Note: When Master mode: Address is sent
	//     		When Slave mode: Address matched with own address
	SR_flag = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	if(ITEVTEN_flag && SR_flag){
		// ADDR event
		// Clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);
	}

	// 3. Handle for interrupt generated by BTF (Byte Transfer Finished) event
	SR_flag = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	if(ITEVTEN_flag && SR_flag){
		// BTF event
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
			if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE)){
				// Both BTF and TXE are set => close the transmission if TxLen is zero
				if(pI2CHandle->TxLen == 0){
					// 1. Generate the STOP condition
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
						I2C_GenerateSTOPCondition(pI2CHandle->pI2Cx);

					// 2. Reset all the member elements of the handle structure.
					I2C_CloseSendData(pI2CHandle);

					// 3. Notify the application about transmission complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
			//+++
			if(pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE)){
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}
	}

	// 4. Handle for interrupt generated by STOPF event
	//	  Note: Stop detection flag is applicable only in slave mode. For master this flag will never be set
	SR_flag = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	if(ITEVTEN_flag && SR_flag){
		// STOPF event
		// This code will not be executed by the master since the STOPF will not be set in master mode

		// Clear the STOPF (1. read SR1 (already done), 2. Write to CR1)
		pI2CHandle->pI2Cx->CR1 |= 0x0000; // dummy write to clear STOPF

		// Notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	// 5. Handle for interrupt generated by TXE event
	SR_flag = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
	if(ITEVTEN_flag && ITBUFEN_flag && SR_flag){
		// Check for device mode
		if(I2C_IN_MASTER_MODE(pI2CHandle)){
			// TXE event
			// If we are in transmission data will be sent
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}else{
			// slave
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)){
				// In transmitter mode
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}

	// 6. Handle for interrupt generated by RXNE event
	SR_flag = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
	if(ITEVTEN_flag && ITBUFEN_flag && SR_flag){
		// RXNE event
		if(I2C_IN_MASTER_MODE(pI2CHandle)){
			// The device is in master mode
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){

				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}else //+++
				SR_flag = pI2CHandle->pI2Cx->DR;
		}else{
			// Slave
			if(!(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))){
				// In receive mode
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}
}

static void I2C_MasterHandleTXEInterrupt(I2C_handle_t *pI2CHandle)
{
	// We have to do the data transmission
	if(pI2CHandle->TxLen > 0){
		// 1. Load data into DR
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
		// 2. Increment buffer pointer
		pI2CHandle->pTxBuffer++;
		// 3. Decrement buffer length
		pI2CHandle->TxLen--;
	}
}

static void I2C_MasterHandleRXNEInterrupt(I2C_handle_t *pI2CHandle)
{
	if(pI2CHandle->RxSize == 1){
		// Generate STOP condition
		if(pI2CHandle->Sr == I2C_DISABLE_SR)
			I2C_GenerateSTOPCondition(pI2CHandle->pI2Cx);
		// Read data into buffer
		*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
		// Decrement RxLen
		pI2CHandle->RxLen--;


	}else if(pI2CHandle->RxSize > 1){
		if(pI2CHandle->RxLen == 2){
			// Disable ACKing
			//+++I2C_ACKingControl(pI2CHandle->pI2Cx, DISABLE); //+++ comentado porque no hacia ACK
			// Generate STOP condition
			//					if(pI2CHandle->Sr == I2C_DISABLE_SR)
			//						I2C_GenerateSTOPCondition(pI2CHandle->pI2Cx);
		}
		// Read data register into buffer
		*(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->DR;
		// Increment buffer pointer
		pI2CHandle->pRxBuffer++;
		// Decrement RxLen
		pI2CHandle->RxLen--;
	}

	if(pI2CHandle->RxLen == 0){
		// close the I2c data reception and notify the application
		// just in case, we clear RXNE flag
		uint32_t dummy = pI2CHandle->pI2Cx->DR;
		(void)dummy;
		// 1. generate STOP condition

		// 2. Close the I2C Rx
		I2C_CloseReceiveData(pI2CHandle);

		// 3. Notify the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);

		// Re-enable ACKing
	}
}
/******************************************************************************
 * @fn 				- I2C_ER_IRQHandling
 *
 * @brief			- This function handles the I2C error interrupts
 *
 * @param[in]		- I2Cx handle containing the I2Cx base address and the I2C configuration
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void I2C_ER_IRQHandling(I2C_handle_t *pI2CHandle)
{
	uint32_t ITERREN_flag, ERR_flag;

	// Check ITERREN flag status
	ITERREN_flag = (pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITERREN));

	/* BUS ERROR */
	ERR_flag = (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BERR));
	if(ITERREN_flag && ERR_flag){
		// Clear flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);

		// Inform application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERR_BERR);
	}

	/* ARLO (Arbitration lost) ERROR (only in Master mode) */
	ERR_flag = (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ARLO));
	if(ITERREN_flag && ERR_flag){
		// Clear flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);

		// Inform application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERR_ARLO);
	}

	/* AF (Acknowledge Failure) ERROR */
	ERR_flag = (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_AF));
	if(ITERREN_flag && ERR_flag){
		// Clear flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

		// Inform application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERR_AF);
	}

	/* Overrun/Underrun ERROR */
	ERR_flag = (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_OVR));
	if(ITERREN_flag && ERR_flag){
		// Clear flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);

		// Inform application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERR_OVR);
	}

	/* Timeout ERROR */
	ERR_flag = (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TIMEOUT));
	if(ITERREN_flag && ERR_flag){
		// Clear flag
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

		// Inform application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERR_TOUT);
	}
}

__weak void I2C_ApplicationEventCallback(I2C_handle_t *pI2CHandle, I2C_events_t AppEv)
{
	// This function has to be implemented by the application
}






















