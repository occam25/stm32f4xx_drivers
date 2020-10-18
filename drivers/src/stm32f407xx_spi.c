

#include "stm32f407xx_spi.h"


static void spi_txe_interrupt_handle(SPI_handle_t *pSPIHandle);
static void spi_rxne_interrupt_handle(SPI_handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_handle_t *pSPIHandle);


uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t flag)
{
	if(pSPIx->SR & flag)
		return SET;

	return RESET;
}


/******************************************************************************
 * @fn 				- SPI_ClockControl
 *
 * @brief			- This function enables or disables peripheral clock for the given SPI
 *
 * @param[in]		- SPIx peripheral
 * @param[in]		- ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void SPI_ClockControl(SPI_RegDef_t *pSPIx, uint8_t status)
{
	if(status == ENABLE){
		if(pSPIx == SPI1){
			SPI1_PCLK_EN();
		}else if(pSPIx == SPI2){
			SPI2_PCLK_EN();
		}else if(pSPIx == SPI3){
			SPI3_PCLK_EN();
		}
	}else if(status == DISABLE){
		if(pSPIx == SPI1){
			SPI1_PCLK_DI();
		}else if(pSPIx == SPI2){
			SPI2_PCLK_DI();
		}else if(pSPIx == SPI3){
			SPI3_PCLK_DI();
		}
	}
}

/******************************************************************************
 * @fn 				- SPI_Init
 *
 * @brief			- This function initializes the given SPI peripheral
 *
 * @param[in]		- SPIx handle containing the SPIx base address and the SPI configuration
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void SPI_Init(SPI_handle_t *pSPIHandle)
{

	// Enable peripheral clock
	SPI_ClockControl(pSPIHandle->pSPIx, ENABLE);

	// Configure SPI_CR1 register
	uint32_t tempreg = 0;

	// 1. configure the device mode
	tempreg |= pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR;

	// 2. configure the bus config
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		// BIDI mode should be cleared
		tempreg &= ~(1 << 15);
	}else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		// BIDI mode should be set
		tempreg |= (1 << 15);
	}else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		// BIDI mode should be cleared
		tempreg &= ~(1 << 15);
		// RXONLY bit must be set
		tempreg |= (1 << 10);
	}
	// 3. Configure the SPI serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR;

	// 4. Configure the DFF
	tempreg |= pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF;

	// 5. Configure the CPOL
	tempreg |= pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL;

	// 6. Configure the CPHA
	tempreg |= pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA;

	// 7. Configure the SSM
	tempreg |= pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM;
	if(pSPIHandle->SPI_Config.SPI_DeviceMode == SPI_DEVICE_MODE_MASTER && pSPIHandle->SPI_Config.SPI_SSM){
		// If SSN (Software Slave Management) is enabled and we are in Master mode, set SSI bit to
		// tie NSS pin to +Vcc internaly. Otherwise a MODF error will occur
		tempreg |= (1 << SPI_CR1_SSI);
	}

	pSPIHandle->pSPIx->CR1 = tempreg; // reset value for CR1 is 0x00000000, so the bits that were not configured will be at reset state so we can use the assing operator
}

/******************************************************************************
 * @fn 				- SPI_DeInit
 *
 * @brief			- This function de-initializes the given SPI peripheral
 *
 * @param[in]		- SPIx handle containing the SPIx base address
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1){
		SPI1_RST();
	}else if(pSPIx == SPI2){
		SPI2_RST();
	}else if(pSPIx == SPI3){
		SPI3_RST();
	}
}

/******************************************************************************
 * @fn 				- SPI_SendData
 *
 * @brief			- This function sends data through SPI peripheral
 *
 * @param[in]		- Base address of the SPI peripheral
 * @param[in]		- Pointer to the transmission buffer
 * @param[in]		- Number of transmission units
 *
 * @return			- none
 *
 * @Note			- This is a blocking call
 *
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{

	while(len > 0){
		// 1. Wait until the TX buffer is empty
		while(!SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG));

		// 2. Check DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF)){
			// 16-bit TX buffer
			pSPIx->DR = *((uint16_t *)pTxBuffer);
			(uint16_t *)pTxBuffer++;
			len -= 2;
		}else{
			// 8-bit TX buffer
			pSPIx->DR = *pTxBuffer;
			pTxBuffer++;
			len--;
		}
	}

}

/******************************************************************************
 * @fn 				- SPI_ReceiveData
 *
 * @brief			- This function receives data from the SPI peripheral
 *
 * @param[in]		- Base address of the SPI peripheral
 * @param[in]		- Pointer to the receive buffer
 * @param[in]		- Receive buffer size (number of bytes to receive)
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len){

	while(len){
		// 1. Wait until the RX buffer is not empty
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == RESET);

		// 2. Check DFF bit in CR1
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF)){
			// 16-bit RX buffer
			*((uint16_t *)pRxBuffer) = pSPIx->DR;
			(uint16_t *)pRxBuffer++;
			len -= 2;
		}else{
			// 8-bit RX buffer
			*pRxBuffer = pSPIx->DR;
			pRxBuffer++;
			len--;
		}
	}
}

/*
 * IRQ configuration and ISR handling
 */
void SPI_IRQHandling(SPI_handle_t *pSPIHandle);


/******************************************************************************
 * @fn 				- SPI_IRQInterruptConfig
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
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t status)
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
 * @fn 				- SPI_IRQPriorityConfig
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
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// Set priority in NVIC_IPRx registers
	uint8_t NVIC_IPRx_idx = IRQNumber / 4;
	uint8_t IRQx_idx = IRQNumber % 4;
	__vo uint32_t *NVIC_IPRx = (uint32_t *)(NVIC_IPR_BASEADDR + 4*NVIC_IPRx_idx);

	*NVIC_IPRx &= ~(0xFF << (IRQx_idx * 8));
	*NVIC_IPRx |= (IRQPriority << (IRQx_idx * 8 + (8 - NVIC_NUM_OF_PR_BITS_IMPLEMENTED)));
}


/******************************************************************************
 * @fn 				- SPI_PeripheralControl
 *
 * @brief			- This function enables or disables the SPI peripheral
 *
 * @param[in]		- Base address of the SPI peripheral
 * @param[in]		- ENABLE or DISABLE
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t status)
{
	if(status == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/******************************************************************************
 * @fn 				- SPI_SSIConfig
 *
 * @brief			- This function enables or disables the SPI SSI bit
 *
 * @param[in]		- Base address of the SPI peripheral
 * @param[in]		- ENABLE or DISABLE
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t status)
{
	if(status == ENABLE){
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/******************************************************************************
 * @fn 				- SPI_SSOEConfig
 *
 * @brief			- This function enables or disables the SPI SSOE bit
 *
 * @param[in]		- Base address of the SPI peripheral
 * @param[in]		- ENABLE or DISABLE
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t status)
{
	if(status == ENABLE){
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}else{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}


/******************************************************************************
 * @fn 				- SPI_SendDataIT
 *
 * @brief			- This function sends data through SPI peripheral using interrupts
 *
 * @param[in]		- SPI peripheral handle
 * @param[in]		- Pointer to the transmission buffer
 * @param[in]		- Number of transmission units
 *
 * @return			- SPI Tx state
 *
 * @Note			- This is a blocking call
 *
 */
uint8_t SPI_SendDataIT(SPI_handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX){
		// 1. Save the Tx buffer address and len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = len;

		// 2. Mark the SPI state as busy in transmission so that no other code can take over
		//    same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		// 3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}
	// 4. Data transmission will be handled by the ISR code

	return state;
}

/******************************************************************************
 * @fn 				- SPI_ReceiveDataIT
 *
 * @brief			- This function receives data from the SPI peripheral using interrupts
 *
 * @param[in]		- SPI peripheral handle
 * @param[in]		- Pointer to the receive buffer
 * @param[in]		- Receive buffer size (number of bytes to receive)
 *
 * @return			- SPI Rx state
 *
 * @Note			- none
 *
 */
uint8_t SPI_ReceiveDataIT(SPI_handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX){
		// 1. Save the Rx buffer address and len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = len;

		// 2. Mark the SPI state as busy in reception so that no other code can take over
		//    same SPI peripheral until reception is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		// 3. Enable the RXNEIE control bit to get interrupt whenever RXNE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}
	// 4. Data transmission will be handled by the ISR code

	return state;
}

/******************************************************************************
 * @fn 				- SPI_IRQHandling
 *
 * @brief			- This function is the IRQ handler for the SPI peripheral
 *
 * @param[in]		- SPI peripheral handle
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void SPI_IRQHandling(SPI_handle_t *pSPIHandle)
{
	uint8_t SR_flag, IE_flag;

	// Check TXE and TXEIE flags
	SR_flag = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	IE_flag = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if(SR_flag && IE_flag){
		// Handle TXE
		spi_txe_interrupt_handle(pSPIHandle);
	}

	// Check RXNE and RXNEIE flags
	SR_flag = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	IE_flag = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if(SR_flag && IE_flag){
		// Handle RXE
		spi_rxne_interrupt_handle(pSPIHandle);
	}

	// Check OVR and ERRIE flags
	SR_flag = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	IE_flag = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if(SR_flag && IE_flag){
		// Handle over run error
		spi_ovr_err_interrupt_handle(pSPIHandle);
	}
}

/*
 * Helper functions
 *
 */

static void spi_txe_interrupt_handle(SPI_handle_t *pSPIHandle)
{
	// Check DFF bit in CR1
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)){
		// 16-bit TX buffer
		pSPIHandle->pSPIx->DR = *((uint16_t *)pSPIHandle->pTxBuffer);
		(uint16_t *)pSPIHandle->pTxBuffer++;
		pSPIHandle->TxLen -= 2;
	}else{
		// 8-bit TX buffer
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTxBuffer;
		pSPIHandle->pTxBuffer++;
		pSPIHandle->TxLen--;
	}

	if(!pSPIHandle->TxLen){
		// TxLen is zero, so close the spi transmission and inform the application that Tx is over

		SPI_CloseTransmission(pSPIHandle);

		// Inform the application through a callback
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);

	}
}
static void spi_rxne_interrupt_handle(SPI_handle_t *pSPIHandle)
{

	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)){
		// 16-bit RX buffer
		*((uint16_t *)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->DR;
		(uint16_t *)pSPIHandle->pRxBuffer++;
		pSPIHandle->RxLen -= 2;
	}else{
		// 8-bit RX buffer
		*pSPIHandle->pRxBuffer = pSPIHandle->pSPIx->DR;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->RxLen--;
	}

	if(!pSPIHandle->RxLen){
		// RxLen is zero, so close the spi reception and inform the application that Rx is over

		SPI_CloseReception(pSPIHandle);

		// Inform the application through a callback
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);

	}
}
static void spi_ovr_err_interrupt_handle(SPI_handle_t *pSPIHandle)
{
	uint8_t temp;
	// 1. Clear the ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		// if spi is in SPI_BUSY_IN_TX, the application will have to clear this flags by calling SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx) function

		// 1.1 Read the SPI_DR register
		temp = pSPIHandle->pSPIx->DR;
		// 1.2 Read the SPI_SR register
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;

	// 2. Inform application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}


void SPI_CloseTransmission(SPI_handle_t *pSPIHandle)
{
	// Prevent interrupts from setting up of TXE flag
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	// Reset variables
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_handle_t *pSPIHandle)
{
	// Prevent interrupts from setting up of RXNE flag
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	// Reset variables
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	// 1 Read the SPI_DR register
	temp = pSPIx->DR;
	// 2 Read the SPI_SR register
	temp = pSPIx->SR;

	(void)temp;
}

__weak void SPI_ApplicationEventCallback(SPI_handle_t *pSPIHandle, uint8_t AppEv)
{
	// This is a weak implementation. The application may override this function
}
