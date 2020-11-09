/*
 * stm32f407xx_usart.c
 *
 *  Created on: 8 nov. 2020
 *      Author: javi
 */

#include <stm32f407xx_usart.h>
#include <stm32f407xx_rcc.h>

/******************************************************************************
 * @fn 				- USART_ClockControl
 *
 * @brief			- This function enables or disables peripheral clock for the given USART
 *
 * @param[in]		- USARTx peripheral
 * @param[in]		- ENABLE or DISABLE macros
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void USART_ClockControl(USART_RegDef_t *pUSARTx, uint8_t status)
{
	if(status == ENABLE){
		if(pUSARTx == USART1)
			USART1_PCLK_EN();
		else if(pUSARTx == USART2)
			USART2_PCLK_EN();
		else if(pUSARTx == USART3)
			USART3_PCLK_EN();
		else if(pUSARTx == UART4)
			UART4_PCLK_EN();
		else if(pUSARTx == UART5)
			UART5_PCLK_EN();
		else if(pUSARTx == USART6)
			USART6_PCLK_EN();
	}else if(status == DISABLE){
		if(pUSARTx == USART1)
			USART1_PCLK_DI();
		else if(pUSARTx == USART2)
			USART2_PCLK_DI();
		else if(pUSARTx == USART3)
			USART3_PCLK_DI();
		else if(pUSARTx == UART4)
			UART4_PCLK_DI();
		else if(pUSARTx == UART5)
			UART5_PCLK_DI();
		else if(pUSARTx == USART6)
			USART6_PCLK_DI();
	}
}

/******************************************************************************
 * @fn 				- USART_PeripheralControl
 *
 * @brief			- This function enables or disables the USART peripheral
 *
 * @param[in]		- Base address of the USART peripheral
 * @param[in]		- ENABLE or DISABLE
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t status)
{
	if(status == ENABLE){
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	}else if(status == DISABLE){
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}
}

/******************************************************************************
 * @fn 				- USART_GetFlagStatus
 *
 * @brief			- This function gets the status of an USART flag
 *
 * @param[in]		- Base address of the USART peripheral
 * @param[in]		- Flag to check
 *
 * @return			- Flag status
 *
 * @Note			- none
 *
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t flag)
{
	if(pUSARTx->SR & flag)
		return SET;

	return RESET;
	//return (pUSARTx->SR & (1 << flag));
}

/******************************************************************************
 * @fn 				- USART_ClearFlag
 *
 * @brief			- This function clears the status of an USART flag
 *
 * @param[in]		- Base address of the USART peripheral
 * @param[in]		- Flag to clear
 *
 * @return			- none
 *
 * @Note			- Not all flags in SR can be cleared by writing in the register
 *
 */
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t flag)
{
	pUSARTx->SR &= ~(1 << flag);
}


/******************************************************************************
 * @fn 				- USART_Init
 *
 * @brief			- This function initializes the given USART peripheral
 *
 * @param[in]		- USARTx handle containing the I2Cx base address and the I2C configuration
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void USART_Init(USART_handle_t *pUSARTHandle)
{
	uint32_t tempreg = 0;

	// Enable peripheral clock
	USART_ClockControl(pUSARTHandle->pUSARTx, ENABLE);

	/******************************** Configuration of CR1 ******************************************/
	/* USART MODE */
	if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX){
		tempreg |= (1 << USART_CR1_RE);
	}else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX){
		tempreg |= (1 << USART_CR1_TE);
	}else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX){
		tempreg |= (1 << USART_CR1_TE) | (1 << USART_CR1_RE);
	}

	/* PARITY */
	if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN){
		tempreg |= (1 << USART_CR1_PCE);
		// parity even bit set to zero (not need to do anything since tempreg was initialized to zero)
	}else if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD){
		tempreg |= (1 << USART_CR1_PCE);
		tempreg |= (1 << USART_CR1_PS);
	}

	/* WORD LENGTH */
	tempreg |= (pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M);

	pUSARTHandle->pUSARTx->CR1 = tempreg;

	/******************************** Configuration of CR2 ******************************************/
	tempreg = 0;
	/* STOP BITS */
	tempreg |= (pUSARTHandle->USART_Config.USART_StopBits << USART_CR2_STOP);

	pUSARTHandle->pUSARTx->CR2 = tempreg;

	/******************************** Configuration of CR3 ******************************************/
	tempreg = 0;
	/* HW FLOW CONTROL */
	if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS){
		tempreg |= (1 << USART_CR3_CTSE);
	}else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS){
		tempreg |= (1 << USART_CR3_RTSE);
	}else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS){
		tempreg |= (1 << USART_CR3_CTSE) | (1 << USART_CR3_RTSE);
	}

	pUSARTHandle->pUSARTx->CR3 = tempreg;

	/*********************** Configuration of BRR(Baudrate register)*********************************/
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);


}

/******************************************************************************
 * @fn 				- USART_SetBaudRate
 *
 * @brief			- This function sets the baud rate for the given USART peripheral
 *
 * @param[in]		- Base address of the USART peripheral
 * @param[in]		- Baud rate to set
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
	uint32_t tempreg = 0;
	uint32_t usartdiv;
	uint32_t DIV_Fraction;
	uint32_t DIV_Mantissa;
	uint8_t oversampling;
	uint8_t carry = 0;
	uint32_t PCLKx;


	if(pUSARTx == USART1 || pUSARTx == USART6){
		// USART1 and USART6 are hanging on APB2 BUS
		PCLKx = RCC_GetPCLK2Value();
	}else{
		// Other USARTs are hanging on APB1 BUS
		PCLKx = RCC_GetPCLK1Value();
	}

	// Check for OVER8 configuration bit
	if(pUSARTx->CR1 & (1 << USART_CR1_OVER8)){
		// OVER8 = 1, Oversampling by 8
		oversampling = 8;

	}else{
		// OVER8 = 0, Oversampling by 16
		oversampling = 16;
	}

	usartdiv = (PCLKx * 100)/(oversampling*BaudRate); // Add a scale factor of 100

	DIV_Fraction = (usartdiv % 100)*oversampling;
	DIV_Fraction = (DIV_Fraction + 50)/100; // includes rounding

	if(DIV_Fraction >= oversampling){
		carry = 1;
		DIV_Fraction = 0;
	}

	DIV_Mantissa = usartdiv/100 + carry;

	if(oversampling == 16)
		tempreg |= (DIV_Mantissa << 4) | (DIV_Fraction & 0x0f);
	else
		tempreg |= (DIV_Mantissa << 4) | (DIV_Fraction & 0x07);

	pUSARTx->BRR = tempreg;
}

/******************************************************************************
 * @fn 				- USART_DeInit
 *
 * @brief			- This function de-initializes the given USART peripheral
 *
 * @param[in]		- Base address of the USART peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 */
void USART_DeInit(USART_RegDef_t *pUSARTx)
{
	if(pUSARTx == USART1)
		USART1_RST();
	else if(pUSARTx == USART2)
		USART2_RST();
	else if(pUSARTx == USART3)
		USART3_RST();
	else if(pUSARTx == UART4)
		UART4_RST();
	else if(pUSARTx == UART5)
		UART5_RST();
	else if(pUSARTx == USART6)
		USART6_RST();
}


/******************************************************************************
 * @fn      		  - USART_SendData
 *
 * @brief             - This function sends data through USART peripheral
 *
 * @param[in]         - USARTx handle containing the I2Cx base address and the I2C configuration
 * @param[in]         - Tx buffer to send
 * @param[in]         - len of Tx buffer
 *
 * @return            - none
 *
 * @Note              - none

 */
void USART_SendData(USART_handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t len)
{

	uint16_t *pdata;

	for(uint32_t i = 0 ; i < len; i++){

		// Wait until TXE flag is set in the SR
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_TXE_FLAG));

         //Check the USART_WordLength item for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			// if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF); // Note: bits 10-16 will be discarded, the application has to create the buffer data accordingly

			//check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else
		{
			//This is 8bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);

			//Implement the code to increment the buffer address
			pTxBuffer++;
		}
	}

	// Wait till TC flag is set in the SR
	while( ! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_TC_FLAG));
}


/*********************************************************************
 * @fn      		  - USART_ReceiveData
 *
 * @brief             -
 *
 * @param[in]         - USARTx handle containing the I2Cx base address and the I2C configuration
 * @param[in]         - Buffer to store the received data
 * @param[in]         - Number of bytes to receive
 *
 * @return            - none
 *
 * @Note              - none

 */
void USART_ReceiveData(USART_handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t len)
{
   //Loop over until "len" number of bytes are transferred
	for(uint32_t i = 0 ; i < len; i++)
	{
		// Wait until RXNE flag is set in the SR
		while(!USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_RXNE_FLAG));

		//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//We are going to receive 9bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used. so, all 9bits will be of user data

				//read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

				//Now increment the pRxBuffer two times
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				 *pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

				 //Increment the pRxBuffer
				 pRxBuffer++;
			}
		}
		else
		{
			//We are going to receive 8bit data in a frame

			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data

				//read 8 bits from DR
				 *pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
			}

			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity

				//read only 7 bits , hence mask the DR with 0X7F
				 *pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);

			}
			pRxBuffer++;
		}
	}

}

/*********************************************************************
 * @fn      		  - USART_SendDataIT
 *
 * @brief             -
 *
 * @param[in]         - USARTx handle containing the I2Cx base address and the I2C configuration
 * @param[in]         - Buffer to store the received data
 * @param[in]         - Number of bytes to receive
 *
 * @return            - none
 *
 * @Note              - none

 */
uint8_t USART_SendDataIT(USART_handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t len)
{
	uint8_t txstate = pUSARTHandle->TxRxState;

	if(txstate != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxRxState = USART_BUSY_IN_TX;

		// Enable interrupt for TXE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TXEIE);

		// Enable interrupt for TC
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_TCIE);
	}

	return txstate;

}


/*********************************************************************
 * @fn      		  - USART_ReceiveDataIT
 *
 * @brief             -
 *
 * @param[in]         - USARTx handle containing the I2Cx base address and the I2C configuration
 * @param[in]         - Buffer that holds the data to be sent
 * @param[in]         - Number of bytes to be sent
 *
 * @return            - none
 *
 * @Note              - none

 */
uint8_t USART_ReceiveDataIT(USART_handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t len)
{
	uint8_t rxstate = pUSARTHandle->TxRxState;

	if(rxstate != USART_BUSY_IN_RX)
	{
		pUSARTHandle->RxLen = len;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->TxRxState = USART_BUSY_IN_RX;

		//Implement the code to enable interrupt for RXNE
		pUSARTHandle->pUSARTx->CR1 |= (1 << USART_CR1_RXNEIE);

	}

	return rxstate;

}

/*********************************************************************
 * @fn      		  - USART_IRQHandler
 *
 * @brief             -
 *
 * @param[in]         - USARTx handle containing the I2Cx base address and the I2C configuration
 *
 * @return            - none
 *
 * @Note              - none

 */
void USART_IRQHandling(USART_handle_t *pUSARTHandle)
{
	uint16_t *pdata;
	uint32_t temp1 , temp2, temp3;

/*************************Check for TC flag ********************************************/

    //Implement the code to check the state of TC bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TC);

	 //Implement the code to check the state of TCEIE bit
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TCIE);

	if(temp1 && temp2 )
	{
		//this interrupt is because of TC

		//close transmission and call application callback if TxLen is zero
		if ( pUSARTHandle->TxRxState == USART_BUSY_IN_TX)
		{
			//Check the TxLen . If it is zero then close the data transmission
			if(pUSARTHandle->TxLen == 0)
			{
				//Implement the code to clear the TC flag
				pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_TC);

				//Implement the code to clear the TCIE control bit
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TCIE);

				//Reset the application state
				pUSARTHandle->TxRxState = USART_READY;

				//Reset Buffer address to NULL
				pUSARTHandle->pTxBuffer = NULL;

				//Reset the length to zero
				pUSARTHandle->TxLen = 0;

				//Call the application call back with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_TX_CMPLT);
			}
		}
	}

/*************************Check for TXE flag ********************************************/

	//Implement the code to check the state of TXE bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_TXE);

	//Implement the code to check the state of TXEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_TXEIE);;


	if(temp1 && temp2 )
	{
		//this interrupt is because of TXE

		if(pUSARTHandle->TxRxState == USART_BUSY_IN_TX)
		{
			//Keep sending data until Txlen reaches to zero
			if(pUSARTHandle->TxLen > 0)
			{
				//Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//if 9BIT , load the DR with 2bytes masking the bits other than first 9 bits
					pdata = (uint16_t*)pUSARTHandle->pTxBuffer;

					//loading only first 9 bits , so we have to mask with the value 0x01FF
					pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

					//check for USART_ParityControl
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used in this transfer , so, 9bits of user data will be sent
						//Implement the code to increment pTxBuffer twice
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;

						//Implement the code to decrement the length
						pUSARTHandle->TxLen -= 2;
					}
					else
					{
						//Parity bit is used in this transfer . so , 8bits of user data will be sent
						//The 9th bit will be replaced by parity bit by the hardware
						pUSARTHandle->pTxBuffer++;

						//Implement the code to decrement the length
						pUSARTHandle->TxLen--;
					}
				}
				else
				{
					//This is 8bit data transfer
					pUSARTHandle->pUSARTx->DR = (*(pUSARTHandle->pTxBuffer)  & (uint8_t)0xFF);

					//Implement the code to increment the buffer address
					pUSARTHandle->pTxBuffer++;

					//Implement the code to decrement the length
					pUSARTHandle->TxLen--;
				}

			}
			if (pUSARTHandle->TxLen == 0 )
			{
				//TxLen is zero
				//Implement the code to clear the TXEIE bit (disable interrupt for TXE flag )
				pUSARTHandle->pUSARTx->CR1 &= ~(1 << USART_CR1_TXEIE);
			}
		}
	}

/*************************Check for RXNE flag ********************************************/

	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_RXNE);
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_RXNEIE);


	if(temp1 && temp2 )
	{
		//this interrupt is because of rxne
		//this interrupt is because of txe
		if(pUSARTHandle->TxRxState == USART_BUSY_IN_RX)
		{
			//TXE is set so send data
			if(pUSARTHandle->RxLen > 0)
			{
				//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//We are going to receive 9bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used. so, all 9bits will be of user data

						//read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*)pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

						//Now increment the pRxBuffer two times
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;

						//Implement the code to decrement the length
						pUSARTHandle->RxLen -= 2;
					}
					else
					{
						//Parity is used. so, 8bits will be of user data and 1 bit is parity
						 *(pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

						 //Now increment the pRxBuffer
						 pUSARTHandle->pRxBuffer++;

						 //Implement the code to decrement the length
						 pUSARTHandle->RxLen--;
					}
				}
				else
				{
					//We are going to receive 8bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used , so all 8bits will be of user data

						//read 8 bits from DR
						 *(pUSARTHandle->pRxBuffer) = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
					}

					else
					{
						//Parity is used, so , 7 bits will be of user data and 1 bit is parity

						//read only 7 bits , hence mask the DR with 0X7F
						 *(pUSARTHandle->pRxBuffer) = (uint8_t) (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);

					}

					//Now , increment the pRxBuffer
					pUSARTHandle->pRxBuffer++;

					//Implement the code to decrement the length
					pUSARTHandle->RxLen--;
				}


			}//if of >0

			if(! pUSARTHandle->RxLen)
			{
				//disable the rxne
				pUSARTHandle->pUSARTx->CR1 &= ~( 1 << USART_CR1_RXNEIE );
				pUSARTHandle->TxRxState = USART_READY;
				USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_RX_CMPLT);
			}
		}
	}


/*************************Check for CTS flag ********************************************/
//Note : CTS feature is not applicable for UART4 and UART5

	//Implement the code to check the status of CTS bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_CTS);

	//Implement the code to check the state of CTSE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSE);

	//Implement the code to check the state of CTSIE bit in CR3 (This bit is not available for UART4 & UART5.)
	temp3 = pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_CTSIE);


	if(temp1  && temp2 && temp3)
	{
		//Implement the code to clear the CTS flag in SR
		pUSARTHandle->pUSARTx->SR &= ~( 1 << USART_SR_CTS);

		//this interrupt is because of cts
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_CTS);
	}

/*************************Check for IDLE detection flag ********************************************/

	//Implement the code to check the status of IDLE flag bit in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & ( 1 << USART_SR_IDLE);

	//Implement the code to check the state of IDLEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & ( 1 << USART_CR1_IDLEIE);


	if(temp1 && temp2)
	{
		//Implement the code to clear the IDLE flag. Refer to the RM to understand the clear sequence
		pUSARTHandle->pUSARTx->SR &= ~(1 << USART_SR_IDLE);
		//this interrupt is because of idle
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_IDLE);
	}

/*************************Check for Overrun detection flag ********************************************/

	//Implement the code to check the status of ORE flag  in the SR
	temp1 = pUSARTHandle->pUSARTx->SR & USART_SR_ORE;

	//Implement the code to check the status of RXNEIE  bit in the CR1
	temp2 = pUSARTHandle->pUSARTx->CR1 & USART_CR1_RXNEIE;


	if(temp1  && temp2 )
	{
		//Need not to clear the ORE flag here, instead give an api for the application to clear the ORE flag .

		//this interrupt is because of Overrun error
		USART_ApplicationEventCallback(pUSARTHandle,USART_EVENT_ORE);
	}



/*************************Check for Error Flag ********************************************/

//Noise Flag, Overrun error and Framing Error in multibuffer communication
//We dont discuss multibuffer communication in this course. please refer to the RM
//The blow code will get executed only if multibuffer mode is used.

	temp2 =  pUSARTHandle->pUSARTx->CR3 & ( 1 << USART_CR3_EIE) ;

	if(temp2 )
	{
		temp1 = pUSARTHandle->pUSARTx->SR;
		if(temp1 & ( 1 << USART_SR_FE))
		{
			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (an read to the USART_SR register
				followed by a read to the USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERREVENT_FE);
		}

		if(temp1 & ( 1 << USART_SR_NF) )
		{
			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (an read to the USART_SR register followed by a read to the
				USART_DR register).
			*/
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERREVENT_NF);
		}

		if(temp1 & ( 1 << USART_SR_ORE) )
		{
			USART_ApplicationEventCallback(pUSARTHandle,USART_ERREVENT_ORE);
		}
	}


}


/******************************************************************************
 * @fn 				- USART_IRQInterruptConfig
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
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t status)
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
 * @fn 				- USART_IRQPriorityConfig
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
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// Set priority in NVIC_IPRx registers
	uint8_t NVIC_IPRx_idx = IRQNumber / 4;
	uint8_t IRQx_idx = IRQNumber % 4;
	__vo uint32_t *NVIC_IPRx = (uint32_t *)(NVIC_IPR_BASEADDR + 4*NVIC_IPRx_idx);

	*NVIC_IPRx &= ~(0xFF << (IRQx_idx * 8));
	*NVIC_IPRx |= (IRQPriority << (IRQx_idx * 8 + (8 - NVIC_NUM_OF_PR_BITS_IMPLEMENTED)));
}

__weak void USART_ApplicationEventCallback(USART_handle_t *pUSARTHandle, USART_events_t AppEv)
{

}
