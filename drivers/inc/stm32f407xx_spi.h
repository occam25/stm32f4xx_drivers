
#ifndef _STM32F407XX_SPI_H_
#define _STM32F407XX_SPI_H_

#include "stm32f407xx.h"

/*
 * Configuration structure for SPIx peripheral
 */
typedef struct {
	uint8_t SPI_DeviceMode;				/* <! Possible values from @SPI_DeviceMode >*/
	uint8_t SPI_BusConfig;				/* <! Possible values from @SPI_BusConfig >*/
	uint8_t SPI_SclkSpeed;				/* <! Possible values from @SPI_SclkSpeed >*/
	uint8_t SPI_DFF;					/* <! Possible values from @SPI_DFF >*/
	uint8_t SPI_CPOL;					/* <! Possible values from @SPI_CPOL >*/
	uint8_t SPI_CPHA;					/* <! Possible values from @SPI_CPHA >*/
	uint8_t SPI_SSM;					/* <! Possible values from @SPI_SSM >*/
}SPI_Config_t;

/*
 * Handler structure for for SPIx peripheral
 */
typedef struct {
	SPI_RegDef_t *pSPIx; 				/* <! This holds the base address of SPIx peripheral >*/
	SPI_Config_t SPI_Config;			/* <! This holds SPIx configuration settings >*/
	uint8_t *pTxBuffer;					/* <! To store the app. Tx buffer address >*/
	uint8_t *pRxBuffer;					/* <! To store the app. Rx buffer address >*/
	uint32_t TxLen;						/* <! To store Tx len>*/
	uint32_t RxLen;						/* <! To store Rx len>*/
	uint8_t TxState;					/* <! To store Tx state>*/
	uint8_t RxState;					/* <! To store Rx state>*/
}SPI_handle_t;

/*
 * SPI application states
 */
#define SPI_READY					0
#define SPI_BUSY_IN_RX				1
#define SPI_BUSY_IN_TX				2

/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_SLAVE		0
#define SPI_DEVICE_MODE_MASTER		1

/*
 * Possible SPI application events
 */
#define SPI_EVENT_TX_CMPLT 		0
#define SPI_EVENT_RX_CMPLT		1
#define SPI_EVENT_OVR_ERR		2
#define SPI_EVENT_CRC_ERR		3
/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD					0		/* Full-Duplex */
#define SPI_BUS_CONFIG_HD					1		/* Half-Duplex */
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		2		/* Simplex RX only */

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2					0
#define SPI_SCLK_SPEED_DIV4					1
#define SPI_SCLK_SPEED_DIV8					2
#define SPI_SCLK_SPEED_DIV16				3
#define SPI_SCLK_SPEED_DIV32				4
#define SPI_SCLK_SPEED_DIV64				5
#define SPI_SCLK_SPEED_DIV128				6
#define SPI_SCLK_SPEED_DIV256				7

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BITS		0
#define SPI_DFF_16BITS		1

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_LOW		0
#define SPI_CPOL_HIGH		1

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_LOW		0
#define SPI_CPHA_HIGH		1

/*
 * @SPI_SSM
 */
#define SPI_SSM_DI			0
#define SPI_SSM_EN			1

/*
 * SPI related status flags definitions
 */

#define SPI_RXNE_FLAG			(1 << SPI_SR_RXNE)
#define SPI_TXE_FLAG			(1 << SPI_SR_TXE)
#define SPI_CHSIDE_FLAG			(1 << SPI_SR_CHSIDE)
#define SPI_UDR_FLAG			(1 << SPI_SR_UDR)
#define SPI_CRCERR_FLAG			(1 << SPI_SR_CRCERR)
#define SPI_MODF_FLAG			(1 << SPI_SR_MODF)
#define SPI_OVR_FLAG			(1 << SPI_SR_OVR)
#define SPI_BSY_FLAG			(1 << SPI_SR_BSY)
#define SPI_FRE_FLAG			(1 << SPI_SR_FRE)

/*********************************************************************************************
 * 							APIs supported by this driver
 * 		For more information about the APIs check the function definitions
 *********************************************************************************************/

/*
 *  Peripheral Clock setup
 */
void SPI_ClockControl(SPI_RegDef_t *pSPIx, uint8_t status);

/*
 *  Init and De-init
 */
void SPI_Init(SPI_handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 *  Data send and receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

uint8_t SPI_SendDataIT(SPI_handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len);
uint8_t SPI_ReceiveDataIT(SPI_handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len);

/*
 * IRQ configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t status);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_handle_t *pSPIHandle);


/*
 * Other peripheral control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t status);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t status);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t status);

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint8_t flag);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_handle_t *pSPIHandle);
void SPI_CloseReception(SPI_handle_t *pSPIHandle);

/*
 * Application callback : to be implemented in the application
 */

void SPI_ApplicationEventCallback(SPI_handle_t *pSPIHandle, uint8_t AppEv);

#endif /* _STM32F407XX_SPI_H_ */


