/*
 * stm32f407xx_i2c.h
 *
 *  Created on: Oct 17, 2020
 *      Author: javi
 */

#ifndef INC_STM32F407XX_I2C_H_
#define INC_STM32F407XX_I2C_H_

#include "stm32f407xx.h"

/*
 * Configuration structure for I2Cx peripheral
 */
typedef struct {
	uint32_t I2C_SCLSpeed;				/* <! Possible values from @I2C_SCLSpeed >*/
	uint8_t I2C_DeviceAddress;			/* <! Supplied by the user >*/
	uint8_t I2C_ACKControl;				/* <! Possible values from @I2C_ACKControl >*/
	uint16_t I2C_FMDutyCycle;			/* <! Possible values from @I2C_FMDutyCycle >*/
}I2C_Config_t;

/*
 * Handler structure for for I2Cx peripheral
 */
typedef struct {
	I2C_RegDef_t *pI2Cx; 				/* <! This holds the base address of I2Cx peripheral >*/
	I2C_Config_t I2C_Config;			/* <! This holds I2Cx configuration settings >*/

}I2C_handle_t;


/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM	100000
#define I2C_SCL_SPEED_FM2K	200000
#define I2C_SCL_SPEED_FM4K	400000

/*
 * @I2C_ACKControl
 */
#define I2C_ACK_ENABLE			1
#define I2C_ACK_DISABLE			0

/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2			0
#define I2C_FM_DUTY_19_9		1


/*
 * I2C related status flags definitions
 */
#define I2C_SB_FLAG				(1 << I2C_SR1_SB)
#define I2C_ADDR_FLAG			(1 << I2C_SR1_ADDR)
#define I2C_BTF_FLAG			(1 << I2C_SR1_BTF)
#define I2C_ADD10_FLAG			(1 << I2C_SR1_ADD10)
#define I2C_STOPF_FLAG			(1 << I2C_SR1_STOPF)
#define I2C_RXNE_FLAG			(1 << I2C_SR1_RXNE)
#define I2C_TXE_FLAG			(1 << I2C_SR1_TXE)
#define I2C_BERR_FLAG			(1 << I2C_SR1_BERR)
#define I2C_ARLO_FLAG			(1 << I2C_SR1_ARLO)
#define I2C_AF_FLAG				(1 << I2C_SR1_AF)
#define I2C_OVR_FLAG			(1 << I2C_SR1_OVR)
#define I2C_PECERR_FLAG			(1 << I2C_SR1_PECERR)
#define I2C_TIMEOUT_FLAG		(1 << I2C_SR1_TIMEOUT)
#define I2C_SMBALERT_FLAG		(1 << I2C_SR1_SMBALERT)

#define I2C_MSL_FLAG			(1 << I2C_SR2_MSL)
#define I2C_BUSY_FLAG			(1 << I2C_SR2_BUSY)
#define I2C_TRA_FLAG			(1 << I2C_SR2_TRA)
#define I2C_GENCALL_FLAG		(1 << I2C_SR2_GENCALL)
#define I2C_SMBDEFAULT_FLAG		(1 << I2C_SR2_SMBDEFAULT)
#define I2C_SMBHOST_FLAG		(1 << I2C_SR2_SMBHOST)
#define I2C_DUALF_FLAG			(1 << I2C_SR2_DUALF)

/*
 * Useful definitions
 */
#define I2C_WRITE	0
#define I2C_READ	1

#define I2C_NO_SR	0
#define I2C_SR		1
/*********************************************************************************************
 * 							APIs supported by this driver
 * 		For more information about the APIs check the function definitions
 *********************************************************************************************/

/*
 *  Peripheral Clock setup
 */
void I2C_ClockControl(I2C_RegDef_t *pI2Cx, uint8_t status);

/*
 *  Init and De-init
 */
void I2C_Init(I2C_handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/*
 *  Data send and receive
 */
void I2C_MasterSendData(I2C_handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t start_repetition);
void I2C_MasterReceiveData(I2C_handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t start_repetition);

/*
 * IRQ configuration and ISR handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t status);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);


/*
 * Other peripheral control APIs
 */
void I2C_PeripheralControl(I2C_handle_t *pI2CHandle, uint8_t status);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint8_t flag, uint32_t SR);


/*
 * Application callback : to be implemented in the application
 */

void I2C_ApplicationEventCallback(I2C_handle_t *pI2CHandle, uint8_t AppEv);

#endif /* INC_STM32F407XX_I2C_H_ */
