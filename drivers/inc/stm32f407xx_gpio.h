/*
 * stm32f407xx_gpio.h
 *
 *  Created on: Jun 19, 2020
 *      Author: javi
 */

#ifndef _STM32F407XX_GPIO_H_
#define _STM32F407XX_GPIO_H_

#include "stm32f407xx.h"

/*
 * Configuration structure for a GPIO pin
 */
typedef struct {
	uint8_t GPIO_PinNumber;				/* <! Possible values from @GPIO_PIN_NUMBERS >*/
	uint8_t GPIO_PinMode;				/* <! Possible values from @GPIO_PIN_MODES >*/
	uint8_t GPIO_PinSpeed;				/* <! Possible values from @GPIO_PIN_SPEED >*/
	uint8_t GPIO_PinPuPdControl;		/* <! Possible values from @GPIO_PIN_PUPD >*/
	uint8_t GPIO_PinOPType;				/* <! Possible values from @GPIO_PIN_OPTYPES >*/
	uint8_t GPIO_PinAltFunMode;
}GPIO_PinConfig_t;

/*
 * Handler structure for a GPIO pin
 */
typedef struct {
	GPIO_RegDef_t *pGPIOx; 				/* <! This holds the base address of the GPIO port to which the pin belongs >*/
	GPIO_port_t   GPIOx_port;
	GPIO_PinConfig_t GPIO_PinConfig;	/* <! This holds GPIO pin configuration settings >*/
}GPIO_handle_t;

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_0			0
#define GPIO_PIN_1			1
#define GPIO_PIN_2			2
#define GPIO_PIN_3			3
#define GPIO_PIN_4			4
#define GPIO_PIN_5			5
#define GPIO_PIN_6			6
#define GPIO_PIN_7			7
#define GPIO_PIN_8			8
#define GPIO_PIN_9			9
#define GPIO_PIN_10			10
#define GPIO_PIN_11			11
#define GPIO_PIN_12			12
#define GPIO_PIN_13			13
#define GPIO_PIN_14			14
#define GPIO_PIN_15			15

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

/*
 * @GPIO_PIN_OPTYPES
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

/*
 * @GPIO_PIN_PUPD
 * GPIO pin pull up and pull down configuration macros
 */
#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2

/*
 * @GPIO_PIN_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_HIGH			3



/****************************************************************************
 * 					APIs supported by this driver
 * 		For more information check the function definitions
 ***************************************************************************/

/*
 * Peripheral clock control
 */
void GPIO_ClockControl(GPIO_port_t GPIOx_port, uint8_t status);

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_port_t GPIOx_port);


/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t status);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

#endif /* _STM32F407XX_GPIO_H_ */
