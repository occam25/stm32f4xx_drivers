/*
 * stm32f407xx_rcc.h
 *
 *  Created on: 9 nov. 2020
 *      Author: javi
 */

#ifndef INC_STM32F407XX_RCC_H_
#define INC_STM32F407XX_RCC_H_

#include "stm32f407xx.h"


uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);
uint32_t RCC_GetPLLOutputClock(void);

#endif /* INC_STM32F407XX_RCC_H_ */
