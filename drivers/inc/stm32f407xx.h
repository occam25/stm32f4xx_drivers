/*
 * stm32f407xx.h
 *
 *  Created on: Jun 18, 2020
 *      Author: javi
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stddef.h>
#include <stdint.h>

#define __vo 	volatile
#define __weak __attribute__((weak))

#define HSI_FREQ		16000000  /* Internal oscillator */
#define HSE_FREQ		 8000000  /* Depends on the oscillator installed on the board */

/********************************** Processor Specific Details ****************************************************/
/*
 *  ARM Cortex-Mx Processor NVIC ISERx register addresses (only the first 3 registers)
 */
#define NVIC_ISER0				(*((__vo uint32_t *)0xE000E100U))
#define NVIC_ISER1				(*((__vo uint32_t *)0xE000E104U))
#define NVIC_ISER2				(*((__vo uint32_t *)0xE000E108U))

/*
 *  ARM Cortex-Mx Processor NVIC ICERx register addresses (only the first 3 registers)
 */
#define NVIC_ICER0				(*((__vo uint32_t *)0xE000E180U))
#define NVIC_ICER1				(*((__vo uint32_t *)0xE000E184U))
#define NVIC_ICER2				(*((__vo uint32_t *)0xE000E188U))

/*
 *  ARM Cortex-Mx Processor NVIC IPRx register addresses (only the first 24 registers)
 */
#define NVIC_IPR_BASEADDR		0xE000E400U

/*
 * ARM Cortex Mx Processor, number of priority bits implemented in the NVIC Priority register
 * ( In stm32f4xx MCU the priority bits must be placed in the 4 most significant bits of the section )
 */
#define NVIC_NUM_OF_PR_BITS_IMPLEMENTED			4


/*
 * Generic macros
 */
#define ENABLE			1
#define DISABLE			0
#define SET				ENABLE
#define RESET			DISABLE

#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET


/*
 * Base addresses of flash and SRAM memories
 */

#define FLASH_BASEADDR			0x08000000U
#define SRAM1_BASEADDR			0x20000000U  	// 112KB
#define SRAM2_BASEADDR			0x2001C000U		//  16KB
#define ROM_BASEADDR			0x1FFF0000U
#define OTP_BASEADDR			0x1FFF7800U
#define SRAM 					SRAM1_BASEADDR

/*
 * Base addresses of buses
 */

#define PERIPH_BASEADDR			0x40000000U
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR		0x40010000U
#define AHB1PERIPH_BASEADDR		0x40020000U
#define AHB2PERIPH_BASEADDR		0x50000000U

/*
 * Base addresses of peripherals that are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR			(AHB1PERIPH_BASEADDR + 0x2000)

#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3800)

/*
 * Base addresses of peripherals that are hanging on APB1 bus
 */

#define SPI2_BASEADDR			(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR			(APB1PERIPH_BASEADDR + 0x3C00)

#define USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR			(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR			(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR			(APB1PERIPH_BASEADDR + 0x5000)

#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR + 0x5C00)


/*
 * Base addresses of peripherals that are hanging on APB2 bus
 */

#define USART1_BASEADDR			(APB1PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR			(APB1PERIPH_BASEADDR + 0x1400)

#define SPI1_BASEADDR			(APB1PERIPH_BASEADDR + 0x3000)

#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + 0x3800)

#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x3C00)


/*
 *  Peripheral register definition structures
 */

typedef struct {
	__vo uint32_t MODER;			/*!< GPIO port mode register. 						Address offset: 0x00 >*/
	__vo uint32_t OTYPER;			/*!< GPIO port output type register. 				Address offset: 0x04 >*/
	__vo uint32_t OSPEEDR;			/*!< GPIO port output speed register. 				Address offset: 0x08 >*/
	__vo uint32_t PUPDR;			/*!< GPIO port pull-up/pull-down register. 			Address offset: 0x0C >*/
	__vo uint32_t IDR;				/*!< GPIO port input data register. 				Address offset: 0x10 >*/
	__vo uint32_t ODR;				/*!< GPIO port output data register. 				Address offset: 0x14 >*/
	__vo uint32_t BSRR;				/*!< GPIO port bit set/reset register. 				Address offset: 0x18 >*/
	__vo uint32_t LCKR;				/*!< GPIO port configuration lock register. 		Address offset: 0x1C >*/
	__vo uint32_t AFR[2];			/*!< GPIO port alternate function low register. 	Address offset: AFR[0]: 0x20  AFR[1]: 0x24 >*/
}GPIO_RegDef_t;

typedef struct {
	__vo uint32_t CR;				/*!< RCC clock control register. 									Address offset: 0x00 >*/
	__vo uint32_t PLLCFGR;			/*!< RCC PLL configuration register. 								Address offset: 0x04 >*/
	__vo uint32_t CFGR;				/*!< RCC clock configuration register. 								Address offset: 0x08 >*/
	__vo uint32_t CIR;				/*!< RCC clock interrupt register. 									Address offset: 0x0C >*/
	__vo uint32_t AHB1RSTR;			/*!< RCC AHB1 peripheral reset register. 							Address offset: 0x10 >*/
	__vo uint32_t AHB2RSTR;			/*!< RCC AHB2 peripheral reset register. 							Address offset: 0x14 >*/
	__vo uint32_t AHB3RSTR;			/*!< RCC AHB3 peripheral reset register. 							Address offset: 0x18 >*/

	uint32_t RESERVED0;

	__vo uint32_t APB1RSTR;			/*!< RCC APB1 peripheral reset register. 							Address offset: 0x20 >*/
	__vo uint32_t APB2RSTR;			/*!< RCC APB2 peripheral reset register. 							Address offset: 0x24 >*/

		 uint32_t RESERVED1[2];

    __vo uint32_t AHB1ENR;			/*!< RCC AHB1 peripheral clock enable register. 					Address offset: 0x30 >*/
	__vo uint32_t AHB2ENR;			/*!< RCC AHB2 peripheral clock enable register. 					Address offset: 0x34 >*/
	__vo uint32_t AHB3ENR;			/*!< RCC AHB3 peripheral clock enable register. 					Address offset: 0x38 >*/

	uint32_t RESERVED2;

    __vo uint32_t APB1ENR;			/*!< RCC APB1 peripheral clock enable register. 					Address offset: 0x40 >*/
	__vo uint32_t APB2ENR;			/*!< RCC APB2 peripheral clock enable register. 					Address offset: 0x44 >*/

	uint32_t RESERVED3[2];

	__vo uint32_t AHB1LPENR;		/*!< RCC AHB1 peripheral clock enable in low power mode register. 	Address offset: 0x50 >*/
	__vo uint32_t AHB2LPENR;		/*!< RCC AHB2 peripheral clock enable in low power mode register. 	Address offset: 0x54 >*/
	__vo uint32_t AHB3LPENR;		/*!< RCC AHB3 peripheral clock enable in low power mode register. 	Address offset: 0x58 >*/

	uint32_t RESERVED4;

	__vo uint32_t APB1LPENR;		/*!< RCC APB1 peripheral clock enable in low power mode register. 	Address offset: 0x60 >*/
	__vo uint32_t APB2LPENR;		/*!< RCC APB2 peripheral clock enable in low power mode register. 	Address offset: 0x64 >*/

	uint32_t RESERVED5[2];

	__vo uint32_t BDCR;				/*!< RCC Backup domain control register. 							Address offset: 0x70 >*/
	__vo uint32_t CSR;				/*!< RCC clock control & status register. 							Address offset: 0x74 >*/

	uint32_t RESERVED6[2];

	__vo uint32_t SSCGR;			/*!< RCC spread spectrum clock generation register. 				Address offset: 0x80 >*/
	__vo uint32_t PLLI2SCFGR;		/*!< RCC PLLI2S configuration register. 							Address offset: 0x84 >*/
}RCC_RegDef_t;

typedef struct {
	__vo uint32_t IMR;			/*!< EXTI Interrupt mask register. 						Address offset: 0x00 >*/
	__vo uint32_t EMR;			/*!< EXTI Event mask register. 							Address offset: 0x04 >*/
	__vo uint32_t RTSR;			/*!< EXTI trigger selection register. 					Address offset: 0x08 >*/
	__vo uint32_t FTSR;			/*!< EXTI Falling trigger selection register. 			Address offset: 0x0C >*/
	__vo uint32_t SWIER;		/*!< EXTI Software interrupt event register. 			Address offset: 0x10 >*/
	__vo uint32_t PR;			/*!< EXTI Pending register. 							Address offset: 0x14 >*/
}EXTI_RegDef_t;

typedef struct {
	__vo uint32_t MEMRMP;		/*!< SYSCFG Interrupt mask register. 						Address offset: 0x00 >*/
	__vo uint32_t PMC;			/*!< SYSCFG Event mask register. 							Address offset: 0x04 >*/
	__vo uint32_t EXTICR[4];	/*!< SYSCFG external interrupt configuration register 1-4. 	Address offset: 0x08 - 0x14>*/
	__vo uint32_t RESERVED[2];
	__vo uint32_t CMPCR;		/*!< SYSCFG Compensation cell control register. 			Address offset: 0x20 >*/
}SYSCFG_RegDef_t;

typedef struct {
	__vo uint32_t CR1;			/*!< SPI control register 1 (not used in I2S mode). 			Address offset: 0x00 >*/
	__vo uint32_t CR2;			/*!< SPI control register 2. 									Address offset: 0x04 >*/
	__vo uint32_t SR;			/*!< SPI status register. 										Address offset: 0x08 >*/
	__vo uint32_t DR;			/*!< SPI data register. 										Address offset: 0x0C >*/
	__vo uint32_t CRCPR;		/*!< SPI CRC polynomial register (not used in I2S mode).		Address offset: 0x10 >*/
	__vo uint32_t RXCRCR;		/*!< SPI RX CRC register (not used in I2S mode). 				Address offset: 0x14 >*/
	__vo uint32_t TXCRCR;		/*!< SPI TX CRC register (not used in I2S mode). 				Address offset: 0x18 >*/
	__vo uint32_t I2SCFGR;		/*!< SPI_I2S configuration register. 							Address offset: 0x1C >*/
	__vo uint32_t I2SPR;		/*!< SPI_I2S prescaler register. 								Address offset: 0x20 >*/
}SPI_RegDef_t;

typedef struct {
	__vo uint32_t CR1;			/*!< I2C control register 1.						 			Address offset: 0x00 >*/
	__vo uint32_t CR2;			/*!< I2C control register 2. 									Address offset: 0x04 >*/
	__vo uint32_t OAR1;			/*!< I2C own address register 1.								Address offset: 0x08 >*/
	__vo uint32_t OAR2;			/*!< I2C own address register 2.								Address offset: 0x0C >*/
	__vo uint32_t DR;			/*!< I2C data register.											Address offset: 0x10 >*/
	__vo uint32_t SR1;			/*!< I2C status register 1.						 				Address offset: 0x14 >*/
	__vo uint32_t SR2;			/*!< I2C status register 2. 									Address offset: 0x18 >*/
	__vo uint32_t CCR;			/*!< I2C clock control register. 								Address offset: 0x1C >*/
	__vo uint32_t TRISE;		/*!< I2C TRISE register.	 									Address offset: 0x20 >*/
	__vo uint32_t FLTR;			/*!< I2C FLTR register. 	 									Address offset: 0x24 >*/
}I2C_RegDef_t;

/*
 *  Peripheral definitions (Peripheral base address typecasted to xxx_RegDef_t)
 */

#define GPIOA			((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB			((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC			((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD			((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE			((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF			((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG			((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH			((GPIO_RegDef_t *)GPIOH_BASEADDR)
#define GPIOI			((GPIO_RegDef_t *)GPIOI_BASEADDR)

#define RCC				((RCC_RegDef_t *)RCC_BASEADDR)

#define EXTI			((EXTI_RegDef_t *)EXTI_BASEADDR)

#define SYSCFG			((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)

#define SPI1			((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI2			((SPI_RegDef_t *)SPI2_BASEADDR)
#define SPI3			((SPI_RegDef_t *)SPI3_BASEADDR)

#define I2C1			((I2C_RegDef_t *)I2C1_BASEADDR)
#define I2C2			((I2C_RegDef_t *)I2C2_BASEADDR)
#define I2C3			((I2C_RegDef_t *)I2C3_BASEADDR)

/*
 * GPIO port numbers enum
 */
typedef enum {
	GPIOA_PORT,
	GPIOB_PORT,
	GPIOC_PORT,
	GPIOD_PORT,
	GPIOE_PORT,
	GPIOF_PORT,
	GPIOG_PORT,
	GPIOH_PORT,
	GPIOI_PORT
}GPIO_port_t;

/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOx_PCLK_EN(x)	(RCC->AHB1ENR |= (1 << x))

#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1 << 8))

/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1 << 23))

/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))

/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()		(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()		(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()	(RCC->APB2ENR |= (1 << 5))

/*
 * Clock Enable Macro for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))

/*
 * Clock Enable Macro for ADCx peripherals
 */
#define ADC1_PCLK_EN()		(RCC->APB2ENR |= (1 << 8))
#define ADC2_PCLK_EN()		(RCC->APB2ENR |= (1 << 9))
#define ADC3_PCLK_EN()		(RCC->APB2ENR |= (1 << 10))

/*
 * Clock Enable Macros for TIMx peripherals
 */
#define TIM1_PCLK_EN()		(RCC->APB2ENR |= (1 << 0))
#define TIM2_PCLK_EN()		(RCC->APB1ENR |= (1 << 0))
#define TIM3_PCLK_EN()		(RCC->APB1ENR |= (1 << 1))
#define TIM4_PCLK_EN()		(RCC->APB1ENR |= (1 << 2))
#define TIM5_PCLK_EN()		(RCC->APB1ENR |= (1 << 3))
#define TIM6_PCLK_EN()		(RCC->APB1ENR |= (1 << 4))
#define TIM7_PCLK_EN()		(RCC->APB1ENR |= (1 << 5))

#define TIM8_PCLK_EN()		(RCC->APB2ENR |= (1 << 1))
#define TIM9_PCLK_EN()		(RCC->APB2ENR |= (1 << 16))
#define TIM10_PCLK_EN()		(RCC->APB2ENR |= (1 << 17))
#define TIM11_PCLK_EN()		(RCC->APB2ENR |= (1 << 18))

#define TIM12_PCLK_EN()		(RCC->APB1ENR |= (1 << 6))
#define TIM13_PCLK_EN()		(RCC->APB1ENR |= (1 << 7))
#define TIM14_PCLK_EN()		(RCC->APB1ENR |= (1 << 8))

/*
 * Clock Enable Macros for CANx peripherals
 */
#define CAN1_PCLK_EN()		(RCC->APB1ENR |= (1 << 25))
#define CAN2_PCLK_EN()		(RCC->APB1ENR |= (1 << 26))

/*
 * Clock Enable Macro for DAC peripheral
 */
#define DAC_PCLK_EN()		(RCC->APB1ENR |= (1 << 29))

/*
 * Clock Enable Macro for POWER interface
 */
#define PWR_PCLK_EN()		(RCC->APB1ENR |= (1 << 28))


/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define GPIOx_PCLK_DI(x)	(RCC->AHB1ENR &= ~(1 << x))

#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 8))

/*
 *  This macro returns a the port number
 */
#define GPIO_BASEADDR_TO_CODE(x)		((x == GPIOA) ? 0 : \
										 (x == GPIOB) ? 1 : \
										 (x == GPIOC) ? 2 : \
										 (x == GPIOD) ? 3 : \
										 (x == GPIOE) ? 4 : \
										 (x == GPIOF) ? 5 : \
										 (x == GPIOG) ? 6 : \
										 (x == GPIOH) ? 7 : \
										 (x == GPIOI) ? 8 : 0)

/*
 * Reset Macros for GPIOx peripherals
 */
#define GPIOx_RST(x)		do{ RCC->AHB1RSTR |= (1 << x); RCC->AHB1RSTR &= ~(1 << x); }while(0)

#define GPIOA_RST()			do{ RCC->AHB1RSTR |= (1 << 0); RCC->AHB1RSTR &= ~(1 << 0); }while(0)
#define GPIOB_RST()			do{ RCC->AHB1RSTR |= (1 << 1); RCC->AHB1RSTR &= ~(1 << 1); }while(0)
#define GPIOC_RST()			do{ RCC->AHB1RSTR |= (1 << 2); RCC->AHB1RSTR &= ~(1 << 2); }while(0)
#define GPIOD_RST()			do{ RCC->AHB1RSTR |= (1 << 3); RCC->AHB1RSTR &= ~(1 << 3); }while(0)
#define GPIOE_RST()			do{ RCC->AHB1RSTR |= (1 << 4); RCC->AHB1RSTR &= ~(1 << 4); }while(0)
#define GPIOF_RST()			do{ RCC->AHB1RSTR |= (1 << 5); RCC->AHB1RSTR &= ~(1 << 5); }while(0)
#define GPIOG_RST()			do{ RCC->AHB1RSTR |= (1 << 6); RCC->AHB1RSTR &= ~(1 << 6); }while(0)
#define GPIOH_RST()			do{ RCC->AHB1RSTR |= (1 << 7); RCC->AHB1RSTR &= ~(1 << 7); }while(0)
#define GPIOI_RST()			do{ RCC->AHB1RSTR |= (1 << 8); RCC->AHB1RSTR &= ~(1 << 8); }while(0)

/*
 * Clock Disable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 23))

/*
 * Reset Macros for I2Cx peripherals
 */
#define I2C1_RST()			do{ RCC->APB1RSTR |= (1 << 21); RCC->APB1RSTR &= ~(1 << 21); }while(0)
#define I2C2_RST()			do{ RCC->APB1RSTR |= (1 << 22); RCC->APB1RSTR &= ~(1 << 22); }while(0)
#define I2C3_RST()			do{ RCC->APB1RSTR |= (1 << 23); RCC->APB1RSTR &= ~(1 << 23); }while(0)

/*
 * Clock Disable Macros for SPIx peripherals
 */
#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 15))

/*
 * Reset Macros for SPIx peripherals
 */
#define SPI1_RST()			do{ RCC->APB2RSTR |= (1 << 12); RCC->APB2RSTR &= ~(1 << 12); }while(0)
#define SPI2_RST()			do{ RCC->APB1RSTR |= (1 << 14); RCC->APB1RSTR &= ~(1 << 14); }while(0)
#define SPI3_RST()			do{ RCC->APB1RSTR |= (1 << 15); RCC->APB1RSTR &= ~(1 << 15); }while(0)

/*
 * Clock Disable Macros for USARTx peripherals
 */
#define USART1_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 5))

/*
 * Clock Disable Macro for SYSCFG peripheral
 */
#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 14))

/*
 * Clock Disable Macro for ADCx peripherals
 */
#define ADC1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 8))
#define ADC2_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 9))
#define ADC3_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 10))

/*
 * Clock Disable Macros for TIMx peripherals
 */
#define TIM1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 0))
#define TIM2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 0))
#define TIM3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 1))
#define TIM4_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 2))
#define TIM5_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 3))
#define TIM6_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 4))
#define TIM7_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 5))

#define TIM8_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 1))
#define TIM9_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 16))
#define TIM10_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 17))
#define TIM11_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 18))

#define TIM12_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 6))
#define TIM13_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 7))
#define TIM14_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 8))

/*
 * Clock Disable Macros for CANx peripherals
 */
#define CAN1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 25))
#define CAN2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 26))

/*
 * Clock Disable Macro for DAC peripheral
 */
#define DAC_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 29))

/*
 * Clock Disable Macro for POWER interface
 */
#define PWR_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 28))



/*
 * EXTI number to corresponding IRQ
 */


#define EXTI_TO_IRQ(x)					((x == EXTI0) ? 6 : \
										 (x == EXTI1) ? 7 : \
										 (x == EXTI2) ? 8 : \
										 (x == EXTI3) ? 9 : \
										 (x == EXTI4) ? 10 : \
										 (x > EXTI4 && x < EXTI10) ? 23 : \
										 (x > EXTI9) ? 40 : 0)
/*
 *  IRQ (Interrupt Request) Numbers of STM32F407x MCU
 */
#define IRQ_EXTI0				6
#define IRQ_EXTI1				7
#define IRQ_EXTI2				8
#define IRQ_EXTI3				9
#define IRQ_EXTI4				10
#define IRQ_EXTI9_5				23
#define IRQ_EXTI15_10			40

#define IRQ_SPI1				35
#define IRQ_SPI2				36
#define IRQ_SPI3				51

#define IRQ_I2C1_EV				31
#define IRQ_I2C1_ER				32
#define IRQ_I2C2_EV				33
#define IRQ_I2C2_ER				34
#define IRQ_I2C3_EV				72
#define IRQ_I2C3_ER				73

/*
 *  IRQ priority levels
 */
#define NVIC_IRQ_PRIO0			0
#define NVIC_IRQ_PRIO1			1
#define NVIC_IRQ_PRIO2			2
#define NVIC_IRQ_PRIO3			3
#define NVIC_IRQ_PRIO4			4
#define NVIC_IRQ_PRIO5			5
#define NVIC_IRQ_PRIO6			6
#define NVIC_IRQ_PRIO7			7
#define NVIC_IRQ_PRIO8			8
#define NVIC_IRQ_PRIO9			9
#define NVIC_IRQ_PRIO10			10
#define NVIC_IRQ_PRIO11			11
#define NVIC_IRQ_PRIO12			12
#define NVIC_IRQ_PRIO13			13
#define NVIC_IRQ_PRIO14			14
#define NVIC_IRQ_PRIO15			15
#define NVIC_IRQ_PRIO16			16
#define NVIC_IRQ_PRIO17			17
#define NVIC_IRQ_PRIO18			18
#define NVIC_IRQ_PRIO19			19
#define NVIC_IRQ_PRIO20			20


/*
 *  Bit position definitions for SPI peripheral
 */
#define SPI_CR1_CPHA			 0
#define SPI_CR1_CPOL			 1
#define SPI_CR1_MSTR			 2
#define SPI_CR1_BR				 3
#define SPI_CR1_SPE				 6
#define SPI_CR1_LSB_FIRST		 7
#define SPI_CR1_SSI				 8
#define SPI_CR1_SSM				 9
#define SPI_CR1_RXONLY			10
#define SPI_CR1_DFF				11
#define SPI_CR1_CRCNEXT			12
#define SPI_CR1_CRCEN			13
#define SPI_CR1_BIDIOE			14
#define SPI_CR1_BIDIMODE		15

#define SPI_CR2_RXDMAEN			 0
#define SPI_CR2_TXDMAEN			 1
#define SPI_CR2_SSOE			 2
#define SPI_CR2_FRF				 4
#define SPI_CR2_ERRIE			 5
#define SPI_CR2_RXNEIE			 6
#define SPI_CR2_TXEIE			 7

#define SPI_SR_RXNE				 0
#define SPI_SR_TXE				 1
#define SPI_SR_CHSIDE			 2
#define SPI_SR_UDR				 3
#define SPI_SR_CRCERR			 4
#define SPI_SR_MODF				 5
#define SPI_SR_OVR				 6
#define SPI_SR_BSY				 7
#define SPI_SR_FRE				 8

/*
 *  Bit position definitions for I2C peripheral
 */
#define I2C_CR1_PE		 		 0
#define I2C_CR1_SMBUS	  		 1
#define I2C_CR1_SMBTYPE			 3
#define I2C_CR1_ENARP			 4
#define I2C_CR1_ENPEC			 5
#define I2C_CR1_ENGC			 6
#define I2C_CR1_NOSTRETCH		 7
#define I2C_CR1_START			 8
#define I2C_CR1_STOP			 9
#define I2C_CR1_ACK				10
#define I2C_CR1_POS				11
#define I2C_CR1_PEC				12
#define I2C_CR1_ALERT			13
#define I2C_CR1_SWRST			15

#define I2C_CR2_FREQ	 		 0
#define I2C_CR2_ITERREN	  		 8
#define I2C_CR2_ITEVTEN			 9
#define I2C_CR2_ITBUFEN			10
#define I2C_CR2_DMAEN			11
#define I2C_CR2_LAST			12

#define I2C_OAR1_ADD0	 		 0
#define I2C_OAR1_ADD7_1		  	 1
#define I2C_OAR1_ADD9_8			 8
#define I2C_OAR1_ADDMODE		15

#define I2C_OAR2_ENDUAL	 		 0
#define I2C_OAR2_ADD2		  	 1

#define I2C_SR1_SB		 		 0
#define I2C_SR1_ADDR	  		 1
#define I2C_SR1_BTF				 2
#define I2C_SR1_ADD10			 3
#define I2C_SR1_STOPF			 4
#define I2C_SR1_RXNE			 6
#define I2C_SR1_TXE				 7
#define I2C_SR1_BERR			 8
#define I2C_SR1_ARLO			 9
#define I2C_SR1_AF				10
#define I2C_SR1_OVR				11
#define I2C_SR1_PECERR			12
#define I2C_SR1_TIMEOUT			14
#define I2C_SR1_SMBALERT		15

#define I2C_SR2_MSL		 		 0
#define I2C_SR2_BUSY	  		 1
#define I2C_SR2_TRA				 2
#define I2C_SR2_GENCALL			 4
#define I2C_SR2_SMBDEFAULT		 5
#define I2C_SR2_SMBHOST			 6
#define I2C_SR2_DUALF			 7
#define I2C_SR2_PEC				 8

#define I2C_CCR_CCR		 		 0
#define I2C_CCR_DUTY	  		14
#define I2C_CCR_FS				15

#define I2C_FLTR_DNF	 		 0
#define I2C_FLTR_ANOFF	  		 4

#endif /* INC_STM32F407XX_H_ */
