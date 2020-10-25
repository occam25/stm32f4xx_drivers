
#include <string.h>

#include "stm32f407xx.h"
#include "stm32f407xx_spi.h"
#include "stm32f407xx_gpio.h"

volatile uint8_t button_pressed_f;

void delay(uint32_t count);

void EXTI0_IRQHandler(void)
{
	GPIO_IRQHandling(0);

	button_pressed_f = 1;
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);
}

void SPI2_GPIO_init(void)
{
	GPIO_handle_t spi2_pins = {0};

	/*
	 * PB15 -> SPI2_MOSI
	 * PB14 -> SPI2_MISO
	 * PB13 -> SPI2_SCLK
	 * PB12 -> SPI2_NSS
	 * ALT function mode : 5
	 */

//	GPIO_ClockControl(GPIOB_PORT, ENABLE); //no es necesario, ahora se habilita el clock dentro de GPIO_Init

	spi2_pins.pGPIOx = GPIOB;
	spi2_pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	spi2_pins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	spi2_pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	spi2_pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	spi2_pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	// SCLK
	spi2_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&spi2_pins);

	// MOSI
	spi2_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&spi2_pins);

//	// MISO
//	spi2_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
//	GPIO_Init(&spi2_pins);
//
	// NSS
	spi2_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&spi2_pins);

}

void GPIO_setup(void)
{
	GPIO_handle_t button = {0};
	GPIO_handle_t LED = {0};

	GPIO_ClockControl(GPIOA_PORT, ENABLE);
	GPIO_ClockControl(GPIOD_PORT, ENABLE);

	LED.pGPIOx = GPIOD;
	LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	LED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	LED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	LED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	LED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&LED);

	button.pGPIOx = GPIOA;
	button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;

	GPIO_Init(&button);

	// GPIO IRQ Configurations
	GPIO_IRQPriorityConfig(IRQ_EXTI0, NVIC_IRQ_PRIO15); // Optional
	GPIO_IRQInterruptConfig(IRQ_EXTI0, ENABLE);
}

void SPI2_init(void)
{
	SPI_handle_t spi2 = {0};

//	SPI_ClockControl(SPI2, ENABLE); //no es necesario, ahora se habilita el clock dentro de SPI_Init

	spi2.pSPIx = SPI2;
	spi2.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	spi2.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	spi2.SPI_Config.SPI_SSM = SPI_SSM_DI; // hardware slave management through NSS pin
	spi2.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;  // SCLK of 2MHz
	spi2.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	spi2.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	spi2.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;

	SPI_Init(&spi2);
}

void HW_setup(void)
{
	SPI2_GPIO_init();
	SPI2_init();
	GPIO_setup();
}

int main(void)
{

	char data_to_send[] = "Hello world";

	HW_setup();

	/*
	 * making SSOE 1 does NSS output enable.
	 * The NSS pin is automatically managed by the hardware.
	 * When SPE = 1, NSS will be pulled to low
	 * When SPE = 0, NSS will be pushed to high
	 */
	SPI_SSOEConfig(SPI2, ENABLE);

//	// Enable the SPI2 peripheral
//	SPI_PeripheralControl(SPI2, ENABLE);

	while(1){
		if(button_pressed_f){
			button_pressed_f = 0;

			// Enable the SPI2 peripheral
			SPI_PeripheralControl(SPI2, ENABLE);

			// first send length information
			uint8_t data_len = strlen(data_to_send);
			SPI_SendData(SPI2, &data_len, 1);

			SPI_SendData(SPI2, (uint8_t *)data_to_send, strlen(data_to_send));

			while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));

			// Disable the SPI2 peripheral
			SPI_PeripheralControl(SPI2, DISABLE);
		}
	}


	return 0;
}

void delay(uint32_t count)
{
	for(uint32_t i = 0; i < count; i++);
}


