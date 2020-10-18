
#include <string.h>

#include "stm32f407xx.h"
#include "stm32f407xx_spi.h"
#include "stm32f407xx_gpio.h"



void delay(uint32_t count);


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
	spi2_pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

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
//	// NSS
//	spi2_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
//	GPIO_Init(&spi2_pins);

}

void SPI2_init(void)
{
	SPI_handle_t spi2 = {0};

//	SPI_ClockControl(SPI2, ENABLE); //no es necesario, ahora se habilita el clock dentro de SPI_Init

	spi2.pSPIx = SPI2;
	spi2.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	spi2.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	spi2.SPI_Config.SPI_SSM = SPI_SSM_EN; // software slave management enabled for NSS pin
	spi2.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;  // SCLK of 8MHz
	spi2.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	spi2.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	spi2.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;

	SPI_Init(&spi2);
}

void HW_setup(void)
{
	SPI2_GPIO_init();
	SPI2_init();
}

int main(void)
{

	char data_to_send[] = "Hello world";

	HW_setup();

	// Enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	for(uint8_t i = 0; i < 4; i++)
		SPI_SendData(SPI2, (uint8_t *)data_to_send, strlen(data_to_send));

	while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));

	// Disable the SPI2 peripheral
	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);

	return 0;
}

void delay(uint32_t count)
{
	for(uint32_t i = 0; i < count; i++);
}


