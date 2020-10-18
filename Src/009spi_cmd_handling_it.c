
#include <stdio.h>
#include <string.h>

#include "stm32f407xx.h"
#include "stm32f407xx_spi.h"
#include "stm32f407xx_gpio.h"


#define CMD_LED_CTRL			0x50
#define CMD_SENSOR_READ			0x51
#define CMD_LED_READ			0x52
#define CMD_PRINT				0x53
#define CMD_ID_READ				0x54

#define LED_ON		1
#define LED_OFF		0

// Arduino analog pins
#define ARDUINO_ANALOG_PIN0			0
#define ARDUINO_ANALOG_PIN1			1
#define ARDUINO_ANALOG_PIN2			2
#define ARDUINO_ANALOG_PIN3			3
#define ARDUINO_ANALOG_PIN4			4
#define ARDUINO_ANALOG_PIN5			5

// Arduino led
#define ARDUINO_LED_PIN		9

SPI_handle_t SPI2_handle;

volatile uint8_t button_pressed_f;

uint8_t Rx_buff[100];
uint8_t Tx_buff[100];

uint8_t state;
uint8_t arduino_led_st = 1;


void delay(uint32_t count);

//extern void initialise_monitor_handles();


void EXTI0_IRQHandler(void)
{
	GPIO_IRQHandling(0);

	button_pressed_f = 1;
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);
}

void SPI2_IRQHandler(void)
{
	SPI_IRQHandling(&SPI2_handle);
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

	// MISO
	spi2_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&spi2_pins);

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
//	GPIO_IRQPriorityConfig(IRQ_EXTI0, NVIC_IRQ_PRIO15); // Optional
//	GPIO_IRQInterruptConfig(IRQ_EXTI0, ENABLE);
}

void SPI2_init(void)
{
	SPI2_handle.pSPIx = SPI2;
	SPI2_handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2_handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2_handle.SPI_Config.SPI_SSM = SPI_SSM_DI; // hardware slave management through NSS pin
	SPI2_handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;  // SCLK of 2MHz
	SPI2_handle.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
	SPI2_handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2_handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;

	SPI2_handle.pRxBuffer = Rx_buff;
	SPI2_handle.RxLen = sizeof(Rx_buff);
	SPI2_handle.RxState = SPI_READY;

	SPI2_handle.pTxBuffer = Tx_buff;
	SPI2_handle.TxLen = sizeof(Tx_buff);
	SPI2_handle.TxState = SPI_READY;

	SPI_Init(&SPI2_handle);

}

void HW_setup(void)
{
	SPI2_GPIO_init();
	SPI2_init();
	GPIO_setup();
}

void wait_for_button_press(void)
{
	//wait till button is pressed
	while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_0) );

	//to avoid button de-bouncing related issues 200ms of delay
	delay(250000);
}

void SPI_ApplicationEventCallback(SPI_handle_t *SPI2_handle,uint8_t AppEv)
{
	switch(AppEv)
	{
	case SPI_EVENT_TX_CMPLT:
		if(state == 0){
			SPI_ReceiveDataIT(SPI2_handle, Rx_buff, 1);
		}

		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);
		break;
	case SPI_EVENT_RX_CMPLT:
		if(state == 0){
			if(Rx_buff[0] == 0xf5){

				Tx_buff[0] = ARDUINO_LED_PIN;
				Tx_buff[1] = arduino_led_st;
				arduino_led_st ^= 1;

				SPI_SendDataIT(SPI2_handle, Tx_buff, 2);
			}
		}
		break;
	default:

		break;
	}
}

int main(void)
{


	uint8_t dummy_write_byte = 0xff;
	uint8_t dummy_read_byte;
	uint8_t cmd_byte;
	uint8_t ACK_byte;
	uint8_t cmd_args[2];

	uint8_t arduino_analog_val = 0;
	uint8_t arduino_led_val = 0;



	uint8_t arduino_ID[11];

//	initialise_monitor_handles();

	HW_setup();

	/*
	 * making SSOE 1 does NSS output enable.
	 * The NSS pin is automatically managed by the hardware.
	 * When SPE = 1, NSS will be pulled to low
	 * When SPE = 0, NSS will be pushed to high
	 */
	SPI_SSOEConfig(SPI2, ENABLE);

	SPI_IRQInterruptConfig(IRQ_SPI2, ENABLE);
//	SPI_IRQPriorityConfig(IRQ_SPI2, 5);
//	SPI_IRQHandling(&SPI2_handle);

	while(1){

			wait_for_button_press();

			// Enable the SPI2 peripheral
			SPI_PeripheralControl(SPI2, ENABLE);

			printf("SPI ENABLED\n");

			switch(state){
			case 0:
				// CMD_LED_CTRL <PIN_number> <State>
				// Send command byte
				Tx_buff[0] = CMD_LED_CTRL;
				SPI_SendDataIT(&SPI2_handle, Tx_buff, 1);
				SPI_ReceiveDataIT(&SPI2_handle, Rx_buff, 1);
				break;
			case 1:
				SPI_ReceiveDataIT(&SPI2_handle, Rx_buff, 1);
				break;
			case 2:
				Tx_buff[0] = ARDUINO_LED_PIN;
				Tx_buff[1] = arduino_led_st;
				arduino_led_st ^= 1;

				SPI_SendDataIT(&SPI2_handle, Tx_buff, 2);
//				SPI_ReceiveDataIT(&SPI2_handle, Rx_buff, 1);


				/////
				// Send dummy byte to fetch the ACK from the slave
//				SPI_SendData(SPI2, &dummy_write_byte, 1); // with this sending the slave response will be received
//				// Read the received byte
//				SPI_ReceiveData(SPI2, &ACK_byte, 1);
//
//				if(ACK_byte == 0xf5){
//					// Send arguments
//					cmd_args[0] = ARDUINO_LED_PIN;
//					cmd_args[1] = arduino_led_st;
//					SPI_SendData(SPI2, cmd_args, 2);
//
//					printf("CMD_LED_CTRL sent: %d\n", arduino_led_st);
//
//					arduino_led_st ^= 1;
//
//				}
				break;
			case 10:
				// CMD_SENSOR_READ <analog pin number>
				// Send command byte
				cmd_byte = CMD_SENSOR_READ;
				SPI_SendData(SPI2, &cmd_byte, 1);
				// This sending will push a garbage byte into the Rx buffer, so we do a dummy read to clear RXNE flag
				SPI_ReceiveData(SPI2, &dummy_read_byte, 1);

				// Send dummy byte to fetch the ACK from the slave
				SPI_SendData(SPI2, &dummy_write_byte, 1); // with this sending the slave response will be received
				// Read the received byte
				SPI_ReceiveData(SPI2, &ACK_byte, 1);

				if(ACK_byte == 0xf5){
					// Send arguments
					cmd_args[0] = ARDUINO_ANALOG_PIN0;
					SPI_SendData(SPI2, cmd_args, 1);
					// dummy read to clear RXNE flag
					SPI_ReceiveData(SPI2, &dummy_read_byte, 1);

					delay(100000);

					// dummy byte to fetch the value
					SPI_SendData(SPI2, &dummy_write_byte, 1);
					SPI_ReceiveData(SPI2, &arduino_analog_val, 1);

					printf("CMD_SENSOR_READ %d\n",arduino_analog_val);
				}
				break;
			case 20:
				// CMD_LED_READ <LED pin number>
				// Send command byte
				cmd_byte = CMD_LED_READ;
				SPI_SendData(SPI2, &cmd_byte, 1);
				// This sending will push a garbage byte into the Rx buffer, so we do a dummy read to clear RXNE flag
				SPI_ReceiveData(SPI2, &dummy_read_byte, 1);

				// Send dummy byte to fetch the ACK from the slave
				SPI_SendData(SPI2, &dummy_write_byte, 1); // with this sending the slave response will be received
				// Read the received byte
				SPI_ReceiveData(SPI2, &ACK_byte, 1);

				if(ACK_byte == 0xf5){
					// Send arguments
					cmd_args[0] = ARDUINO_LED_PIN;
					SPI_SendData(SPI2, cmd_args, 1);
					// dummy read to clear RXNE flag
					SPI_ReceiveData(SPI2, &dummy_read_byte, 1);

					delay(100000);

					// dummy byte to fetch the value
					SPI_SendData(SPI2, &dummy_write_byte, 1);
					SPI_ReceiveData(SPI2, &arduino_led_val, 1);

					printf("CMD_LED_READ %d\n",arduino_led_val);
				}
				break;
			case 30:

				// CMD_PRINT <msg>
				// Send command byte
				cmd_byte = CMD_PRINT;
				SPI_SendData(SPI2, &cmd_byte, 1);
				// This sending will push a garbage byte into the Rx buffer, so we do a dummy read to clear RXNE flag
				SPI_ReceiveData(SPI2, &dummy_read_byte, 1);

				// Send dummy byte to fetch the ACK from the slave
				SPI_SendData(SPI2, &dummy_write_byte, 1); // with this sending the slave response will be received
				// Read the received byte
				SPI_ReceiveData(SPI2, &ACK_byte, 1);

				if(ACK_byte == 0xf5){
					uint8_t msg[] = "Message from Nucleo";
					cmd_args[0] = strlen((char *)msg);

					SPI_SendData(SPI2, cmd_args, 1);
					// This sending will push a garbage byte into the Rx buffer, so we do a dummy read to clear RXNE flag
//					SPI_ReceiveData(SPI2, &dummy_read_byte, 1);

					// Send msg
//					for(uint8_t i = 0; i < msg_len; i++){
						SPI_SendData(SPI2, msg, cmd_args[0]);
						// dummy read to clear RXNE flag
//						SPI_ReceiveData(SPI2, &dummy_read_byte, 1);
//					}

					printf("CMD_PRINT sent\n");

				}
				break;
			case 40:
				// CMD_ID_READ
				// Send command byte
				cmd_byte = CMD_ID_READ;
				SPI_SendData(SPI2, &cmd_byte, 1);
				// This sending will push a garbage byte into the Rx buffer, so we do a dummy read to clear RXNE flag
				SPI_ReceiveData(SPI2, &dummy_read_byte, 1);

				// Send dummy byte to fetch the ACK from the slave
				SPI_SendData(SPI2, &dummy_write_byte, 1); // with this sending the slave response will be received
				// Read the received byte
				SPI_ReceiveData(SPI2, &ACK_byte, 1);

				if(ACK_byte == 0xf5){
					for(uint8_t i = 0;i < 10; i++){
						SPI_SendData(SPI2, &dummy_write_byte, 1);
						SPI_ReceiveData(SPI2, &arduino_ID[i], 1);
					}
					arduino_ID[11] = '\0';

					printf("CMD_ID_READ: %s\n", arduino_ID);
				}
				break;

			}

			while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));

			// Disable the SPI2 peripheral
			SPI_PeripheralControl(SPI2, DISABLE);

		printf("SPI DISABLED\n");

		}
//	}


	return 0;
}

void delay(uint32_t count)
{
	for(uint32_t i = 0; i < count; i++);
}


