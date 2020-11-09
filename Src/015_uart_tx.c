
#include <stdio.h>
#include <string.h>

#include "stm32f407xx.h"
#include "stm32f407xx_usart.h"
#include "stm32f407xx_gpio.h"


USART_handle_t usart2_handle;

uint8_t TxBuff[64];
uint8_t RxBuff[16];

volatile uint8_t button_pressed_f;

void delay(uint32_t count);

//extern void initialise_monitor_handles();


void USART_GPIO_init(void)
{
	GPIO_handle_t usart2_pins = {0};

	/*
	 * PA2 -> USART2_TX
	 * PA3 -> USART2_RX
	 * ALT function mode : 7
	 */

	usart2_pins.pGPIOx = GPIOA;
	usart2_pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usart2_pins.GPIO_PinConfig.GPIO_PinAltFunMode = 7;
	usart2_pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usart2_pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	usart2_pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	// TX
	usart2_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_2;
	GPIO_Init(&usart2_pins);

	// RX
	usart2_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_3;
	GPIO_Init(&usart2_pins);

}

void GPIO_setup(void)
{
	GPIO_handle_t button = {0};
	GPIO_handle_t LED = {0};

	LED.pGPIOx = GPIOD;
	LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	LED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	LED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	LED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	LED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&LED);

	button.pGPIOx = GPIOA;
	button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN; //GPIO_MODE_IT_FT;
	button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;
	button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&button);

}

void USART_init(void)
{

	usart2_handle.pUSARTx = USART2;
	usart2_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	usart2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart2_handle.USART_Config.USART_Mode = USART_MODE_TXRX;
	usart2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	usart2_handle.USART_Config.USART_StopBits = USART_STOPBITS_1;
	usart2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;

	USART_Init(&usart2_handle);

}

void HW_setup(void)
{
	USART_GPIO_init();
	USART_init();
	GPIO_setup();
}

void wait_for_button_press(void)
{
	//wait till button is pressed
	while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_0) );

	//to avoid button de-bouncing related issues 200ms of delay
	delay(250000);
}

int main(void)
{
//	initialise_monitor_handles();

	HW_setup();

	USART_PeripheralControl(usart2_handle.pUSARTx, ENABLE);

//	printf("Initialized\n");

	while(1){
		wait_for_button_press();

		snprintf((char *)TxBuff, sizeof(TxBuff),"Waiting for command...\r\n");
		USART_SendData(&usart2_handle, TxBuff, strlen((char *)TxBuff));

		USART_ReceiveData(&usart2_handle, RxBuff, 1);

		snprintf((char *)TxBuff, sizeof(TxBuff),"Received data: %s\r\n", RxBuff);
		USART_SendData(&usart2_handle, TxBuff, strlen((char *)TxBuff));

		if(RxBuff[0] == 't')
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);

	}

	USART_ClockControl(usart2_handle.pUSARTx, DISABLE);
	return 0;
}



void delay(uint32_t count)
{
	for(uint32_t i = 0; i < count; i++);
}


