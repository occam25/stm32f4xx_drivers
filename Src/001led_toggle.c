
#include "stm32f407xx.h"
#include "stm32f407xx_gpio.h"


void delay(uint32_t count);

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
	button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_0;

	GPIO_Init(&button);

}

void HW_setup(void)
{
	GPIO_setup();
}

int main(void)
{


	HW_setup();



	while(1){
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_0)){
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);
			delay(500000);
		}else
			GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);

	}

	return 0;
}

void delay(uint32_t count)
{
	for(uint32_t i = 0; i < count; i++);
}
