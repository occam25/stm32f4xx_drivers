
#include "stm32f407xx.h"
#include "stm32f407xx_gpio.h"


//volatile uint8_t button_pressed_f;

void delay(uint32_t count);


void EXTI0_IRQHandler(void)
{
	GPIO_IRQHandling(0);

//	button_pressed_f = 1;
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);
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

void HW_setup(void)
{
	GPIO_setup();
}

int main(void)
{


	HW_setup();


	while(1){
//		if(button_pressed_f){
//			button_pressed_f = 0;
//			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);
//			delay(500000);
//		}
	}

	return 0;
}

void delay(uint32_t count)
{
	for(uint32_t i = 0; i < count; i++);
}
