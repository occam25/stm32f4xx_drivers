
#include <stdio.h>
#include <string.h>

#include "stm32f407xx.h"
#include "stm32f407xx_i2c.h"
#include "stm32f407xx_gpio.h"


#define ARDUINO_I2C_ADDR	0x68
#define MY_I2C_ADDRES		0x61

I2C_handle_t i2c_handle;

//uint8_t some_data[] = "We are testing I2C master Tx\n";

volatile uint8_t button_pressed_f;

void delay(uint32_t count);

//extern void initialise_monitor_handles();


void EXTI0_IRQHandler(void)
{
	GPIO_IRQHandling(0);

	button_pressed_f = 1;
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_12);
}

void I2C_GPIO_init(void)
{
	GPIO_handle_t i2c_pins = {0};

	/*
	 * PB8 -> I2C_SCL
	 * PB7 -> I2C_SDA
	 * ALT function mode : 4
	 */

//	GPIO_ClockControl(GPIOB_PORT, ENABLE); //no es necesario, ahora se habilita el clock dentro de GPIO_Init

	i2c_pins.pGPIOx = GPIOB;
	i2c_pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	i2c_pins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	i2c_pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	i2c_pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	i2c_pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	// SCL
	i2c_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_8;
	GPIO_Init(&i2c_pins);

	// SDA
	i2c_pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
	GPIO_Init(&i2c_pins);

}

void GPIO_setup(void)
{
	GPIO_handle_t button = {0};
	GPIO_handle_t LED = {0};

//	GPIO_ClockControl(GPIOA_PORT, ENABLE);
//	GPIO_ClockControl(GPIOD_PORT, ENABLE);

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

	// GPIO IRQ Configurations
//	GPIO_IRQPriorityConfig(IRQ_EXTI0, NVIC_IRQ_PRIO15); // Optional
//	GPIO_IRQInterruptConfig(IRQ_EXTI0, ENABLE);
}

void I2C_init(void)
{

	i2c_handle.pI2Cx = I2C1;
	i2c_handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	i2c_handle.I2C_Config.I2C_DeviceAddress = MY_I2C_ADDRES;
	i2c_handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	i2c_handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;


	I2C_Init(&i2c_handle);
}

void HW_setup(void)
{
	I2C_GPIO_init();
	I2C_init();
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
	int8_t cnt = 0;
	uint8_t msg[64] = {0};

//	initialise_monitor_handles();

	HW_setup();

	I2C_PeripheralControl(I2C1, ENABLE);

	while(1){

		wait_for_button_press();
		cnt++;
		snprintf((char *)msg, sizeof(msg), "Btn pressed. Cnt: %d\n", cnt);
		I2C_MasterSendData(&i2c_handle, msg, strlen((char *)msg), ARDUINO_I2C_ADDR);
//		I2C_MasterSendData(&i2c_handle, some_data, strlen((char *)some_data), ARDUINO_I2C_ADDR);

	}

	I2C_ClockControl(I2C1, DISABLE);
	return 0;
}

void delay(uint32_t count)
{
	for(uint32_t i = 0; i < count; i++);
}


