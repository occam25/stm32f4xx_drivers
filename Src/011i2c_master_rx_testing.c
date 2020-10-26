
#include <stdio.h>
#include <string.h>

#include "stm32f407xx.h"
#include "stm32f407xx_i2c.h"
#include "stm32f407xx_gpio.h"


#define ARDUINO_I2C_ADDR	0x68
#define MY_I2C_ADDRES		0x61

#define I2C_READ_LEN_CMD	0x51
#define I2C_READ_DATA_CMD	0x52

I2C_handle_t i2c_handle;

volatile uint8_t button_pressed_f;

void delay(uint32_t count);

extern void initialise_monitor_handles();


void I2C_GPIO_init(void)
{
	GPIO_handle_t i2c_pins = {0};

	/*
	 * PB8 -> I2C_SCL
	 * PB7 -> I2C_SDA
	 * ALT function mode : 4
	 */

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
	uint8_t cmd;
	uint8_t data_len;
	uint8_t data[64] = {0};

	initialise_monitor_handles();

	HW_setup();

	I2C_PeripheralControl(&i2c_handle, ENABLE);

	printf("Initialized\n");

	while(1){
		printf("Waiting for the button to be pressed\n");
		wait_for_button_press();

		// 1. Send I2C_READ_LEN_CMD command
		cmd = I2C_READ_LEN_CMD;
		I2C_MasterSendData(&i2c_handle, &cmd, 1, ARDUINO_I2C_ADDR);

		// 2. Read arduino's data len
		I2C_MasterReceiveData(&i2c_handle, &data_len, 1, ARDUINO_I2C_ADDR);

		if(data_len > (sizeof(data) - 1))
			data_len = sizeof(data) - 1;

		// 3. Send I2C_READ_DATA_CMD command
		cmd = I2C_READ_DATA_CMD;
		I2C_MasterSendData(&i2c_handle, &cmd, 1, ARDUINO_I2C_ADDR);

		// 4. Read arduino's data
		I2C_MasterReceiveData(&i2c_handle, data, data_len, ARDUINO_I2C_ADDR);

		data[data_len] = '\0';
		printf("Data from Arduino: %s\n", data);
	}

	I2C_ClockControl(I2C1, DISABLE);
	return 0;
}

void delay(uint32_t count)
{
	for(uint32_t i = 0; i < count; i++);
}


