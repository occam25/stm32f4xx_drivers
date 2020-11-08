
#include <stdio.h>
#include <string.h>

#include "stm32f407xx.h"
#include "stm32f407xx_i2c.h"
#include "stm32f407xx_gpio.h"


#define ARDUINO_I2C_ADDR	0x68
#define MY_I2C_ADDRES		0x69

#define I2C_READ_LEN_CMD	0x51
#define I2C_READ_DATA_CMD	0x52

I2C_handle_t i2c_handle;
uint8_t tx_buff[64];
uint8_t rx_buff[64];

uint8_t msg[] = {"Message from STM32\r\n"};


volatile uint8_t button_pressed_f;

void delay(uint32_t count);

//extern void initialise_monitor_handles();

typedef enum i2c_comm_st_e{
	I2C_comm_waiting_for_cmd,
	I2C_comm_process_cmd,
	I2C_comm_send_data_byte,
	I2C_comm_send_waiting_TX_CMPL

}i2c_comm_st_t;

i2c_comm_st_t I2C_comm_st = I2C_comm_waiting_for_cmd;


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

//	i2c_handle.pRxBuffer = &rx_buff;
//	i2c_handle.pTxBuffer = &tx_buff;
//	i2c_handle.RxSize = sizeof(rx_buff);
	i2c_handle.TxRxState = I2C_READY;

	I2C_Init(&i2c_handle);

	I2C_SlaveCallbackEventsControl(I2C1, ENABLE);
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
//	initialise_monitor_handles();

	HW_setup();

	I2C_PeripheralControl(&i2c_handle, ENABLE);

	printf("Initialized\n");

	I2C_IRQInterruptConfig(IRQ_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_I2C1_ER, ENABLE);

	printf("Waiting for commands\n");
	while(1){

	}

	I2C_ClockControl(I2C1, DISABLE);
	return 0;
}


void I2C_ApplicationEventCallback(I2C_handle_t *pI2CHandle, I2C_events_t AppEv)
{

	static uint8_t msg_idx = 0;
	static uint8_t cmd = 0;

	//printf("Event: %d\n", AppEv);

	if(AppEv == I2C_EV_DATA_REQ){
		// Master wants some data. Slave has to send it
		if(cmd == I2C_READ_LEN_CMD){
			// send length to the master
			I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen((char *)msg));
		}else if(cmd == I2C_READ_DATA_CMD){
			// Send data
			I2C_SlaveSendData(pI2CHandle->pI2Cx, msg[msg_idx++]);
		}
	}if(AppEv == I2C_EV_DATA_RCV){
		// Data is waiting for slave to read. Slave has to read it
		cmd = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);
	}else if(AppEv == I2C_ERR_AF){
		// This happends only during slave txing.
		// Master has sent the NACK. Slave should understand that master
		// doesn't want more data
		cmd = 0;
		msg_idx = 0;

	}else if(AppEv == I2C_EV_STOP){
		// This happends only during slave reception.
		// Master has ended the I2C communication with the slave


	}

	if(AppEv == I2C_ERR_ARLO){
		printf("I2C arbitration lost (ARLO)\n");
	}

	if(AppEv == I2C_ERR_BERR){
		printf("I2C Bus error\n");
	}

	if(AppEv == I2C_ERR_OVR){
		printf("Overrun/underrun error\n");
	}

	if(AppEv == I2C_ERR_TOUT){
		printf("I2C Timeout error\n");
	}
}

void I2C1_EV_IRQHandler(void)
{
	I2C_EV_IRQHandling(&i2c_handle);
}

void I2C1_ER_IRQHandler(void)
{
	I2C_ER_IRQHandling(&i2c_handle);
}

void delay(uint32_t count)
{
	for(uint32_t i = 0; i < count; i++);
}


