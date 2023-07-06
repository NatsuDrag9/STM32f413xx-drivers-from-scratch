/*
 * ex6_i2c_with_arduino.c
 *
 * This program tests the i2c driver with
 * Arduino Uno R3 as slave and STM32F413ZH
 * as master.
 * When the user button on is pressed, master
 * should send data to Arduino board which
 * will be displayed on the Arduino IDE's
 * serial monitor.
 *
 * Configuration:
 * I2C SCL = 100 kHz (Standard Mode)
 * Use 3.3 kohm external pull-up resistor for SDA and SCL lines
 *
 * On STM32F413ZH
 * PB8 -- D15 --> SCL
 * PB9 -- D14 --> SDA
 */

#include "ex7_i2c_with_arduino.h"

// Global variables
I2C_Handle_t I2C1Handle;
uint8_t usr_data[] = "I2C Arduino testing master tx\n";	// max size is 32 bytes -- limitation imposed by Arduinp's wire.h library

// Software delay
void delay(void) {
	for (uint32_t i = 0; i < 500000/2; i++);
}

// User button GPIO initialization
void GPIO_Btn_Init() {
	GPIO_Handle_t GPIO_Btn;

	// user button B1 configuration on PC13
	GPIO_Btn.pGPIOx = GPIOC;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClock_Control(GPIOC, ENABLE);
	GPIO_Init(&GPIO_Btn);
}

// I2C1 GPIO Initialization
void I2C1_GPIOInits() {
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;	// open-drain for I2C
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCL pin
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GPIO_Init(&I2CPins);

	// SDA pin
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&I2CPins);
}

// I2C1 initialization
void I2C1_Inits(){
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_MASTER_ADDR;	// address value is permitted according to I2C specification
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
}

int main(void)
{
	// initializes the user button GPIO pin
	GPIO_Btn_Init();

	// configures GPIO pins to I2C1
	I2C1_GPIOInits();

	// initializes I2C1 configuration
	I2C1_Inits();

	// enable the I2C1 peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	// enable ACK
	I2C_ManageAcking(I2C1, ENABLE);

	while (1) {
		// exits loop on button press
		while (!(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == BTN_PRESSED));
		delay();

		// send data
		I2C_MasterSendData(&I2C1Handle, usr_data, strlen((char*)usr_data), SLAVE_ADDR);
	}

	return 0;
}
