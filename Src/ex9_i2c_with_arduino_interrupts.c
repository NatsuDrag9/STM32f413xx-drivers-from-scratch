/*
 * ex8_i2c_with_arduino.c
 *
* This program tests the i2c driver in interrupt mode
 * with Arduino Uno R3 as slave and STM32F413ZH as
 * master.
 *
 * The test tasks in ex8 are repeated here
 *
 * Configuration:
 * I2C SCL = 100 kHz (Standard Mode)
 * Use internal pull-up resistor for SDA and SCL lines
 *
 * On STM32F413ZH
 * PB8 -- D15 --> SCL
 * PB9 -- D14 --> SDA
 *
 */

#ifndef EX8_I2C_WITH_ARDUINO_C_
#define EX8_I2C_WITH_ARDUINO_C_

// Command codes
#define I2C_CMDCODE_GET_LENGTH	0x51
#define I2C_CMDCODE_GET_DATA	0x52

#include "ex9_i2c_with_arduino_interrupts.h"

// Global variables
I2C_Handle_t I2C1Handle;
uint8_t rxBuffer[32];

// Function prototypes
extern void initialise_monitor_handles(void);

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
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
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

// I2C1 event IRQ handler implementation
void I2C1_EVT_IRQHandler(void){
	I2C_EV_IRQHandler(&I2C1Handle);
}

// I2C1 error IRQ handler implementation
void I2C1_ERR_IRQHandler(void){
	I2C_ER_IRQHandler(&I2C1Handle);
}

// I2C event application callback implementation
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent){
	if(AppEvent == I2C_EV_TX_CMPLT){
		printf("Transmission complete\n");
	}
	else if (AppEvent == I2C_EV_RX_CMPLT){
		printf("Reception complete\n");
	}
	else if (AppEvent == I2C_ERROR_AF){
		printf("Error: ACK failure occurred\n");

		// closing communication
		I2C_CloseSendData(pI2CHandle);

		// generate STOP condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		// hang in infinite loop to prevent further execution
		while(1);
	}
}

int main(void)
{
	initialise_monitor_handles();

	uint8_t commandCode = 0;
	uint8_t rxDataLen = 0;
	uint8_t btnPressCount = 0;

	// initializes the user button GPIO pin
	GPIO_Btn_Init();

	// configures GPIO pins to I2C1
	I2C1_GPIOInits();

	// initializes I2C1 configuration
	I2C1_Inits();

	// enable the I2C1 peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	// I2C IRQ configuration
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EVT, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ERR, ENABLE);

	// enable ACK
	I2C_ManageAcking(I2C1, ENABLE);

	while (1) {
		// exits loop on button press
		while (!(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == BTN_PRESSED));
		// adds a small delay for debouncing
		delay();
		btnPressCount++;
		printf("User pressed button %u\n", btnPressCount);

		// send command to fetch the length of data (in bytes) to be received
		commandCode = I2C_CMDCODE_GET_LENGTH;
		while(I2C_MasterSendDataIT(&I2C1Handle, &commandCode, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);
		printf("Sent command: %u\n", I2C_CMDCODE_GET_LENGTH);
		while(I2C_MasterReceiveDataIT(&I2C1Handle, &rxDataLen, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);
		printf("Data length received from slave: %u\n", rxDataLen);

		// read data from slave
		commandCode = I2C_CMDCODE_GET_DATA;
		while(I2C_MasterSendDataIT(&I2C1Handle, &commandCode, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);
		printf("Sent command: %u\n", I2C_CMDCODE_GET_DATA);
		while(I2C_MasterReceiveDataIT(&I2C1Handle, rxBuffer, rxDataLen, SLAVE_ADDR, I2C_DISABLE_SR) != I2C_READY);
		rxBuffer[rxDataLen+1] = '\0';
		printf("Data received from slave: %s\n", rxBuffer);
	}

	return 0;
}

#endif /* EX8_I2C_WITH_ARDUINO_C_ */
