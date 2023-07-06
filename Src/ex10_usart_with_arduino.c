/*
 * ex10_usart_with_arduino.c
 *
 * This program tests the usart driver with Arduino Uno
 * as the second device. The program sends some message
 * which will be displayed on Arduino IDE's serial
 * terminal.
 *
 * Configuration:
 * Baudrate: 115200 bps
 * Frame Format: 1 Stop bit, 8 data bits, no parity
 *
 * USART6 is used
 * PG14 -- D1 --> USART6_TX
 * PG9 -- D0 --> USART6_RX
 *
 *
 * Note -- After testing, I found out that Arduino serial monitor displays
 * data for a maximum baudrate of 57600 bps. Any baudrate beyond that is
 * displayed as garbage values
 */

#include "ex10_usart_with_arduino.h"

// global variables
USART_Handle_t USART6Handle;
char someText[1025] = "USART6 testing...\n\r";

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

// USART6 GPIO Initialization
void USART6_GPIOInits() {
	GPIO_Handle_t USARTPins;

	USARTPins.pGPIOx = GPIOG;
	USARTPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	USARTPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	USARTPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	USARTPins.GPIO_PinConfig.GPIO_PinAltFunMode = 8;
	USARTPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// TX pin
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&USARTPins);

	// RX pin
	USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&USARTPins);
}

// USART6 initialization
void USART6_Inits(){
	USART6Handle.pUSARTx = USART6;
	USART6Handle.USART_Config.USART_Baud = USART_STD_BAUD_9600;
	USART6Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	USART6Handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	USART6Handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	USART6Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	USART6Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART6Handle.USART_Config.USART_OverSampling = USART_OVER_SAMPLING_16;

	USART_Init(&USART6Handle);
}


// USART event application callback implementation
//void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEvent){
//
//}

int main(void)
{
	initialise_monitor_handles();

	uint8_t btnPressCount = 0;

	// initializes the user button GPIO pin
	GPIO_Btn_Init();

	// configures GPIO pins to USART6
	USART6_GPIOInits();

	// initializes USART6 configuration
	USART6_Inits();

	// enable the USART6 peripheral
	USART_PeripheralControl(USART6, ENABLE);

	// USART IRQ configuration
//	USART_IRQInterruptConfig(IRQ_NO_USART6_EVT, ENABLE);
//	USART_IRQInterruptConfig(IRQ_NO_USART6_ERR, ENABLE);


	while (1) {
		// exits loop on button press
		while (!(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == BTN_PRESSED));
		// adds a small delay for debouncing
		delay();
		btnPressCount++;
		printf("User pressed button %u\n", btnPressCount);

		// send data
		USART_SendData(&USART6Handle, (uint8_t*)someText, strlen(someText));
		printf("Sent %s\n", someText);

	}

	return 0;
}
