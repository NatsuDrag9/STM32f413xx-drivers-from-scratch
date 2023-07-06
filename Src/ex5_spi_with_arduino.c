/*
 * ex5_spi_with_Arduino.c
 *
 * This program tests the spi driver with
 * Arduino Uno R3 as slave
 *
 * Configuration:
 * SPI1 in full duplex mode
 * ST board will be SPI master and ARduino will be SPI slave
 * DFF = 0 (8-bit format)
 * Hardware slave management
 * SCLK speed = 2 MHz, fclk (or system clk) = 16 MHz
 *
 */

/*
 * SPI1 pins
 * D13 -- PA5 --> SPI1_SCK
 * D12 -- PA6 --> SPI1_MISO
 * D11 -- PA7 --> SPI1_MOSI
 * D24 -- PA4 --> SPI1_NSS (NSS pin)
 */

#include <ex5_spi_with_arduino.h>

// Software delay
void delay(void) {
	for (uint32_t i = 0; i < 500000/2; i++);
}

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

// SPI1 GPIO Initialization
void SPI1_GPIOInits() {
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOA;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCLK pin
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIO_Init(&SPIPins);

	// MOSI pin
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&SPIPins);

	// MISO pin
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
//	GPIO_Init(&SPIPins);

	// NSS pin configured as output
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_4;
	GPIO_Init(&SPIPins);
}

void SPI1_Inits(void) {
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI1;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_SClkSpeed = SPI_SCLK_SPEED_DIV8;	// generates 2 MHz: 16 MHz/ 8 = 2
	SPI2Handle.SPIConfig.SPI_Dff = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;	// hardware slave management

	SPI_Init(&SPI2Handle);
}

int main(void)
{
	// data to be sent
	char usr_data[] = "Hello World";

	// initializes the user button GPIO pin
	GPIO_Btn_Init();

	// initialize the GPIO pins to SPI1 by setting them in alternate function mode
	SPI1_GPIOInits();

	// initialize the SPI2 peripheral
	SPI1_Inits();

	// NSS pin is set to high by default
	GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_4, GPIO_PIN_SET);

	while (1) {
		// exits loop on button press
		while (!(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == BTN_PRESSED));
		delay();

		// pulling the NSS pin to low
		GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_4, GPIO_PIN_RESET);

		// enable the SPI1 peripheral
		SPI_PeripheralControl(SPI1, ENABLE);

		// first sending no. of bytes
		uint8_t dataLength = strlen(usr_data);
		SPI_SendData(SPI1, &dataLength, sizeof(uint8_t));

		// send data
		SPI_SendData(SPI1, (uint8_t*)usr_data, dataLength);

		// disable the SPI1 peripheral
		SPI_PeripheralControl(SPI1, DISABLE);

		// pulling the NSS pin to high after transmission
		GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_4, GPIO_PIN_SET);
	}

	return 0;
}
