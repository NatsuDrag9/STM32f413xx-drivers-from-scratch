/*
 * ex2_led_toggle_btn_press.c
 *
 * This program toggles the on-board LED1 when
 * the on-board user button is pressed
 *
 */

#include "ex2_led_toggle_btn_press.h"

// Software delay
void delay(void) {
	for (uint32_t i = 0; i < 500000/2; i++);
}

int main(void) {
	GPIO_Handle_t GPIO_Led, GPIO_Btn;

	// LED1 configuration on PB0
	GPIO_Led.pGPIOx = GPIOB;
	GPIO_Led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIO_Led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_Led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_Led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClock_Control(GPIOB, ENABLE);
	GPIO_Init(&GPIO_Led);

	// User button B1 configuration on PC13
	GPIO_Led.pGPIOx = GPIOC;
	GPIO_Led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIO_Led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClock_Control(GPIOC, ENABLE);
	GPIO_Init(&GPIO_Btn);

	while (1) {
		if (GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == BTN_PRESSED) {
			delay();	// // debouncing -- wait for the button press to settle to its state
			GPIO_TogglePin(GPIOB, GPIO_PIN_NO_0);
		}
	}

	return 0;
}
