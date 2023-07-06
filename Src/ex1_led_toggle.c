/*
 * ex1_led_toggle.c
 *
 *	Toggles the on board LED with some delay using:
 *	(i) push-pull configuration of the output pin
 *	(ii) open drain configuration of the output pin
 *
 *  Created on: Jun 21, 2023
 *      Author: rohitimandi
 */


#include "ex1_led_toggle.h"

// Software delay
void delay(void) {
	for (uint32_t i = 0; i < 500000; i++);
}

int main(void) {
	GPIO_Handle_t GPIO_Led;

	GPIO_Led.pGPIOx = GPIOB;
	GPIO_Led.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIO_Led.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIO_Led.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// push-pull configuration
	GPIO_Led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIO_Led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;	// OPType is push-pull so pull-up/pull-down resistors are not required

/*	// open drain configuration
	GPIO_Led.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	// LED1 does not toggle as in open-drain, no PUPD will not pull-up the LED1 to +VDD
//	GPIO_Led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	// LED1 toggles as in open-drain, PU will pull-up the LED1 to +VDD
	// Intensity is low as the internal pull-up resistor has a large
	// value ~ 40 kohm. For larger intensity, use external pull-up
	// resistor
	GPIO_Led.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;*/

	GPIO_PeriClock_Control(GPIOB, ENABLE);
	GPIO_Init(&GPIO_Led);

	while (1) {
		GPIO_TogglePin(GPIOB, GPIO_PIN_NO_0);
		delay();
	}

	return 0;

}
