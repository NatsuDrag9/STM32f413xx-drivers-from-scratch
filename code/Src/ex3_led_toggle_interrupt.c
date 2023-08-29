/*
 * ex3_led_toggle_interrupt.c
 *
 * This program toggles the on-board LED1 when an
 * interrupt is triggered by pressing the on-board
 * button (B1).
 * Interrupt is triggered during falling edge of
 * button press.
 *
 * Implementation
 */

#include "ex3_led_toggle_interrupt.h"

// Software delay
void delay(void) {
	// introduces delay of ~200 ms when system clock is 16 MHz
	for (uint32_t i = 0; i < 500000/2; i++);
}

int main(void) {
	GPIO_Handle_t GPIO_Led, GPIO_Btn;

	// setting initial values of the structure members to 0
	memset(&GPIO_Led, 0, sizeof(GPIO_Led));
	memset(&GPIO_Led, 0, sizeof(GPIO_Btn));

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
	GPIO_Btn.pGPIOx = GPIOC;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIO_Btn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClock_Control(GPIOC, ENABLE);
	GPIO_Init(&GPIO_Btn);

	// GPIO interrupt configuration
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRIORITY15);
 	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);

	while(1);

	return 0;
}

// ISR for the interrupt triggered on button-press
void EXTI15_10_IRQHandler(void) {
	delay();	// debouncing -- wait for the button press to settle to its state
	GPIO_IRQHandler(GPIO_PIN_NO_13);	// clears the pending event on the EXTI115_10 line
	GPIO_TogglePin(GPIOB, GPIO_PIN_NO_0);
}
