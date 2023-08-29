/*
 * udemy_stm32f413xx.c
 *
 * Driver implementation of GPIO pins.
 *
 */

#include "usr_stm32f413xx_gpio_driver.h"

/* GPIO peripheral clock configuration */
/*
 * @func_name				- GPIO_PeriClock_Control
 *
 * @brief					- enables/controls the GPIO port's PCLK
 *
 * @param1					- base address of the GPIO peripheral
 * @param2					- enable or disable macros
 *
 * @return					- none
 *
 * @Note					- none
 *
 */
void GPIO_PeriClock_Control(GPIO_RegDef_t* pGPIOx, uint8_t EnorDi){
	if (EnorDi == ENABLE) {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_EN();
		}
		else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_EN();
		}
		else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_EN();
		}
		else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_EN();
		}
		else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_EN();
		}
		else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_EN();
		}
		else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_EN();
		}
		else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_EN();
		}
	}
	else {
		if (pGPIOx == GPIOA) {
			GPIOA_PCLK_DI();
		}
		else if (pGPIOx == GPIOB) {
			GPIOB_PCLK_DI();
		}
		else if (pGPIOx == GPIOC) {
			GPIOC_PCLK_DI();
		}
		else if (pGPIOx == GPIOD) {
			GPIOD_PCLK_DI();
		}
		else if (pGPIOx == GPIOE) {
			GPIOE_PCLK_DI();
		}
		else if (pGPIOx == GPIOF) {
			GPIOF_PCLK_DI();
		}
		else if (pGPIOx == GPIOG) {
			GPIOG_PCLK_DI();
		}
		else if (pGPIOx == GPIOH) {
			GPIOH_PCLK_DI();
		}
	}
}



/* GPIO init */
/*
 * @func_name				- GPIO_Init
 *
 * @brief					- initializes the GPIO port and pin
 *
 * @param1					- pointer to GPIO_Handle_t
 *
 * @return					- none
 *
 * @Note					- none
 *
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	uint32_t temp = 0;	// temporary register
	uint8_t pinNumber = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	uint8_t pinMode = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode;
	uint8_t pinSpeed = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed;
	uint8_t pupdControl = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl;
	uint8_t opType = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType;
	uint8_t altFunMode = pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode;

	// Enabling GPIO PCLK
	GPIO_PeriClock_Control(pGPIOHandle->pGPIOx, ENABLE);

	// configure the mode of the GPIO pin
	if (pinMode <= GPIO_MODE_ANALOG) {
		// non-interrupt mode
		temp = pinMode << (2*pinNumber);	// each pin takes two bit fields
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2*pinNumber));	// clears bit fields
		pGPIOHandle->pGPIOx->MODER |= temp;	// sets the bit fields
	}
	else {
		// interrupt mode
		if(pinMode == GPIO_MODE_IT_FT) {
			// configure interrupt in FTSR
			EXTI->FTSR |= (1 << pinNumber);
			EXTI->RTSR &= ~(1 << pinNumber);	// clears the corresponding RTSR bit
		}
		else if (pinMode == GPIO_MODE_IT_RT) {
			// configure interrupt in RTSR
			EXTI->RTSR |= (1 << pinNumber);
			EXTI->FTSR &= ~(1 << pinNumber);	// clears the corresponding FTSR bit
		}
		else if (pinMode == GPIO_MODE_IT_RFT) {
			// configure interrupt during both RTSR and FTSR
			EXTI->FTSR |= (1 << pinNumber);
			EXTI->RTSR |= (1 << pinNumber);
		}

		// configure the GPIO port in SYSCFG_EXTICRx (x = 1, 2, 3, 4.)
		uint8_t temp1 = 0, temp2 = 0;
		uint8_t portCode = GPIO_BASEADDR_TO_PORT_CODE(pGPIOHandle->pGPIOx);
		temp1 = pinNumber/4;
		temp2 = pinNumber%4;
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] |= portCode << (4*temp2);

		// enable the EXTI interrupt delivery using interrupt mask register (IMR)
		EXTI->IMR |= (1 << pinNumber);
	}

	temp = 0;

	// configure the speed
	temp = pinSpeed << (2*pinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2*pinNumber));	// clears bit fields
	pGPIOHandle->pGPIOx->OSPEEDR |= temp; // sets the bit fields

	temp = 0;

	// configure the pupd settings
	temp = pupdControl << (2*pinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2*pinNumber));	// clears bit fields
	pGPIOHandle->pGPIOx->PUPDR |= temp; // sets the bit fields

	temp = 0;

	// configure the optype
	temp = opType << pinNumber;
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pinNumber);	// clears bit fields
	pGPIOHandle->pGPIOx->OTYPER |= temp; // sets the bit fields

	temp = 0;

	if (pinMode == GPIO_MODE_ALTFN){
		// configure the alt functions
		uint8_t temp1 = 0, temp2 = 0;
		temp1 = pinNumber/8;
		temp2 = pinNumber%8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4*pinNumber));	// clears bit fields
		pGPIOHandle->pGPIOx->AFR[temp1] |= altFunMode << (4*temp2);	// sets the bit fields
	}
}



/* GPIO de-init */
/*
 * @func_name				- GPIO_DeInit
 *
 * @brief					- de-initializes the GPIOx registers by resetting it to default state
 *
 * @param1					- pointer to GPIO port
 *
 * @return					- none
 *
 * @Note					- none
 *
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {
	if (pGPIOx == GPIOA) {
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOB) {
		GPIOB_REG_RESET();
	}
	else if (pGPIOx == GPIOC) {
		GPIOC_REG_RESET();
	}
	else if (pGPIOx == GPIOD) {
		GPIOD_REG_RESET();
	}
	else if (pGPIOx == GPIOE) {
		GPIOE_REG_RESET();
	}
	else if (pGPIOx == GPIOF) {
		GPIOF_REG_RESET();
	}
	else if (pGPIOx == GPIOG) {
		GPIOG_REG_RESET();
	}
	else if (pGPIOx == GPIOH) {
		GPIOH_REG_RESET();
	}
}

/* GPIO read pin */
/*
 * @func_name				- GPIO_ReadFromInputPin
 *
 * @brief					- reads from a given GPIO pin
 *
 * @param1					- GPIO port of the given GPIO pin
 * @param2					- GPIO pin number
 *
 * @return					- pin value
 *
 * @Note					- none
 *
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t value = 0;
	value = (uint8_t) ((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}


/* GPIO read from port */
/*
 * @func_name				- GPIO_ReadFromInputPort
 *
 * @brief					- reads from a given GPIO port
 *
 * @param1					- GPIO port of the given GPIO pin
 *
 * @return					- values read from all 16 pins
 *
 * @Note					- none
 *
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){
	uint16_t value = 0;
	value = (uint8_t) pGPIOx->IDR;
	return value;
}

/* GPIO write */
/*
 * @func_name				- GPIO_WriteToOutputPin
 *
 * @brief					- writes to a GPIO pin
 *
 * @param1					- GPIO port of the given GPIO pin
 * @param2					- GPIO pin number
 * @param3					- 8-bit value to be written to the given GPIO pin
 *
 * @return					- none
 *
 * @Note					- none
 *
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){
	if (Value == GPIO_PIN_SET) {
		// write 1
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else {
		// write 0
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}


/* GPIO write to port */
/*
 * @func_name				- GPIO_WriteToOutputPort
 *
 * @brief					- writes to a given GPIO port
 *
 * @param1					- GPIO port of the given GPIO pin
 * @param3					- 16-bit value to be written to the given GPIO port
 *
 * @return					- none
 *
 * @Note					- none
 *
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value){
	pGPIOx->ODR = Value;
}

/* GPIO toggle pin */
/*
 * @func_name				- GPIO_TogglePin
 *
 * @brief					- toggles the state of GPIO pin
 *
 * @param1					- GPIO port of the given GPIO pin
 * @param3					- GPIO pin number
 *
 * @return					- none
 *
 * @Note					- none
 *
 */
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^= (1 << PinNumber);
}

/* GPIO interrupt configuration */
/*
 * @func_name				- GPIO_IRQConfig
 *
 * @brief					- sets the GPIO interrupt configuration in NVIC registers.
 *							  Details about the NVIC registers are mentioned in pg-220 of
 *							  Cortex M4 user guide. The range of IRQNumbers (from 0 to 101)
 *							  is taken from table 40 from pg 250-253 in STM32F413_RM
 *
 * @param1					- interrupt number
 * @param3					- enable/disable macro
 *
 * @return					- none
 *
 * @Note					- none
 *
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
	if (EnorDi == ENABLE) {
		if (IRQNumber <= 31) {
			// program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber <= 63) {
			// program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));	// 32 to 64
		}
		else if (IRQNumber > 63 && IRQNumber <= 95) {
			// program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 64));	// 64 to 96
		}
		else if (IRQNumber > 95 && IRQNumber <= 101 ) {
			// program ISER3 register
			*NVIC_ISER3 |= (1 << (IRQNumber % 96));	// 96 to 128
		}
	}
	else {
		if (IRQNumber <= 31) {
			// program ISER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber <= 63) {
			// program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));	// 32 to 64
		}
		else if (IRQNumber > 63 && IRQNumber <= 95) {
			// program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 64));	// 64 to 96
		}
		else if (IRQNumber > 95 && IRQNumber <= 101 ) {
			// program ICER3 register
			*NVIC_ICER3 |= (1 << (IRQNumber % 96));	// 96 to 128
		}
	}
}

/* GPIO interrupt priority configuration */
/*
 * @func_name				- GPIO_IRQConfig
 *
 * @brief					- sets the GPIO interrupt priority configuration in NVIC
 * 							  IPRx registers. Details about the NVIC IPRx registers
 * 							  are mentioned in pg-220 of Cortex M4 user guide.
 * 							  The shift_amount is computed according to the steps
 * 							  mentioned in pg-224 of Cortex M4 user guide.
 *
 * @param1					- interrupt number
 * @param2					- interrupt priority
 *
 * @return					- none
 *
 * @Note					- none
 *
 */
void GPIO_IRQPriorityConfig(uint32_t IRQNumber, uint8_t IRQPriority) {
	// finding the x in IPRx register (x = 0 to 59)
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8*iprx_section) + (8-NO_PR_BITS_IMPLEMENTED);
	*(NVIC_IPRx_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}

/* GPIO interrupts handler */
/*
 * @func_name				- GPIO_IRQHandler
 *
 * @brief					- clears the EXTI PR register corresponding to the pin number
 *
 * @param1					- GPIO pin on which the interrupt occurred
 *
 * @return					- none
 *
 * @Note					- none
 *
 */
void GPIO_IRQHandler(uint8_t PinNumber) {

	if (EXTI->PR & (1 << PinNumber)) {
		// clear the bit
		EXTI->PR |= (1 << PinNumber);
	}
}
