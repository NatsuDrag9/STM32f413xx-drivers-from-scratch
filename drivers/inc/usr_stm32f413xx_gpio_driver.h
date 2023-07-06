/*
 * usr_stm32f413xx_gpio.h
 *
 *	This header file contains the macro definitions and
 *	function declarations used to implement the driver
 *	for GPIO pins.
 *
 *
 */

#ifndef INC_USR_STM32F413XX_GPIO_H_
#define INC_USR_STM32F413XX_GPIO_H_

#include "usr_stm32f413xx.h"

/*
 * GPIO pin configuration structure
 */
typedef struct {
	uint8_t GPIO_PinNumber;			// possible values from @GPIO_PIN_NUMBERS
	uint8_t GPIO_PinMode;			// possible values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;			// possible values from @GPIO_OP_SPEED
	uint8_t GPIO_PinPuPdControl;	// possible values from @GPIO_PUPD
	uint8_t GPIO_PinOPType;			// possible values from @GPIO_OP_TYPE
	uint8_t GPIO_PinAltFunMode;
} GPIO_PinConfig_t;


/*
 * @GPIO_PIN_NUMBERS
 * Possible vales of GPIO pin numbers
 */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15

/*
 * @GPIO_PIN_MODES
 * Possible modes of a GPIO pin
  */
#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

/*
 * @GPIO_OP_TYPE
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP		0
#define GPIO_OP_TYPE_OD		1

/*
 * @GPIO_OP_SPEED
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MEDIUM	1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

/*
 * @GPIO_PUPD
 * GPIO pin pupd configuration
 */
#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2

/*
 * Handle structure for GPIO pin
 */
typedef struct {
	GPIO_RegDef_t *pGPIOx;				// pointer holds the base address of GPIO peripheral
	GPIO_PinConfig_t GPIO_PinConfig;	// holds GPIO pin configuration settings

} GPIO_Handle_t;

/* GPIO API prototypes */
/* GPIO peripheral clock configuration */
void GPIO_PeriClock_Control(GPIO_RegDef_t* pGPIOx, uint8_t EnorDi);

/* GPIO init and de-init */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/* GPIO read and write */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_TogglePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/* GPIO interrupts */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint32_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandler(uint8_t PinNumber);

#endif /* INC_USR_STM32F413XX_GPIO_H_ */
