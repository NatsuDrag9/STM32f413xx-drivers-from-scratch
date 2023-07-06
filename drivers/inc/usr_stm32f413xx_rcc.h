/*
 * usr_stm32f413xx_rcc.h
 *
 *  Created on: Jul 2, 2023
 *      Author: rohitimandi
 */

#ifndef INC_USR_STM32F413XX_RCC_H_
#define INC_USR_STM32F413XX_RCC_H_

#include "usr_stm32f413xx.h"

/* Prototypes */
uint32_t RCC_GetPLLOutputClock(void);
uint32_t RCC_GetPCLK1Value(void);
uint32_t RCC_GetPCLK2Value(void);

#endif /* INC_USR_STM32F413XX_RCC_H_ */
