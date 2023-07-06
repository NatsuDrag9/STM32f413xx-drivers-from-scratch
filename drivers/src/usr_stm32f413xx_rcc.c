/*
 * usr_stm32f413xx_rcc.c
 *
 * Implements APIs to compute peripheral bus
 * clock
 *
 */
#include "usr_stm32f413xx_rcc.h"

/* Global variables */
uint16_t AHB_PreScalar[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint16_t APB1_PreScalar[4] = {2, 4, 8, 16};
uint16_t APB2_PreScalar[4] = {2, 4, 8, 16};

/* Not implemented */
uint32_t RCC_GetPLLOutputClock(void) {
	uint32_t pllClock = 0;

	return pllClock;
}

/*
 * @func_name				- RCC_GetPCLK1Value
 *
 * @brief					- computes APB1 peripheral clock value
 *
 * @param1					- none
 *
 * @return					- returns APB1 peripheral clock value - uint32_t
 *
 * @Note					- The process of computing pclk1 is taken from
 *							- the clock block diagram in STM32F4XX_RM
 *
 */
uint32_t RCC_GetPCLK1Value(void){
	uint32_t pclk1 = 0, systemClk = 0;
	uint8_t clkSource = 0, temp = 0, ahbPrescalar = 0, apb1Prescalar = 0;

	clkSource = ((RCC->CFGR >> 2) & 0x03);

	if (clkSource == 0) {
		systemClk = 16000000;	// HSI ==> 16 MHz
	}
	else if (clkSource == 1) {
		systemClk = 8000000;	// HSE ==> 8 MHz
	}
	else if (clkSource == 2)
	{
		systemClk = RCC_GetPLLOutputClock();
	}

	// gets AHB Prescalar
	temp = ((RCC->CFGR >> 4) & 0xF);
	if (temp < 8) {
		ahbPrescalar = 1;
	}
	else {
		ahbPrescalar = AHB_PreScalar[temp-8];
	}

	// gets APB1 Prescalar
	temp = ((RCC->CFGR >> 10) & 0x07);
	if (temp < 4) {
		apb1Prescalar = 1;
	}
	else {
		apb1Prescalar = APB1_PreScalar[temp-4];
	}

	pclk1 = (systemClk / ahbPrescalar) / apb1Prescalar;

	return pclk1;
}

/*
 * @func_name				- RCC_GetPCLK2Value
 *
 * @brief					- computes APB2 peripheral clock value
 *
 * @param1					- none
 *
 * @return					- returns APB2 peripheral clock value - uint32_t
 *
 * @Note					- The process of computing pclk1 is taken from
 *							- the clock block diagram in STM32F4XX_RM
 *
 */
uint32_t RCC_GetPCLK2Value(void){
	uint32_t pclk2 = 0, systemClk = 0;
	uint8_t clkSource = 0, temp = 0, ahbPrescalar = 0, apb2Prescalar = 0;

	clkSource = ((RCC->CFGR >> 2) & 0x03);

	if (clkSource == 0) {
		systemClk = 16000000;	// HSI ==> 16 MHz
	}
	else if (clkSource == 1) {
		systemClk = 8000000;	// HSE ==> 8 MHz
	}
	else if (clkSource == 2)
	{
		systemClk = RCC_GetPLLOutputClock();
	}

	// gets AHB Prescalar
	temp = ((RCC->CFGR >> 4) & 0xF);
	if (temp < 8) {
		ahbPrescalar = 1;
	}
	else {
		ahbPrescalar = AHB_PreScalar[temp-8];
	}

	// gets APB2 Prescalar
	temp = ((RCC->CFGR >> 13) & 0x07);
	if (temp < 4) {
		apb2Prescalar = 1;
	}
	else {
		apb2Prescalar = APB2_PreScalar[temp-4];
	}

	pclk2 = (systemClk / ahbPrescalar) / apb2Prescalar;

	return pclk2;
}
