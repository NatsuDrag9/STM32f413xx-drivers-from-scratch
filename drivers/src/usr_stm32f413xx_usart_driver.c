/*
 * usr_stm32f413xx_usart_driver.c
 *
 * This file implements the USART driver for STM32F413XX
 */


#include "usr_stm32f413xx_usart_driver.h"

/* Driver functions implementation */
/*
 * @func_name				- USART_PeriClock_Control
 *
 * @brief					- enables/controls the specified USART peripheral's PCLK
 *
 * @param1					- base address of the USART peripherals whose clock has to be enabled
 * @param2					- enable or disable macros
 *
 * @return					- none
 *
 * @Note					- none
 *
 */
void USART_PeriClock_Control(USART_RegDef_t* pUSARTx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pUSARTx == USART1) {
			USART1_PCLK_EN();
		}
		else if (pUSARTx == USART2) {
			USART2_PCLK_EN();
		}
		else if (pUSARTx == USART3) {
			USART3_PCLK_EN();
		}
		else if (pUSARTx == USART6) {
			USART6_PCLK_EN();
		}
		else if (pUSARTx == UART4){
			UART4_PCLK_EN();
		}
		else if (pUSARTx == UART5){
			UART5_PCLK_EN();
		}
		else if (pUSARTx == UART7){
			UART7_PCLK_EN();
		}
		else if (pUSARTx == UART8){
			UART8_PCLK_EN();
		}
		else if (pUSARTx == UART9){
			UART9_PCLK_EN();
		}
		else if (pUSARTx == UART10){
			UART10_PCLK_EN();
		}
	}
	else {
		if (pUSARTx == USART1) {
			USART1_PCLK_DI();
		}
		else if (pUSARTx == USART2) {
			USART2_PCLK_DI();
		}
		else if (pUSARTx == USART3) {
			USART3_PCLK_DI();
		}
		else if (pUSARTx == USART6) {
			USART6_PCLK_DI();
		}
		else if (pUSARTx == UART4){
			UART4_PCLK_DI();
		}
		else if (pUSARTx == UART5){
			UART5_PCLK_DI();
		}
		else if (pUSARTx == UART7){
			UART7_PCLK_DI();
		}
		else if (pUSARTx == UART8){
			UART8_PCLK_DI();
		}
		else if (pUSARTx == UART9){
			UART9_PCLK_DI();
		}
		else if (pUSARTx == UART10){
			UART10_PCLK_DI();
		}
	}
}

/* Returns whether the specified flag in SR is set or not */
uint8_t USART_GetFlagStatus(USART_RegDef_t* pUSARTx, uint32_t FlagName) {
	if (pUSARTx->SR & FlagName) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/* Clears the specified flag in SR */
void USART_ClearFlag(USART_RegDef_t* pUSARTx, uint32_t FlagName){
	pUSARTx->SR &= ~FlagName;
}



/*
 * @func_name				- USART_SetBaudRate
 *
 * @brief					- resolves the baudrate and computes the USART DIV value
 * 							- for the specified USART peripheral
 *
 * @param1					- USARTx peripheral base address
 * @param2					- baud rate
 * @param3					- no. of bytes to transmit
 *
 * @return					- none
 *
 * @Note					- The formula to calculate USART DIV value for BRR is
 * 							- taken from STM32F413XX_RM
 *
 */
void USART_SetBaudRate(USART_RegDef_t* pUSARTx, uint32_t BaudRate){
	// variable to hold the APB clock value
	uint32_t pClkx = 0;
	// variable to hold the usart prescalar
	uint32_t usartDiv = 0;
	// variable to hold mantissa and fraction values
	uint32_t mantissa = 0, fraction = 0;
	// temporary register
	uint32_t tempreg = 0;

	if ((pUSARTx == USART2) || (pUSARTx == USART3) || (pUSARTx == UART7) || (pUSARTx == UART8)){
		// The peripheral bus is APB1 so fetching its clock
		pClkx = RCC_GetPCLK1Value();
	}
	else if ((pUSARTx == USART1) || (pUSARTx == USART6) || (pUSARTx == UART4) ||
			(pUSARTx == UART5) || (pUSARTx == UART9) || (pUSARTx == UART10)){
		// The peripheral bus is APB2 so fetching its clock
		pClkx = RCC_GetPCLK2Value();
	}

	// check for oversampling configuration
	if ((pUSARTx->CR1 & (1 << USART_CR1_OVER8)) == USART_OVER_SAMPLING_8) {
		// oversampling by 8
		usartDiv = pClkx / (8 * (2 - USART_OVER_SAMPLING_8) * BaudRate);

		// multiplying by 100 to make it into a whole number
		usartDiv = usartDiv * 100;
	}
	else if ((pUSARTx->CR1 & (1 << USART_CR1_OVER8)) == USART_OVER_SAMPLING_16) {
		// oversampling by 16
		usartDiv = pClkx / (8 * (2 - USART_OVER_SAMPLING_16) * BaudRate);

		// multiplying by 100 to make it into a whole number
		usartDiv = usartDiv * 100;
	}

	// calculate mantissa part
	mantissa = usartDiv / 100;
	// load mantissa into 11:0 bit fields of BRR
	tempreg |= mantissa << 4;

	// calculate fractional part
	fraction = usartDiv - mantissa * 100;
	if((pUSARTx->CR1 & (1 << USART_CR1_OVER8)) == USART_OVER_SAMPLING_8){
		// multiplying by 8 because oversampling is by 8
		// adding 50 to round up
		fraction = ((((fraction * 8) + 50) / 100) & (uint8_t)0x07);
	}
	else {
		// multiplying by 16 because oversampling is by 16
		// adding 50 to round up
		fraction = ((((fraction * 16) + 50) / 100) & (uint8_t)0x0F);
	}

	// loading the fraction in bit fields 3:0 of BRR
	tempreg |= fraction;

	// assigning tempreg to BRR
	pUSARTx->BRR = tempreg;
}

/*
 * @func_name				- USART_Init
 *
 * @brief					- initializes the USART peripheral by setting its configuration
 *
 * @param1					- pointer to USART handle to set the USART configuration
 *
 * @return					- none
 *
 * @Note					-
 *
 */
void USART_Init(USART_Handle_t *pUSARTHandle){
	uint32_t tempreg = 0;
	uint8_t mode = pUSARTHandle->USART_Config.USART_Mode;
	uint32_t baud = pUSARTHandle->USART_Config.USART_Baud;
	uint8_t parityControl = pUSARTHandle->USART_Config.USART_ParityControl;
	uint8_t noOfStopBits = pUSARTHandle->USART_Config.USART_NoOfStopBits;
	uint8_t hwFlowControl = pUSARTHandle->USART_Config.USART_HWFlowControl;
	uint8_t wordLength = pUSARTHandle->USART_Config.USART_WordLength;
	uint8_t overSampling = pUSARTHandle->USART_Config.USART_OverSampling;

	// enable USART peripheral clock
	USART_PeriClock_Control(pUSARTHandle->pUSARTx, ENABLE);

	// Enable USART Tx and RX blocks according the mode
	// provided by the application layer
	if(mode == USART_MODE_ONLY_TX){
		// set TE bit in CR1
		tempreg |= (1 << USART_CR1_TE);
	}
	else if (mode == USART_MODE_ONLY_RX){
		// set RE bit in CR1
		tempreg |= (1 << USART_CR1_RE);
	}
	else if (mode == USART_MODE_TXRX){
		// set TE and RE bits;in CR1
		tempreg |= ((1 << USART_CR1_TE) | (1 << USART_CR1_RE));
	}

	// configure the word length in CR1
	tempreg |= wordLength << (USART_CR1_M);

	// configure parity control and parity selection bits in CR1
	if (parityControl == USART_PARITY_EN_EVEN){
		// set PCE bit in CR1
		tempreg |= (1 << USART_CR1_PCE);

		// clear PS bit in CR1
		tempreg &= ~(1 << USART_CR1_PS);
	}
	else if (parityControl == USART_PARITY_EN_ODD){
		// set PCE bit in CR1
		tempreg |= (1 << USART_CR1_PCE);

		// set PS bit in CR1
		tempreg |= (1 << USART_CR1_PS);
	}
	else if (parityControl == USART_PARITY_DISABLE){
		// clear PCE bit in CR1
		tempreg &= ~(1 << USART_CR1_PCE);
	}

	// configure over smapling mode
	tempreg |= (overSampling << USART_CR1_OVER8);

	// assign the value of tempreg to CR1
	pUSARTHandle->pUSARTx->CR1 = tempreg;

	/* configuring CR2 register */
	tempreg = 0;

	// configure the no. of stop bits in CR2
	tempreg |= (noOfStopBits << USART_CR2_STOP);

	// assign the value of tempreg to CR2
	pUSARTHandle->pUSARTx->CR2 = tempreg;

	/* configure CR3 */
	tempreg = 0;

	// configure USART hardware flow control
	if (hwFlowControl == USART_HW_FLOW_CTRL_NONE){
		// clear CTSE bit in CR3
		tempreg &= ~(1 << USART_CR3_CTSE);
		tempreg &= ~(1 << USART_CR3_RTSE);
	}
	else if (hwFlowControl == USART_HW_FLOW_CTS){
		// set CTSE bit in CR3
		tempreg |= (1 << USART_CR3_CTSE);
	}
	else if (hwFlowControl == USART_HW_FLOW_RTS){
		// set RTSE bit in CR3
		tempreg |= (1 << USART_CR3_RTSE);
	}
	else if (hwFlowControl == USART_HW_FLOW_CTS_RTS){
		// set CTSE and RTSE bits in Cr3
		tempreg |= (1 << USART_CR3_CTSE);
		tempreg |= (1 << USART_CR3_RTSE);
	}

	// assign the value of tempreg to CR3
	pUSARTHandle->pUSARTx->CR3 = tempreg;

	/* configure baud rate in BRR register */
	USART_SetBaudRate(pUSARTHandle->pUSARTx, baud);
}

/*
 * @func_name				- USART_DeInit
 *
 * @brief					- de-initializes the specified USART peripheral by resetting to default state
 *
 * @param1					- base address of the USART peripheral which needs to be de-initialized
 *
 * @return					- none
 *
 * @Note					- none
 *
 */
void USART_DeInit(USART_RegDef_t *pUSARTx) {
	if (pUSARTx == USART1) {
		USART1_REG_RESET();
	}
	else if (pUSARTx == USART2) {
		USART2_REG_RESET();
	}
	else if (pUSARTx == USART3) {
		USART3_REG_RESET();
	}
	else if (pUSARTx == USART6){
		USART6_REG_RESET();
	}
	else if (pUSARTx == UART4){
		UART4_REG_RESET();
	}
	else if (pUSARTx == UART5){
		UART5_REG_RESET();
	}
	else if (pUSARTx == UART7){
		UART7_REG_RESET();
	}
	else if (pUSARTx == UART8){
		UART8_REG_RESET();
	}
	else if (pUSARTx == UART9){
		UART9_REG_RESET();
	}
	else if (pUSARTx == UART10){
		UART10_REG_RESET();
	}
}

/*
 * @func_name				- USART_PeripheralControl
 *
 * @brief					- enable/disable USARTx peripheral before the communication begins by
 * 							  setting/resetting the PE bit in CR1 register
 *
 * @param1					- base address of the USART peripheral from which data is being sent
 * @param2					- enables or disable
 *
 * @return					- none
 *
 * @Note					-
 *
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi){
	if (EnorDi == ENABLE) {
		// sets the UE bit
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	}
	else {

		// clears UE bit
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}
}



/*
 * @func_name				- USART_SendData
 *
 * @brief					- sends data via the specified USART peripheral
 *
 * @param1					- USARTx peripheral handle from which data is being sent
 * @param2					- pointer to the transmitter buffer in which the data being sent is stored
 * @param3					- no. of bytes to transmit
 *
 * @return					- none
 *
 * @Note					- this is blocking call (polling) as the function does not return until
 * 							  all the length no. of bytes are not transferred
 *
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t length){
	uint16_t *pData;

	for (uint32_t i = 0; i < length; i++){
		// wait  until TXE is set to 1
		while(!(USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TXE)));

		if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS){
			pData = (uint16_t*)pTxBuffer;
			// load only the first 9 bits in pData into DR by masking
			// the remaining bits
			pUSARTHandle->pUSARTx->DR = (*pData & (uint16_t)0x01FF);

			// check for the parity control
			if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
				// no parity bit is used so all 9 bits are data
				// so increment the buffer twice
				pTxBuffer = pTxBuffer + 2;
			}
			else {
				// 9th bit is parity bit which is attached to the
				// data by hardware. So, only first 8 bits are data bits.
				// Increment the buffer only once
				pTxBuffer++;
			}
		}
		else {
			// 8-bit data so load data directly into DR
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer & (uint8_t)0xFF);

			// increment the buffer address
			pTxBuffer++;
		}

		// wait until the TC is set -- indicates that transmission is complete
		while(!(USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC)));
	}
}

/*
 * @func_name				- USART_ReceiveData
 *
 * @brief					- receives data from the specified USART peripheral
 *
 * @param1					- handle to the USART peripheral from which data is being sent
 * @param2					- pointer to the receiver buffer in which the data being sent is stored
 * @param3					- length of the data to be sent in bytes
 *
 * @return					- none
 *
 * @Note					- this is blocking call (polling) as the function does not return until
 * 							  all the length no. of bytes are not transferred
 *
 */
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t length){
	for (uint32_t i = 0; i < length; i++){
		// wait  until RXE is set to 1
		while(!(USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RXNE)));

		// check whether the USART word length is 8-bits or 9-bits
		// during reception
		if (pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS){
			// word length is 9 bits

			// check for parity control
			if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
				// no parity is used so all 9 bits are data
				*((uint16_t*)pRxBuffer) = (pUSARTHandle->pUSARTx->DR & (uint16_t)0x1FF);

				// increment the buffer twice
				pRxBuffer = pRxBuffer + 2;
			}
			else {
				// parity is used so only 8 bits are data
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR & (uint8_t)0xFF);
				pRxBuffer++;
			}
		}
		else {
			// word length is 8 bits

			// check for parity control
			if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE){
				// no parity is used so all 9 bits are data
				*pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->DR & (uint8_t)0x1FF);
			}
			else {
				// parity is used so only 7 bits are data and the last
				// bit is parity bit
				*pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->DR & (uint8_t)0x7F);
			}

			// increment the buffer once as word length = 8 bits
			pRxBuffer++;
		}
	}
}
/*
 * @func_name				- USART_SendData_IT
 *
 * @brief					- sends data via the specified USART peripheral in interrupt mode
 *
 * @param1					- USART handle of the USART peripheral from which data is being sent
 * @param2					- pointer to the transmitter buffer in which the data being sent is stored
 * @param3					- no. of bytes to transmit
 *
 * @return					- returns whether the USART is busy in transmission
 *
 * @Note					- this is non-blocking as it only sets the registers
 *
 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t length){
	uint8_t state = pUSARTHandle->txState;

	if (state != USART_BUSY_IN_TX){
		pUSARTHandle->txLen = length;
		pUSARTHandle->txSize = length;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->txState = USART_BUSY_IN_TX;

		// enable the TXEIE bit in CR1 to generate interrupt
		// whenever TXE = 1 in SR
		pUSARTHandle->pUSARTx->CR1 = (1 << USART_CR1_TXIE);

		// enable the TCIE bit in CR1 to generate interrupt
		// whenever TCE = 1 in SR
		pUSARTHandle->pUSARTx->CR1 = (1 << USART_CR1_TCIE);
	}

	return state;
}

/*
 * @func_name				- USART_ReceiveData_IT
 *
 * @brief					- receives data via the specified USART peripheral in interrupt mode
 *
 * @param1					- USART handle of the USART peripheral from which data is being received
 * @param2					- pointer to the transmitter buffer in which the data being sent is stored
 * @param3					- no. of bytes to transmit
 *
 * @return					- returns whether the USART is busy in reception
 *
 * @Note					- this is non-blocking as it only sets the registers
 *
 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t length){
	uint8_t state = pUSARTHandle->rxState;

	if (state != USART_BUSY_IN_RX){
		pUSARTHandle->rxLen = length;
		pUSARTHandle->pRxBuffer = pRxBuffer;
		pUSARTHandle->rxState = USART_BUSY_IN_RX;

		// enable the TXEIE bit in CR1 to generate interrupt
		// whenever TXE = 1 in SR
		pUSARTHandle->pUSARTx->CR1 = (1 << USART_CR1_RXNEIE);
	}

	return state;
}


/*
 * @func_name				- USART_IRQInterruptConfig
 *
 * @brief					- sets the USART interrupt configuration in NVIC registers.
 *							  Details about the NVIC registers are mentioned in pg-220 of
 *							  Cortex M4 user guide. The range of IRQNumbers (from 0 to 101)
 *							  is taken from table 40 from pg 250-253 in STM32F413_RM
 *
 * @param1					- interrupt number
 * @param2					- interrupt priority
 *
 * @return					- none
 *
 * @Note					- none
 *
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
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


/* USART interrupt priority configuration */
/*
 * @func_name				- USART_IRQPriorityConfig
 *
 * @brief					- sets the USART interrupt priority configuration in NVIC
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
void USART_IRQPriorityConfig(uint32_t IRQNumber, uint8_t IRQPriority){
	// finding the x in IPRx register (x = 0 to 59)
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8*iprx_section) + (8-NO_PR_BITS_IMPLEMENTED);
	*(NVIC_IPRx_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}


/*
 * @func_name				- USART_IRQHandler
 *
 * @brief					- handles the various interrupt events generated
 *
 * @param1					- pointer to handle of USART peripheral which generated the interrupt event
 *
 * @return					- none
 *
 * @Note					-
 *
 */
void USART_IRQHandler(USART_Handle_t *pUSARTHandle){

}
