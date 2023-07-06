/*
 * usr_stm32f413xx_spi_driver.c
 *
 * This file has the implementation of SPI driver
 */

#include "usr_stm32f413xx_spi_driver.h"

/* Static function prototypes */
static void SPI_Txe_Interrupt_Handle(SPI_Handle_t *pSPIHandle);
static void SPI_Rxne_Interrupt_Handle(SPI_Handle_t *pSPIHandle);
static void SPI_Ovr_Interrupt_Handle(SPI_Handle_t *pSPIHandle);

/* SPI peripheral clock configuration */
/*
 * @func_name				- SPI_PeriClock_Control
 *
 * @brief					- enables/controls the specified SPI peripheral's PCLK
 *
 * @param1					- base address of the SPI peripherals whose clock has to be enabled (SPI1, SPI2, SPI2, SPI4)
 * @param2					- enable or disable macros
 *
 * @return					- none
 *
 * @Note					- none
 *
 */
void SPI_PeriClock_Control(SPI_RegDef_t* pSPIx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pSPIx == SPI1) {
			SPI1_PCLK_EN();
		}
		else if (pSPIx == SPI2) {
			SPI2_PCLK_EN();
		}
		else if (pSPIx == SPI3) {
			SPI3_PCLK_EN();
		}
		else if (pSPIx == SPI4) {
			SPI4_PCLK_EN();
		}

	}
	else {
		if (pSPIx == SPI1) {
			SPI1_PCLK_DI();
		}
		else if (pSPIx == SPI2) {
			SPI2_PCLK_DI();
		}
		else if (pSPIx == SPI3) {
			SPI3_PCLK_DI();
		}
		else if (pSPIx == SPI4) {
			SPI4_PCLK_DI();
		}
	}
}

/* SPI init */
/*
 * @func_name				- SPI_Init
 *
 * @brief					- initializes the SPI peripheral by setting its configuration
 *
 * @param1					- pointer to SPI handle to set the SPI configuration
 *
 * @return					- none
 *
 * @Note					- none
 *
 */
void SPI_Init(SPI_Handle_t *pSPIHandle){
	uint8_t deviceMode = pSPIHandle->SPIConfig.SPI_DeviceMode;
	uint8_t busConfig = pSPIHandle->SPIConfig.SPI_BusConfig;
	uint8_t dff = pSPIHandle->SPIConfig.SPI_Dff;
	uint8_t cpha = pSPIHandle->SPIConfig.SPI_CPHA;
	uint8_t cpol = pSPIHandle->SPIConfig.SPI_CPOL;
	uint8_t ssm = pSPIHandle->SPIConfig.SPI_SSM;
	uint8_t sClkSpeed = pSPIHandle->SPIConfig.SPI_SClkSpeed;
//	uint8_t lsbFirst = 1;

	// Enabling SPI PCLK
	SPI_PeriClock_Control(pSPIHandle->pSPIx, ENABLE);

	// configure the SPI_CR1 register
	uint32_t tempreg = 0;

	// configure the device as master
	tempreg |= deviceMode << SPI_CR1_MSTR;

	// configure the SPI bus
	if (busConfig == SPI_BUS_CONFIG_FD) {
		// clear the BIDIMODE bit
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	else if (busConfig == SPI_BUS_CONFIG_HD) {
		// set the BIDIMODE bit
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}
	else if (busConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY) {
		// set the BIDIMODE bit
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);

		// clear the RXONLY bit
		tempreg &= ~(1 << SPI_CR1_RXONLY);
	}

	// configure the clock
	tempreg |= (sClkSpeed << SPI_CR1_BR);

	// configure the data frame format (DFF)
	tempreg |= (dff << SPI_CR1_DFF);

	// configure the CPOL bit
	tempreg |= (cpol << SPI_CR1_CPOL);

	// configure the CPHA bit
	tempreg |= (cpha << SPI_CR1_CPHA);

	// configure SSM and:
	// (i) SSI bit in CR1 when SSM is enabled
	// (ii) Or, SSOE bit in CR2 when SSM is disabled

	if (ssm == SPI_SSM_EN) {
		// set the SSM bit in CR1
		tempreg |= (ssm << SPI_CR1_SSM);

		if (deviceMode == SPI_DEVICE_MODE_MASTER) {
			// set SSI bit to configure device as master when SSM is enabled
			tempreg |= (1 << SPI_CR1_SSI);
		}
		else if (deviceMode == SPI_DEVICE_MODE_SLAVE) {
			// clear SSI bit to configure device as salve when SSM is enabled
			tempreg &= ~(1 << SPI_CR1_SSI);
		}
	}
	else if (ssm == SPI_SSM_DI) {
		// clear the SSM bit in CR1 (value of ssm here is '0'
		// so don't need to perform &= ~(1 << SPI_CR1_SSM)
		tempreg |= (ssm << SPI_CR1_SSM);

		if (deviceMode == SPI_DEVICE_MODE_MASTER) {
			// single master configuration
			pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
		}
		else {
			// multi-master configuration
			pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
		}

	}

	// MSB transmitted first by default
//	tempreg &= ~(lsbFirst << SPI_CR1_LSBFIRST);

	pSPIHandle->pSPIx->CR1 = tempreg;
}


/* SPI de-init */
/*
 * @func_name				- SPI_DeInit
 *
 * @brief					- de-initializes the specified SPI peripheral by resetting to default state
 *
 * @param1					- base address of the SPI peripheral which needs to be de-initialized
 *
 * @return					- none
 *
 * @Note					- none
 *
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx) {
	if (pSPIx == SPI1) {
		SPI1_REG_RESET();
	}
	else if (pSPIx == SPI2) {
		SPI2_REG_RESET();
	}
	else if (pSPIx == SPI3) {
		SPI3_REG_RESET();
	}
	else if (pSPIx == SPI4) {
		SPI4_REG_RESET();
	}
}

/* Returns whether the specified flag in SR is set or not */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t* pSPIx, uint32_t FlagName) {
	if (pSPIx->SR & FlagName) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/* SPI peripheral enable */
/*
 * @func_name				- SPI_PeripheralControl
 *
 * @brief					- enable/disable SPIx peripheral before the communication begins by
 * 							  setting/resetting the SPE bit in CR1 register
 *
 * @param1					- base address of the SPI peripheral from which data is being sent
 * @param2					- enables or disable
 *
 * @return					- none
 *
 * @Note					- this is blocking call (polling) as the function does not return until
 * 							  all the length np. of bytes are not transferred
 *
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi){
	if (EnorDi == ENABLE) {
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else {

		// wait until the BSY flag in SR becomes '0'
		while (SPI_GetFlagStatus(pSPIx, SPI_BUSY_FLAG));

		// clears SPE bit when the BSY flag is '0'
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/* SPI data send */
/*
 * @func_name				- SPI_SendData
 *
 * @brief					- sends data via the specified SPI peripheral
 *
 * @param1					- handle to the SPI peripheral from which data is being sent
 * @param2					- pointer to the transmitter buffer in which the data being sent is stored
 * @param3					- no. of bytes to transmit
 *
 * @return					- none
 *
 * @Note					- this is blocking call (polling) as the function does not return until
 * 							  all the length np. of bytes are not transferred
 *
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t* pTxBuffer, uint32_t length){
	while (length > 0) {
		// wait until Tx buffer is empty i.e. until TXE flag is set in SR
		while (!(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG)));

		// check the DFF bit in CR1
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
			// 16-bit DFF
			// load the data into the DR
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			length = length - 2;	// reducing length by 2 bytes as 16-bit data
			(uint16_t*)pTxBuffer++;
		}
		else {
			// 8-bit DFF
			// load the data into the DR
			pSPIx->DR = *pTxBuffer;
			length--;
			(uint8_t*)pTxBuffer++;
		}

		//	During discontinuous communications, there is a 2 APB clock period delay between the
		//	write operation to the SPI_DR register and BSY bit setting. As a consequence it is
		//	mandatory to wait first until TXE is set and then until BSY is cleared after writing the last
		//	data.


		// wait until Tx buffer is empty i.e. until TXE flag is set in SR
		while (!(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG)));

		// wait until the BSY flag in SR becomes '0'
		while (SPI_GetFlagStatus(pSPIx, SPI_BUSY_FLAG));
	}

	//  clear the overrun flag by reading DR and SR
	uint16_t temp = 0;
	if (SPI_GetFlagStatus(pSPIx, SPI_OVR_FLAG)) {
		temp = (uint16_t) pSPIx->DR;
		temp = pSPIx->SR;
	}
	(void) temp;
}

/* SPI data send in interrupt mode */
/*
 * @func_name				- SPI_SendData_IT
 *
 * @brief					- sends data via the specified SPI peripheral in interrupt mode
 *
 * @param1					- SPI handle of the SPI peripheral from which data is being sent
 * @param2					- pointer to the transmitter buffer in which the data being sent is stored
 * @param3					- no. of bytes to transmit
 *
 * @return					- returns whether the SPI is busy in transmission
 *
 * @Note					- this is non-blocking as it only sets the registers
 *
 */
uint8_t SPI_SendData_IT(SPI_Handle_t *pSPIHandle, uint8_t* pTxBuffer, uint32_t length){
	uint8_t state = pSPIHandle->txState;

	if (state != SPI_BUSY_IN_TX) {
		// save the txBuffer address and length of data in global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->txLen = length;

		// mark the SPI state as busy
		pSPIHandle->txState = SPI_BUSY_IN_TX;

		// enable the TXIE control bit to generate interrupt when TXE bit is set
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}

	// data transmission is handled by the ISR code


	return state;
}

/* SPI data receive */
/*
 * @func_name				- SPI_ReceiveData
 *
 * @brief					- receives data from the specified SPI peripheral
 *
 * @param1					- handle to the SPI peripheral from which data is being sent
 * @param2					- pointer to the receiver buffer in which the data being sent is stored
 * @param3					- length of the data to be sent in bytes
 *
 * @return					- none
 *
 * @Note					- this is blocking call (polling) as the function does not return until
 * 							  all the length no. of bytes are not transferred
 *
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t* pRxBuffer, uint32_t length){
	while (length > 0) {
		// wait until Rx buffer is empty i.e. until RXNE flag is set in SR
		while (!(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG)));

		// check the DFF bit in CR1
		if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
			// 16-bit DFF
			// load the data from DR to rxBuffer
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			length = length - 2;	// reducing length by 2 bytes as 16-bit data
			(uint16_t*)pRxBuffer++;
		}
		else {
			// 8-bit DFF
			// load the data from DR to rxBuffer
			*pRxBuffer = pSPIx->DR;
			length--;	// reducing length by 1 byte as 8-bit data
			(uint8_t*)pRxBuffer++;
		}

		//	During discontinuous communications, there is a 2 APB clock period delay between the
		//	write operation to the SPI_DR register and BSY bit setting. As a consequence it is
		//	mandatory to wait first until TXE is set and then until BSY is cleared after writing the last
		//	data.


		// wait until Rx buffer is empty i.e. until TXE flag is set in SR
		while (!(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG)));

		// wait until the BSY flag in SR becomes '0'
		while (SPI_GetFlagStatus(pSPIx, SPI_BUSY_FLAG));
	}

	//  clear the overrun flag by reading DR and SR if set by hardware
	uint16_t temp = 0;
	if (SPI_GetFlagStatus(pSPIx, SPI_OVR_FLAG)) {
		temp = (uint16_t) pSPIx->DR;
		temp = pSPIx->SR;
	}
	(void)temp;

}

/* SPI data receive in interrupt mode */
/*
 * @func_name				- SPI_ReceiveData_IT
 *
 * @brief					- receives data via the specified SPI peripheral in interrupt mode
 *
 * @param1					- SPI handle of the SPI peripheral from which data is being received
 * @param2					- pointer to the transmitter buffer in which the data being sent is stored
 * @param3					- no. of bytes to transmit
 *
 * @return					- returns whether the SPI is busy in reception
 *
 * @Note					- this is non-blocking as it only sets the registers
 *
 */
uint8_t SPI_ReceiveData_IT(SPI_Handle_t *pSPIHandle, uint8_t* pRxBuffer, uint32_t length){
	uint8_t state = pSPIHandle->rxState;

	if (state != SPI_BUSY_IN_RX) {
		// save the txBuffer address and length of data in global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->rxLen = length;

		// mark the SPI state as busy
		pSPIHandle->rxState = SPI_BUSY_IN_RX;

		// enable the TXIE control bit to generate interrupt when TXE bit is set
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}

	// data reception is handled by the ISR code


	return state;
}

/* SPI interrupt configuration */
/*
 * @func_name				- SPI_IRQInterruptConfig
 *
 * @brief					- sets the SPI interrupt configuration in NVIC registers.
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
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
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


/* SPI interrupt priority configuration */
/*
 * @func_name				- SPI_IRQPriorityConfig
 *
 * @brief					- sets the SPI interrupt priority configuration in NVIC
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
void SPI_IRQPriorityConfig(uint32_t IRQNumber, uint8_t IRQPriority){
	// finding the x in IPRx register (x = 0 to 59)
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8*iprx_section) + (8-NO_PR_BITS_IMPLEMENTED);
	*(NVIC_IPRx_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}

/* Implements the SPI transmission in interrupt mode */
static void SPI_Txe_Interrupt_Handle(SPI_Handle_t *pSPIHandle) {
	// check the DFF bit in CR1
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
		// 16-bit DFF
		// load the data from the txbuffer into DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->txLen = pSPIHandle->txLen - 2;	// reducing length by 2 bytes as 16-bit data
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}
	else {
		// 8-bit DFF
		// load the data from the txbuffer into DR
		pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->txLen--;	// reducing length by 1 byte as 16-bit data
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}

	// close SPI transmission when txLen is 0
	if (pSPIHandle->txLen == 0){
		// close transmission
		SPI_CloseTransmission(pSPIHandle);

		// callback on completion of transmission
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

/* Implements the SPI reception in interrupt mode */
static void SPI_Rxne_Interrupt_Handle(SPI_Handle_t *pSPIHandle) {
	// check the DFF bit in CR1
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) {
		// 16-bit DFF
		// load the data into the rxbuffer from DR
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->rxLen = pSPIHandle->rxLen - 2;	// reducing length by 2 bytes as 16-bit data
		// incrementing by 2 bytes
		pSPIHandle->pRxBuffer++;
		pSPIHandle->pRxBuffer++;
	}
	else {
		// 8-bit DFF
		//  load the data into the rxbuffer from DR
		*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
		pSPIHandle->rxLen--;	// reducing length by 1 byte as 16-bit data
		pSPIHandle->pRxBuffer++;
	}

	// close SPI reception when rxLen is 0
	if (pSPIHandle->rxLen == 0){
		// close receptions
		SPI_CloseReception(pSPIHandle);
		// callback on completion of transmission
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}

}

/* Handles the overrun error in interrupt mode */
static void SPI_Ovr_Interrupt_Handle(SPI_Handle_t *pSPIHandle) {

	uint8_t temp;

	// clear the OVR flag
	if (pSPIHandle->txState != SPI_BUSY_IN_TX) {
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}

	(void)temp;
	// inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERROR);
}

/* SPI close transmission when in interrupt mode */
/*
 * @func_name				- SPI_CloseTransmission
 *
 * @brief					- Ends the SPI reception when in interrupt mode
 *
 * @param1					- pointer to SPI handle
 *
 * @return					- none
 *
 * @Note					- none
 *
 */

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle){

	// clear the TXEIE bit to prevent interrupts from it
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);

	// set the txbuffer address to null
	pSPIHandle->pTxBuffer = NULL;

	// reset the global variable holding length of data to be transmitted
	pSPIHandle->txLen = 0;

	// reset the txstate for the next transmission
	pSPIHandle->txState = SPI_READY;
}

/* SPI close reception when in interrupt mode */
/*
 * @func_name				- SPI_CloseReception
 *
 * @brief					- Ends the SPI reception when in interrupt mode
 *
 * @param1					- pointer to SPI handle
 *
 * @return					- none
 *
 * @Note					- none
 *
 */
void SPI_CloseReception(SPI_Handle_t *pSPIHandle){

	// clear the TXEIE bit to prevent interrupts from it
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);

	// set the rxbuffer to NULL
	pSPIHandle->pRxBuffer = NULL;

	// reset the global variable holding length of data to be received
	pSPIHandle->rxLen = 0;

	// reset the txstate for the next reception
	pSPIHandle->rxState = SPI_READY;
}


/*
 * @func_name				- SPI_ClearOVRFlag
 *
 * @brief					- Clears the OVR flag when in interrupt mode
 *
 * @param1					- pointer to SPI handle
 *
 * @return					- none
 *
 * @Note					- none
 *
 */
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx){
	uint8_t temp = 0;
	temp = pSPIx->DR;
	temp = pSPIx->SR;

	(void)temp;
}

/* SPI interrupts handler */
/*
 * @func_name				- SPI_IRQHandler
 *
 * @brief					- clears the EXTI PR register corresponding to the SPI handle
 *
 * @param1					- pointer to SPI handle
 *
 * @return					- none
 *
 * @Note					- none
 *
 */
void SPI_IRQHandler(SPI_Handle_t *pSPIHandle){
	uint8_t temp1 = 0, temp2 = 0;

	// check whether interrupt was from TXE flag in SR
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if (temp1 && temp2) {
		// handle TXE
		SPI_Txe_Interrupt_Handle(pSPIHandle);
	}

	// check whether interrupt was from RXNE flag in SR
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	if (temp1 && temp2) {
		// handle RXNE
		SPI_Rxne_Interrupt_Handle(pSPIHandle);
	}

	// handling overrun error
	// check whether interrupt was from TXE flag in SR
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	if (temp1 && temp2) {
		SPI_Ovr_Interrupt_Handle(pSPIHandle);
	}
}

/*
 * Weak implementation of application callback function
 * Needs to be implemented by the application
 */
__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent){
	// does nothing
}
