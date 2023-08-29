/*
 * usr_stm32f413xx_i2c_driver.c
 *
 *  This file has the implementation of I2C driver
 */
#include <usr_stm32f413xx_i2c_driver.h>
///* Global variables */
//uint16_t AHB_PreScalar[8] = {2, 4, 8, 16, 64, 128, 256, 512};
//uint16_t APB1_PreScalar[4] = {2, 4, 8, 16};

/* Prototypes */
static void I2C_GenerateStartCondition(I2C_RegDef_t* pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t* pI2Cx, uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t* pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t* pI2CHandle);

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t* pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t* pI2CHandle);

/* Static function implementation */
/* Generates the START condition for I2C_MasterSendData() */
static void I2C_GenerateStartCondition(I2C_RegDef_t* pI2Cx){
	// set the START bit field in CR1
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

/* Sends the address with r/!w set to W('0') bit to the slave */
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t* pI2Cx, uint8_t SlaveAddr){
	// shift SlaveAddr by 1 bit
	SlaveAddr = SlaveAddr << 1;

	// set the LSB to 0 for the master to write
	SlaveAddr &= ~(1);

	// loading address into DR
	pI2Cx->DR = SlaveAddr;
}

/* Sends the address with r/!w set to R('1') bit to the slave */
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t* pI2Cx, uint8_t SlaveAddr){
	// shift SlaveAddr by 1 bit
	SlaveAddr = SlaveAddr << 1;

	// set the LSB to 1 for the master to read
	SlaveAddr |= 1;

	// loading address into DR
	pI2Cx->DR = SlaveAddr;
}

/* Clears the ADDR flag by reading SR1 and SR2 */
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle){
	uint32_t dummyRead = 0;

	// check for device mode
	if(pI2CHandle->pI2Cx->SR2 && (1 << I2C_SR2_MSL)){
		// device is in master mode
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
			if(pI2CHandle->RxSize == 1){
				// disable ACK first
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				// clear the ADDR flag
				dummyRead = pI2CHandle->pI2Cx->SR1;
				dummyRead = pI2CHandle->pI2Cx->SR2;
				(void)dummyRead;
			}
		}
		else {
			// clear the ADDR flag
			dummyRead = pI2CHandle->pI2Cx->SR1;
			dummyRead = pI2CHandle->pI2Cx->SR2;
			(void)dummyRead;
		}
	}
	else {
		// device is in slave mode
		// clear the ADDR flag
		dummyRead = pI2CHandle->pI2Cx->SR1;
		dummyRead = pI2CHandle->pI2Cx->SR2;
		(void)dummyRead;
	}
}

/* Generates STOP condition */
void I2C_GenerateStopCondition(I2C_RegDef_t* pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

/* Driver functions implementation */
/* I2C peripheral clock configuration */
/*
 * @func_name				- I2C_PeriClock_Control
 *
 * @brief					- enables/controls the specified I2C peripheral's PCLK
 *
 * @param1					- base address of the I2C peripherals whose clock has to be enabled (I2C1, I2C2, I2C2, I2C4)
 * @param2					- enable or disable macros
 *
 * @return					- none
 *
 * @Note					- none
 *
 */
void I2C_PeriClock_Control(I2C_RegDef_t* pI2Cx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		if (pI2Cx == I2C1) {
			I2C1_PCLK_EN();
		}
		else if (pI2Cx == I2C2) {
			I2C2_PCLK_EN();
		}
		else if (pI2Cx == I2C3) {
			I2C3_PCLK_EN();
		}

	}
	else {
		if (pI2Cx == I2C1) {
			I2C1_PCLK_DI();
		}
		else if (pI2Cx == I2C2) {
			I2C2_PCLK_DI();
		}
		else if (pI2Cx == I2C3) {
			I2C3_PCLK_DI();
		}
	}
}

/*
 * The process of computing pclk1 is taken from
 * the clock block diagram in STM32F4XX_RM
 */
//uint32_t RCC_GetPCLK1Value(void){
//	uint32_t pclk1 = 0, systemClk = 0;
//	uint8_t clkSource = 0, temp = 0, ahbPrescalar = 0, apb1Prescalar = 0;
//
//	clkSource = ((RCC->CFGR >> 2) & 0x03);
//
//	if (clkSource == 0) {
//		systemClk = 16000000;	// HSI ==> 16 MHz
//	}
//	else if (clkSource == 1) {
//		systemClk = 8000000;	// HSE ==> 8 MHz
//	}
//	else if (clkSource == 2)
//	{
//		systemClk = RCC_GetPLLOutputClock();
//	}
//
//	// gets AHB Prescalar
//	temp = ((RCC->CFGR >> 4) & 0xF);
//	if (temp < 8) {
//		ahbPrescalar = 1;
//	}
//	else {
//		ahbPrescalar = AHB_PreScalar[temp-8];
//	}
//
//	// gets APB1 Prescalar
//	temp = ((RCC->CFGR >> 10) & 0x07);
//	if (temp < 4) {
//		apb1Prescalar = 1;
//	}
//	else {
//		apb1Prescalar = APB1_PreScalar[temp-4];
//	}
//
//	pclk1 = (systemClk / ahbPrescalar) / apb1Prescalar;
//
//	return pclk1;
//}

/* I2C init */
/*
 * @func_name				- I2C_Init
 *
 * @brief					- initializes the I2C peripheral by setting its configuration
 *
 * @param1					- pointer to I2C handle to set the I2C configuration
 *
 * @return					- none
 *
 * @Note					- Ensure that the I2Cx (x = 1, 2, 3) is disabled when configuring the peripheral
 *
 */
void I2C_Init(I2C_Handle_t *pI2CHandle){
	uint32_t tempreg = 0;

	// enable the clock for I2Cx peripheral
	I2C_PeriClock_Control(pI2CHandle->pI2Cx, ENABLE);

	// configure the speed of SCL by setting the FREQ
	// bit fields in CR2 register
	tempreg = 0;
	tempreg |= RCC_GetPCLK1Value() / 1000000U;
	pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F);

	// configure the device's 7-bit own address
	// in OAR1 (applicable only when this devices is a slave)
	tempreg = 0;
	tempreg |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1);
	tempreg |= (1 << 14); 	// setting bit 14 to 1 according to pg-873 in STM32F4XX_RM
	pI2CHandle->pI2Cx->OAR1 = tempreg;

	// configure the mode (standard or fast) in CCR
	tempreg = 0;
	uint16_t ccr_value = 0;
	uint8_t sclSpeed = pI2CHandle->I2C_Config.I2C_SCLSpeed;
	uint8_t dutyCycle = pI2CHandle->I2C_Config.I2C_FMDutyCycle;
	if (sclSpeed <= I2C_SCL_SPEED_SM){
		// mode is standard mode -- 15th bit of tempreg is already 0 so need to clear it
		// compute and set ccr value
		ccr_value = (RCC_GetPCLK1Value() / (2*sclSpeed));	// assuming T_high = T_low, then add the equations given in SM mode on pg-879 in STM32F4XX_RM
		tempreg |= (ccr_value & 0xFFF); 	// masking only first 12 bits
	}
	else {
		// set fast mode
		tempreg |= (1 << I2C_CCR_FS);
		tempreg |= (dutyCycle << I2C_CCR_DUTY);

		// compute and set ccr value based on formula on pg-879 of STm32F4XX_RM
		if (dutyCycle == I2C_FM_DUTY_2) {
			ccr_value = (RCC_GetPCLK1Value() / (3*sclSpeed));	// assuming T_high = T_low, then add the equations given in SM mode on pg-879 in STM32F4XX_RM
		}
		else if (dutyCycle == I2C_FM_DUTY_16_9) {
			ccr_value = (RCC_GetPCLK1Value() / (25*sclSpeed));	// assuming T_high = T_low, then add the equations given in SM mode on pg-879 in STM32F4XX_RM
		}
		tempreg |= (ccr_value & 0xFFF); 	// masking only first 12 bits
	}
	pI2CHandle->pI2Cx->CCR = tempreg;

	// configure rise time for I2C pins
	// steps mentioned on pg-880 of STM32F413XX_RM
	tempreg = 0;
	if (sclSpeed <= I2C_SCL_SPEED_SM) {
		// standard mode
		tempreg = (RCC_GetPCLK1Value() / 1000000U) + 1;
	}
	else {
		// fast mode
		tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;
	}
	pI2CHandle->pI2Cx->TRISE = (tempreg & 0x3F);
}

/* I2C de-init */
/*
 * @func_name				- I2C_DeInit
 *
 * @brief					- de-initializes the specified I2C peripheral by resetting to default state
 *
 * @param1					- base address of the I2C peripheral which needs to be de-initialized
 *
 * @return					- none
 *
 * @Note					- none
 *
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx) {
	if (pI2Cx == I2C1) {
		I2C1_REG_RESET();
	}
	else if (pI2Cx == I2C2) {
		I2C2_REG_RESET();
	}
	else if (pI2Cx == I2C3) {
		I2C3_REG_RESET();
	}
}

/* I2C peripheral enable */
/*
 * @func_name				- I2C_PeripheralControl
 *
 * @brief					- enable/disable I2Cx peripheral before the communication begins by
 * 							  setting/resetting the SPE bit in CR1 register
 *
 * @param1					- base address of the I2C peripheral from which data is being sent
 * @param2					- enables or disable
 *
 * @return					- none
 *
 * @Note					-
 *
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi){
	if (EnorDi == ENABLE) {
		// sets the PE bit
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}
	else {

		// clears PE bit
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

/* Returns whether the specified flag in SR1 is set or not */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t* pI2Cx, uint32_t FlagName) {
	if (pI2Cx->SR1 & FlagName) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*
 * @func_name				- I2C_MasterSendData
 *
 * @brief					- selected master sends data via the specified I2C peripheral
 *
 * @param1					- pointer to handle of I2C peripheral from which data is being sent
 * @param2					- pointer to the transmitter buffer in which the data being sent is stored
 * @param3					- no. of bytes to transmit
 * @param4					- address of the slave to which the data is to be sent
 *
 * @return					- none
 *
 * @Note					- this is blocking call (polling) as the function does not return until
 * 							  all the length no. of bytes are not transferred
 *
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t length, uint8_t SlaveAddr){
	// generate START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// confirm that start generation is implemented by checking the SB flag in SR1
	// if start condition was generated, hardware sets SR1 flag which needs to be
	// cleared by the software
	// Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)));

	// send the address of the slave with r/w bit set to w (i.e. '0')
	// -- total 8 bits
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	// confirm address phase is completed by checking the ADDR flag in SR1
	while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)));

	// clear the ADDR flag according to its software sequence
	// refer pg - 877 in STM32F413XX_RM
	I2C_ClearADDRFlag(pI2CHandle);

	// send data until the length becomes
	while (length > 0){
		// wait until TXE is set in SR1
		while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)));
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		length--;
	}

	// when length becomes 0 wait until both, TXE and BTF are set to '1'
	// before generating the STOP condition
	// Note: TXE=1 and BTF=1 means that both SR1 and DR are empty and,
	// next transmission should begin
	// when BTF=1 SCL is stretched (pulled to LOW)
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)));
	while(!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF)));

	// generate the STOP condition
	// Note: generation STOP condition automatically clears the BTF
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

}

/* Manage I2C ACK bit in CR1 */
void I2C_ManageAcking(I2C_RegDef_t* pI2Cx, uint8_t EnorDi){
	if (EnorDi == I2C_ACK_ENABLE){
		// enable ACK bit
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
	else {
		// disable ACK bit
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

/* I2C master receive data */
/*
 * @func_name				- I2C_MasterReceiveData
 *
 * @brief					- selected master receives data from the specified I2C peripheral
 *
 * @param1					- pointer to handle of I2C peripheral to which data is being received
 * @param2					- pointer to the receiver buffer in which the data being sent is stored
 * @param3					- no. of bytes to transmit
 * @param4					- address of the slave to which the data is to be sent
 *
 * @return					- none
 *
 * @Note					- this is blocking call (polling) as the function does not return until
 * 							  all the length no. of bytes are not received
 *
 */

void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t length, uint8_t SlaveAddr){
	// generate START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// confirm that START generation is completed by
	// checking the SB flag in the SR1
	// Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)));

	// send the address of slave with R/!W bit set to read('1')
	// -- total of 8 bits
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	// wait until address phase is completed by checking the ADDR flag
	// in SR1
	while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)));

//	printf("Configured registers for Read\n");

	// procedure to read only one byte from slave
	if (length == 1){
		// disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

		// clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		// wait until RXNE becomes 1 data has been read and
		// DR is empty
		while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

		// read data into buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;

		// generate STOP condition
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	}

	// procedure to read data from slave when length > 1
	if (length > 1) {
		// clear the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		// read the data until length becomes zero
		for(uint32_t i = length; i > 0; i--){

			// read the data from DR into buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			// wait until RXNE becomes 0 indicating that data
			// has been read and DR is empty
			while(!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

			if (i == 2){
				// if last two bytes are remaining

				// clear the ACK bit
				I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);

				// generate STOP condition
				I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}

			// read data into buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;

			// increment the buffer address
			pRxBuffer++;
		}
	}

	// re-enabling ACK
	if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE){
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}
}

/*
 * @func_name				- I2C_MasterSendDataIT
 *
 * @brief					- configures the interrupt bits for the selected master to send data
 * 							- via the specified I2C peripheral using interrupts
 *
 * @param1					- pointer to handle of I2C peripheral from which data is being sent
 * @param2					- pointer to the transmitter buffer in which the data being sent is stored
 * @param3					- no. of bytes to transmit
 * @param4					- address of the slave to which the data is to be sent
 * @param5					- accepts repeated start from application layer
 *
 * @return					- none
 *
 * @Note					- this is non-blocking call

 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t length, uint8_t SlaveAddr, uint8_t Sr){
	uint8_t busyState = pI2CHandle->TxRxState;

	if ((busyState != I2C_BUSY_IN_TX) && (busyState != I2C_BUSY_IN_RX)){
		printf("inside MasterSendDataIT()\n");
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = length;
		pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
		pI2CHandle->DevAddress = SlaveAddr;
		pI2CHandle->Sr = Sr;

		// generate START condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// enable ITBUF bit in CR2 to generate interrupts
		// when TXE = 1 or RXNE = 1
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		// enable ITEVTEN bit in CR2 to generate interrupts
		// when an event occurs (event list on pg-871 of
		// STM32F413XX_RM)
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		// enable ITERREN to generate interrupts when error
		// occurs
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	printf("Exiting MasterSendDataIT()\n");

	return busyState;
}

/*
 * @func_name				- I2C_MasterReceiveDataIT
 *
  * @brief					- configures the interrupt bits for the selected master to
 * 							- receive data via the specified I2C peripheral using interrupts
 *
 * @param1					- pointer to handle of I2C peripheral from which data is being received
 * @param2					- pointer to the receiver buffer in which the data being received is stored
 * @param3					- no. of bytes to transmit
 * @param4					- address of the slave to which the data is to be received
 * @param5					- accepts repeated start from application layer
 *
 * @return					- none
 *
 * @Note					- this is non-blocking call

 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t length, uint8_t SlaveAddr, uint8_t Sr){
	uint8_t busyState = pI2CHandle->TxRxState;

	if ((busyState != I2C_BUSY_IN_TX) && (busyState != I2C_BUSY_IN_RX)){
		pI2CHandle->pRxBuffer = pRxBuffer;
		pI2CHandle->RxLen = length;
		pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
		pI2CHandle->RxSize = length;	// used in the ISR code to manage the data reception
		pI2CHandle->DevAddress = SlaveAddr;
		pI2CHandle->Sr = Sr;

		// generate START condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		// enable ITBUF bit in CR2 to generate interrupts
		// when TXE = 1 or RXNE = 1
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		// enable ITEVTEN bit in CR2 to generate interrupts
		// when an event occurs (event list on pg-871 of
		// STM32F413XX_RM)
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		// enable ITERREN to generate interrupts when error
		// occurs
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	printf("Exiting MasterReceiveDataIT()\n");

	return busyState;
}

/*
 * @func_name				- I2C_SlaveSendData
 *
  * @brief					- sends data to master
 *
 * @param1					- base address of the I2C peripheral that sends data to master
 * @param2					- data to be sent to master (byte-by-byte)
 *
 * @return					- returns the received data
 *
 * @Note					-
 */
void I2C_SlaveSendData(I2C_RegDef_t* pI2Cx, uint8_t data){
	pI2Cx->DR = data;
}

/*
 * @func_name				- I2C_SlaveReceiveData
 *
  * @brief					- receives data from master
 *
 * @param1					- base address of the I2C peripheral that sends data to master
 *
 * @return					- none
 *
 * @Note					- returns the received data
 */
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t* pI2Cx){
	return (uint8_t) pI2Cx->DR;
}

/* I2C interrupt configuration */
/*
 * @func_name				- I2C_IRQInterruptConfig
 *
 * @brief					- sets the I2C interrupt configuration in NVIC registers.
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
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi){
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


/* I2C interrupt priority configuration */
/*
 * @func_name				- I2C_IRQPriorityConfig
 *
 * @brief					- sets the I2C interrupt priority configuration in NVIC
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
void I2C_IRQPriorityConfig(uint32_t IRQNumber, uint8_t IRQPriority){
	// finding the x in IPRx register (x = 0 to 59)
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8*iprx_section) + (8-NO_PR_BITS_IMPLEMENTED);
	*(NVIC_IPRx_BASEADDR + iprx) |= (IRQPriority << shift_amount);
}

/*
 * Disable all interrupt enable bits to prevent
 * further generation of interrupts
 * Resets member elements of I2C handle structure to
 * end transmission
 */
void I2C_CloseSendData(I2C_Handle_t* pI2CHandle){
	// disable ITBUFEN control bit
	pI2CHandle->pI2Cx->SR2 &= ~(1 << I2C_CR2_ITBUFEN);

	// disable ITEVFEN bit
	pI2CHandle->pI2Cx->SR2 &= ~(1 << I2C_CR2_ITEVTEN);

	// reset member elements
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->TxLen = 0;
	pI2CHandle->pTxBuffer = NULL;

}

/*
 * Disable all interrupt enable bits to prevent
 * further generation of interrupts
 * Resets member elements of I2C handle structure to
 * end reception
 */
void I2C_CloseReceiveData(I2C_Handle_t* pI2CHandle){
	// disable ITBUFEN control bit
	pI2CHandle->pI2Cx->SR2 &= ~(1 << I2C_CR2_ITBUFEN);

	// disable ITEVFEN bit
	pI2CHandle->pI2Cx->SR2 &= ~(1 << I2C_CR2_ITEVTEN);

	// reset member elements
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	// enable ACK bit to ensure that it is set to the
	// value provided by application
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE){
		I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}
}

/*
 * Handles the TXE interrupt when device is in master mode
 */
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t* pI2CHandle){
	// transmit data to slave only when device is in master mode
	if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
		if(pI2CHandle->TxLen > 0){
			// load the data into DR
			pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

			// decrement the length
			pI2CHandle->TxLen--;

			// increment the pTxBuffer address
			pI2CHandle->pTxBuffer++;
		}
	}
}

/*
 * Handles the RXNE interrupt when device is in master mode
 */
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t* pI2CHandle){
	// data reception when length = 1
	if(pI2CHandle->RxSize == 1){
		// read DR
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
	}

	// data reception when length > 1
	if(pI2CHandle->RxSize > 1){
		if(pI2CHandle->RxLen == 2){
			// disable ACK bit
			I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
		}

		// read DR
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}

	// end communication when RxLen = 0
	if(pI2CHandle->RxLen == 0){
		// generate STOP condition when no more start repeat is sent
		if(pI2CHandle->Sr == I2C_DISABLE_SR){
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		}

		// close reception
		I2C_CloseReceiveData(pI2CHandle);

		// notify the application about completion of data reception
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}


/*
 * @func_name				- I2C_EV_IRQHandler
 *
 * @brief					- handles the various interrupt events generated by SR1 bit fields:
 * 							- SB, ADDR, BTF, STOPF, TXE, RXNE
 *
 * @param1					- pointer to handle of I2C peripheral which generated the I2C_EV interrupt
 *
 * @return					- none
 *
 * @Note					- this EV interrupt handler is common for both master and slave mode
 * 							- of a device
 *
 */
void I2C_EV_IRQHandler(I2C_Handle_t *pI2CHandle){
	uint8_t temp1 = 0, temp2 = 0, temp3 = 0;

	temp1 = pI2CHandle->pI2Cx->CR1 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR1 & (1 << I2C_CR2_ITBUFEN);
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);

	// handle EV interrupt generated by SB flag in SR1
	// Note - SB flag is only applicable in master mode
	if (temp1 && temp3){
		// device in master mode
		// SB flag is set and generates the EV interrupt
		// executing address phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
			// device is transmitting data
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddress);
		}
		else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
			// device is receiving data
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddress);
		}
	}

	// handle EV interrupt generated by ADDR flag in SR1
	// Note - when device is in master mode, address is sent to slave.
	// When device is in slave mode, address is matched with own
	// address in OARx (x = 1, 2)
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
	if (temp1 && temp3){
		// clear ADDR flag that was set
		I2C_ClearADDRFlag(pI2CHandle);
	}

	// handle EV interrupt generated by BTF flag in SR1
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
	if (temp1 && temp3){
		// BTF flag is set
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX){
			// check whether TXE is set
			if(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)){
				// both BTF and TXE are set
				if(pI2CHandle->TxLen == 0) {
					// closing transmission when TxLen = 0

					if(pI2CHandle->Sr == I2C_DISABLE_SR){
						// generate STOP condition when repeat start is disabled
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}

					// reset all member elements of I2C handle structure
					I2C_CloseSendData(pI2CHandle);

					// notify the application about completion of transmission
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}
			else if (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE)){
				// do nothing
			}
		}
	}

	// handle EV interrupt generated by STOPF flag in SR1
	// Note - only applicable when device is in slave mode
	// The code below will not execute when device is in
	// master mode
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
	if (temp1 && temp3){
		// STOPF flag is set
		// clear the STOPF by: 1.) reading SR1; 2.) writing to CR1
		// SR1 is already read and assigned to temp3 above
		pI2CHandle->pI2Cx->CR1 |= 0x0000;

		// notify the application that STOP bit is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
	}

	// handle EV interrupt generated by TXE flag in SR1
	// TXE interrupt occurs when ITEVFEN and ITBUFEN are
	// enabled
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
	if (temp1 && temp2 && temp3){
		// TXE flag is set
		// check whether device is master
		if(pI2CHandle->pI2Cx->SR2 && (1 << I2C_SR2_MSL)){
			// device is master
			I2C_MasterHandleTXEInterrupt(pI2CHandle);
		}
		else {
			// device is in slave mode
			// ensuring that the slave is in transmitter mode
			if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)){
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}

	// handle EV interrupt generated by RXNE flag in SR1
	// RXNE interrupt occurs when ITEVFEN and ITBUFEN are
	// enabled
	temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
	if (temp1 && temp2 && temp3){
		// ensure that device is in master mode
		if(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL)){
			// RXNE flag is set
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX){
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}
		else {
			// device is in slave mode
			// ensuring that the slave is in receiver mode
			if(!(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))){
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}
}

/*
 * @func_name				- I2C_ER_IRQHandler
 *
 * @brief					- clears the EXTI PR register corresponding to the pin number
 *
 * @param1					- pointer to handle of I2C peripheral which generated the I2C_ER interrupt
 *
 * @return					- none
 *
 * @Note					- this ER interrupt handler is common for both master and slave mode of a device
 *
 */
void I2C_ER_IRQHandler(I2C_Handle_t *pI2CHandle){
	uint8_t temp1 = 0, temp2 = 0;

	// check the status of ITERREN bit in CR2
	temp2 = (pI2CHandle->pI2Cx->CR2) & (1 << I2C_CR2_ITERREN);

	// check for bus error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_BERR);
	if(temp1 && temp2){
		// bus error occurred
		// clear the BERR bit in SR1
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);

		// notify the application that bus error occurred
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
	}

	// check for arbitration lost error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_ARLO);
	if(temp1 && temp2){
		// ACK failure occurred
		// clear the ARLO bit in SR1
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);

		// notify the application that ARLO error occurred
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
	}

	// check for ACK failure
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_AF);
	if(temp1 && temp2){
		// ACK failure occurred
		// clear the AF bit in SR1
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

		// notify the application that ACK failure occurred
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}

	// check for overrun/underrun erorr
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_OVR);
	if(temp1 && temp2){
		// overrun/underrun condition occurred
		// clear the AF bit in SR1
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);

		// notify the application that ACK failure occurred
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}

	// check for TIMEOUT error
	temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_TIMEOUT);
	if(temp1 && temp2){
		// TIMEOUT occurred
		// Note -- hardware generates STOP bit when in master mode.
		// In slave mode, device resets communication.
		// Do nothing in software

		// clear the TIMEOUT bit in SR1
		pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

		// notify the application that ACK failure occurred
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}

}
