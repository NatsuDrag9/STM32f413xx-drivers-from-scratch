/*
 * usr_stm32f413xx_usart_driver.h
 *
 * Header file for USART
 */

#ifndef INC_USR_STM32F413XX_USART_DRIVER_H_
#define INC_USR_STM32F413XX_USART_DRIVER_H_

#include "usr_stm32f413xx.h"

/*
 * Configuration structure for USARTx peripheral
 */
typedef struct {
	uint8_t USART_Mode;					// values from @USART_MODE
	uint32_t USART_Baud;				// values from @USART_BAUDRATE
	uint8_t USART_ParityControl;		// values from @USART_PARITY
	uint8_t USART_NoOfStopBits;			// values from @USART_STOPBITS
	uint8_t USART_HWFlowControl;		// values from @USART_HWFlowControl
	uint8_t USART_WordLength;			// values from @USART_WORDLENGTH
	uint8_t USART_OverSampling;			// values from @USART_OVERSAMPLING
} USART_Config_t;

/*
 * Handle structure for USARTx peripheral
 */
typedef struct {
	USART_RegDef_t *pUSARTx;
	USART_Config_t USART_Config;
	uint8_t *pTxBuffer;			// stores the application's tx buffer address in interrupt mode
	uint8_t *pRxBuffer;			// stores the application's rx buffer address in interrupt mode
	uint8_t txLen;				// stores the length of application's data (in bytes) being sent in interrupt mode
	uint8_t txSize;				// stores the length of application's data (in bytes) being sent in interrupt mode
	uint8_t rxLen;				// stores the length of application's data (in bytes) being received in interrupt mode
	uint8_t txState;			// stores the current state of transmission in interrupt mode
	uint8_t rxState;			// stores the current state of reception in interrupt mode
} USART_Handle_t;

/*
 * @USART_MODE
 */
#define USART_MODE_ONLY_TX	0
#define USART_MODE_ONLY_RX	1
#define USART_MODE_TXRX		2

/*
 * @USART_BAUDRATE possible options
 */
#define USART_STD_BAUD_1200		1200
#define USART_STD_BAUD_2400		2400
#define USART_STD_BAUD_9600		9600
#define USART_STD_BAUD_19200	19200
#define USART_STD_BAUD_38400	38400
#define USART_STD_BAUD_57600	57600
#define USART_STD_BAUD_115200	115200
#define USART_STD_BAUD_230400	230400
#define USART_STD_BAUD_460800	460800
#define USART_STD_BAUD_921600	921600
#define USART_STD_BAUD_2M		2000000
#define USART_STD_BAUD_3M		3000000

/*
 * @USART_PARITYCONTROL possible options
 */
#define USART_PARITY_EN_ODD		2
#define USART_PARITY_EN_EVEN	1
#define USART_PARITY_DISABLE	0

/*
 * @USART_WORDLENGTH possible options
 */
#define USART_WORDLEN_8BITS		0
#define USART_WORDLEN_9BITS		1

/*
 * @USART_HWFLOWCONTORL
 */
#define USART_HW_FLOW_CTRL_NONE	0
#define USART_HW_FLOW_CTS		1
#define USART_HW_FLOW_RTS		2
#define USART_HW_FLOW_CTS_RTS	3

/*
 * @USART_STOPBITS possible options
 */
#define USART_STOPBITS_1		0
#define USART_STOPBITS_0_5		1
#define USART_STOPBITS_2		2
#define USART_STOPBITS_1_5		3

/*
 * @USART_OVERSAMPLING possible values
 */
#define USART_OVER_SAMPLING_16	0
#define USART_OVER_SAMPLING_8	1

/*
 * USARTx status register flags
 */
#define USART_FLAG_PE	(1 << USART_SR_PE)
#define USART_FLAG_FE	(1 << USART_SR_FE)
#define USART_FLAG_NF	(1 << USART_SR_NF)
#define USART_FLAG_ORE	(1 << USART_SR_ORE)
#define USART_FLAG_IDLE	(1 << USART_SR_IDLE)
#define USART_FLAG_RXNE	(1 << USART_SR_RXNE)
#define USART_FLAG_TC	(1 << USART_SR_TC)
#define USART_FLAG_TXE	(1 << USART_SR_TXE)
#define USART_FLAG_LBD	(1 << USART_SR_LBD)
#define USART_FLAG_CTS	(1 << USART_SR_CTS)

/*
 * USART status flags when communication is done in interrupt mode
 */
#define USART_READY		0
#define USART_BUSY_IN_RX	1
#define USART_BUSY_IN_TX	2

/* USART API Prototypes */
/* USART peripheral clock configuration */
void USART_PeriClock_Control(USART_RegDef_t* pUSARTx, uint8_t EnorDi);

/* USART init and de-init */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_RegDef_t *pUSARTx);

/* USART data send and receive */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t length);
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t length);

uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t length);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t length);

/* USART interrupts */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void USART_IRQPriorityConfig(uint32_t IRQNumber, uint8_t IRQPriority);
void USART_IRQHandler(USART_Handle_t *pUSARTHandle);
/*
 * Other peripheral control APIs
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnorDi);
uint8_t USART_GetFlagStatus(USART_RegDef_t* pUSARTx, uint32_t FlagName);
void USART_ClearFlag(USART_RegDef_t* pUSARTx, uint32_t FlagName);
void USART_SetBaudRate(USART_RegDef_t* pUSARTx, uint32_t BaudRate);

/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEvent);


#endif /* INC_USR_STM32F413XX_USART_DRIVER_H_ */
