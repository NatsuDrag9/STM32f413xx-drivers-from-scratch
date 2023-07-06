/*
 * usr_stm32f413xx_i2c.h
 *
 *  This file has the header of I2C driver
 */

#ifndef INC_USR_STM32F413XX_I2C_DRIVER_H_
#define INC_USR_STM32F413XX_I2C_DRIVER_H_

#include "usr_stm32f413xx.h"

/*
 * Configuration structure for I2Cx peripheral
 */
typedef struct {
	uint32_t I2C_SCLSpeed;		// values from @I2C_ClockSpeed
	uint8_t I2C_DeviceAddress;	// values from user
	uint8_t I2C_ACKControl;		// values from @I2C_ACKControl
	uint8_t I2C_FMDutyCycle;	// values from @I2C_FM_DUTY_CYCLE
} I2C_Config_t;

/*
 * Handle structure for I2Cx peripheral
 */
typedef struct {
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
	uint8_t TxRxState;			// stores the communication state when operating as interrupt
	uint8_t *pTxBuffer;			// stores application's tx buffer address
	uint8_t *pRxBuffer;			// stores application's rx buffer address
	uint32_t TxLen;				// stores length of data being transmitted in bytes
	uint32_t RxLen;				// stores length of data being received in bytes
	uint32_t RxSize;
	uint8_t Sr;					// asks user whether to use repeated start or not
	uint8_t DevAddress;			// stores the slave/device address
} I2C_Handle_t;

/*
 * @I2C_ClockSpeed
 * Possible values of I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM		100000		// standard mode = 100 kHz
#define I2C_SCL_SPEED_FM400K	400000		// fast mode 400 = 400 kHz
#define I2C_SCL_SPEED_FM200K	200000		// fast mode 200 = 200 kHz


/*
 * @I2C_ACKControl
 * Possible values of I2C_ACKControl
 */
#define I2C_ACK_ENABLE		1
#define I2C_ACK_DISABLE		0

/*
 * @I2C_FM_DUTY_CYCLE
 * Possible values of I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2		0
#define I2C_FM_DUTY_16_9	1

/*
 * I2C application states
 */
#define I2C_READY		0
#define I2C_BUSY_IN_RX	1
#define I2C_BUSY_IN_TX	2

/*
 * I2C status flags (SR1 and SR2)
 */
#define I2C_FLAG_SB			(1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR		(1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF		(1 << I2C_SR1_BTF)
#define I2C_FLAG_STOPF		(1 << I2C_SR1_STOPF)
#define I2C_FLAG_TXE		(1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE		(1 << I2C_SR1_RXNE)
#define I2C_FLAG_BERR		(1 << I2C_SR1_BERR)
#define I2C_FLAG_AF		(1 << I2C_SR1_AF)
#define I2C_FLAG_OVR		(1 << I2C_SR1_OVR)
//#define I2C_FLAG_SMBALERT	(1 << I2C_SR1_SMBALERT)
#define I2C_FLAG_TIMEOUT	(1 << I2C_SR1_TIMEOUT)

#define I2C_FLAG_MSL		(1 << I2C_SR2_MSL)
#define I2C_FLAG_BUSY		(1 << I2C_SR2_BUSY)
#define I2C_FLAG_TRA		(1 << I2C_SR2_TRA)
#define I2C_FLAG_GENCALL	(1 << I2C_SR2_GENCALL)
//#define I2C_FLAG_SMBDEFAULT (1 << I2C_SR2_SMBDEFAULT)
//#define I2C_FLAG_DUALF		(1 << I2C_SR2_DUALF)

/*
 * Repeat start enable/disable flags
 */
#define I2C_ENABLE_SR	1
#define I2C_DISABLE_SR	0

/*
 * I2C application events for I2C_ApplicationEventCallback()
 */
#define I2C_EV_TX_CMPLT		1
#define I2C_EV_RX_CMPLT		2
#define I2C_EV_STOP			3
#define I2C_ERROR_BERR		4
#define I2C_ERROR_ARLO		5
#define I2C_ERROR_AF		6
#define I2C_ERROR_OVR		7
#define I2C_ERROR_TIMEOUT	8
#define I2C_EV_DATA_REQ		9
#define I2C_EV_DATA_RCV		10

/* I2C API Prototypes */
/* I2C peripheral clock configuration */
void I2C_PeriClock_Control(I2C_RegDef_t* pI2Cx, uint8_t EnorDi);

/* I2C init and de-init */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

/* I2C data send and receive */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t length, uint8_t SlaveAddr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t length, uint8_t SlaveAddr);

uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint32_t length, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint32_t length, uint8_t SlaveAddr, uint8_t Sr);

void I2C_CloseSendData(I2C_Handle_t* pI2CHandle);
void I2C_CloseReceiveData(I2C_Handle_t* pI2CHandle);

void I2C_SlaveSendData(I2C_RegDef_t* pI2Cx, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t* pI2Cx);

/* I2C interrupts */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint32_t IRQNumber, uint8_t IRQPriority);
void I2C_EV_IRQHandler(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandler(I2C_Handle_t *pI2CHandle);

/*
 * Other peripheral control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);
uint8_t I2C_GetFlagStatus(I2C_RegDef_t* pI2Cx, uint32_t FlagName);
void I2C_ManageAcking(I2C_RegDef_t* pI2Cx, uint8_t EnorDi);
void I2C_GenerateStopCondition(I2C_RegDef_t* pI2Cx);
/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent);

#endif /* INC_USR_STM32F413XX_I2C_DRIVER_H_ */
