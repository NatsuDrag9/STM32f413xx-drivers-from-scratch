/*
 * usr_stm32f413xx_spi_driver.h
 *
 * This file contains all the necessary declarations
 * for the SPI driver
 */

#ifndef INC_USR_STM32F413XX_SPI_DRIVER_H_
#define INC_USR_STM32F413XX_SPI_DRIVER_H_

#include "usr_stm32f413xx.h"


typedef struct {
	uint8_t SPI_DeviceMode;		// possible values from @SPI_DEVICE_MODE
	uint8_t SPI_BusConfig;		// possible values from @SPI_BUS_CONFIG
	uint8_t SPI_Dff;			// possible values from @SPI_DATA_FRAME
	uint8_t SPI_CPHA;			// possible values from @SPI_CPHA
	uint8_t SPI_CPOL;			// possible values from @SPI_CPOL
	uint8_t SPI_SSM;			// possible values from @SPI_SOFTWARE_MANAGEMENT
	uint8_t SPI_SClkSpeed;		// possible values from @SPI_SCLK_SPEED
} SPI_Config_t;

typedef struct {
	SPI_RegDef_t *pSPIx;		// pointer holds the base address of the SPI peripheral (x = 1, 2, 3, 4)
	SPI_Config_t SPIConfig;		// holds the SPI configuration settings
	uint8_t *pTxBuffer;			// stores the application's tx buffer address in interrupt mode
	uint8_t *pRxBuffer;			// stores the application's rx buffer address in interrupt mode
	uint8_t txLen;				// stores the length of application's data (in bytes) being sent in interrupt mode
	uint8_t rxLen;				// stores the length of application's data (in bytes) being received in interrupt mode
	uint8_t txState;			// stores the current state of transmission in interrupt mode
	uint8_t rxState;			// stores the current state of reception in interrupt mode
} SPI_Handle_t;

/*
 * @SPI_DEVICE_MODE
 * Possible values of SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER	1
#define SPI_DEVICE_MODE_SLAVE	0

/*
 * @SPI_BUS_CONFIG
 * Possible values of SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD					1
#define SPI_BUS_CONFIG_HD					2
//#define SPI_BUS_CONFIG_SIMPLEX_TXONLY		3	// not required as the full-duplex configuration is same as simplex_TxOnly
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY		3

/*
 * @SPI_CPHA
 * Possible values of SPI_CPHA
 */
#define SPI_DFF_8BITS	0
#define SPI_DFF_16BITS	1

/*
 * @SPI_DATA_FRAME
 * Possible values of SPI_DataFrame
 */
#define SPI_CPHA_LOW	0
#define SPI_CPHA_HIGH	1

/*
 * @SPI_CPOL
 * Possible values of SPI_CPOL
 */
#define SPI_CPOL_LOW	0
#define SPI_CPOL_HIGH	1

/*
 * @SPI_SSM
 * Possible values of SPI_SSM
 */
#define SPI_SSM_DI		0
#define SPI_SSM_EN		1

/*
 * @SPI_SCLK_SPEED
 * Possible values of SPI_SClkSpeed
 */
#define SPI_SCLK_SPEED_DIV2			0
#define SPI_SCLK_SPEED_DIV4			1
#define SPI_SCLK_SPEED_DIV8			2
#define SPI_SCLK_SPEED_DIV16		3
#define SPI_SCLK_SPEED_DIV32		4
#define SPI_SCLK_SPEED_DIV64		5
#define SPI_SCLK_SPEED_DIV128		6
#define SPI_SCLK_SPEED_DIV256		7

/*
 * SPI status register flag definitions
 */
#define SPI_RXNE_FLAG	(1 << SPI_SR_RXNE)
#define SPI_TXE_FLAG	(1 << SPI_SR_TXE)
#define SPI_CHSIDE_FLAG	(1 << SPI_SR_CHSIDE)
//#define SPI_UDR_FLAG	(1 << SPI_SR_UDR)
//#define SPI_CRCERR_FLAG	(1 << SPI_SR_CRCERR)
//#define SPI_MODF_FLAG	(1 << SPI_SR_MODF)
#define SPI_OVR_FLAG	(1 << SPI_SR_OVR)
#define SPI_BUSY_FLAG	(1 << SPI_SR_BSY)
//#define SPI_FRE_FLAG	(1 << SPI_SR_FRE)

/*
 * SPI status flags when communication is done in interrupt mode
 */
#define SPI_READY		0
#define SPI_BUSY_IN_RX	1
#define SPI_BUSY_IN_TX	2

/*
 * Possible SPI application events
 */
#define SPI_EVENT_TX_CMPLT	1
#define SPI_EVENT_RX_CMPLT	2
#define SPI_EVENT_OVR_ERROR	3
#define SPI_EVENT_CRC_ERROR	4

/* SPI API prototypes */
/* SPI peripheral clock configuration */
void SPI_PeriClock_Control(SPI_RegDef_t* pSPIx, uint8_t EnorDi);

/* SPI init and de-init */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/* SPI data send and receive */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t* pTxBuffer, uint32_t length);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t* pRxBuffer, uint32_t length);

uint8_t SPI_SendData_IT(SPI_Handle_t *pSPIHandle, uint8_t* pTxBuffer, uint32_t length);
uint8_t SPI_ReceiveData_IT(SPI_Handle_t *pSPIHandle, uint8_t* pRxBuffer, uint32_t length);

/* SPI interrupts */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint32_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandler(SPI_Handle_t *pSPIHandle);

/*
 * Other peripheral control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t* pSPIx, uint32_t FlagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Application callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent);

#endif /* INC_USR_STM32F413XX_SPI_DRIVER_H_ */
