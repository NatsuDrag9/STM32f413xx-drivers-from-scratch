/*
 * stm32f413xx.h
 *
 *	MCU specific data
 *
 *	This header contains macros defining base addresses
 *	of memory, clock and device peripherals of STM32F$13xx.
 *
 *	Reference manual of STM32F413ZH with ARM Cortex M4 is
 *	referred to as STM32F4_RM.
 *	Datasheet and Usermanual of STM32F$13ZH are referred
 *	to as STM32F4_DT and STM32F4_UM
 *
 */

#ifndef INC_UDEMY_STM32F413XX_H_
#define INC_UDEMY_STM32F413XX_H_

#include <stdint.h>

/* ******************************************* PROCESSOR SPECIFIC DETAILS ******************************************
 *
 * ARM cortex Mx processor NVIC ISERx register base addresses
 *
 */

#define NVIC_ISER0			((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1			((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2			((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3			((volatile uint32_t*)0xE000E10C)

/*
 * ARM cortex Mx processor NVIC ICERx register base addresses
 */

#define NVIC_ICER0			((volatile uint32_t*)0xE000E180)
#define NVIC_ICER1			((volatile uint32_t*)0xE000E184)
#define NVIC_ICER2			((volatile uint32_t*)0xE000E188)
#define NVIC_ICER3			((volatile uint32_t*)0xE000E18C)

/*
 * ARM cortex Mx processor NVIC IPRx (x = 0 to 59) register base address
 */
#define NVIC_IPRx_BASEADDR	((volatile uint32_t*)0xE000E400)

/*
 * ARM cortex Mx processor NVIC IPRx register 8-bit field implementation
 * Only the first four MSB bits are implemented and can be used to set
 * priority
 */
# define NO_PR_BITS_IMPLEMENTED	4		//

/* Base addresses of memory */
#define FLASH_BASEADDR		0x08000000U	// taken from section 2.6 on pg-65 in STM32F4_RM
#define SRAM1_BASEADDR		0x20000000U	// size = 256kB taken from section 2.3 on pg-62 in STM32F4_RM

/*
 * If SRAM2 base address is not detected, compute it by adding 256 kB to SRAM1's baseaddress
 * SRAM2_BASEADDRESS = toHex(256 *1-24) + 0x20000000UL
 */
//#define SRAM2_BASEADDR	0x20040000UL	// size = 64 kB taken from section 2.3 on pg-62 in STM32F4_RM

#define SYSMEM_BASEADDR		0x1FFF0000U	// system memory (which is ROM) is taken from section 2.6 on pg-65 in STM32F4_RM

/*
 * Base addresses of buses taken from
 * section 2.2.2 on pg-59 and pg-60 in STM32F4_RM
 */
#define PERIPH_BASE			0x40000000U
#define APB1PERIPH_BASE		PERIPH_BASE
#define APB2PERIPH_BASE		0x40010000U
#define AHB1PERIPH_BASE		0x40020000U
#define AHB2PERIPH_BASE		0x50000000U

/*
 * Base addresses of peripherals on AHB1 bus
 * taken from section 2.2.2 on pg-60 in STM32F4_RM
 */
#define GPIOA_BASEADDR		(AHB1PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR		(AHB1PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR		(AHB1PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR		(AHB1PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR		(AHB1PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR		(AHB1PERIPH_BASE + 0x1400)
#define GPIOG_BASEADDR		(AHB1PERIPH_BASE + 0x1800)
#define GPIOH_BASEADDR		(AHB1PERIPH_BASE + 0x1C00)

#define RCC_BASEADDR		(AHB1PERIPH_BASE + 0x3800)

/*
 * Base addresses of peripherals on APB1 bus
 * taken from section 2.2.2 on pg-61 in STM32F4_RM
 */
#define I2C1_BASEADDR		(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR		(APB1PERIPH_BASE + 0x5800)
#define I2C3_BASEADDR		(APB1PERIPH_BASE + 0x5C00)

#define SPI2_BASEADDR		(APB1PERIPH_BASE + 0x3800)
#define SPI3_BASEADDR		(APB1PERIPH_BASE + 0x3C00)

#define USART2_BASEADDR		(APB1PERIPH_BASE + 0x4400)
#define USART3_BASEADDR		(APB1PERIPH_BASE + 0x4800)
#define UART4_BASEADDR		(APB1PERIPH_BASE + 0x4C00)
#define UART5_BASEADDR		(APB1PERIPH_BASE + 0x5000)
#define UART7_BASEADDR		(APB1PERIPH_BASE + 0x7800)
#define UART8_BASEADDR		(APB1PERIPH_BASE + 0x7C00)

/*
 *  Base address of peripherals on APB2 bus
 *  taken from section 2.2.2 on pg-60 in STM32F4_RM
 */
#define EXTI_BASEADDR		(APB2PERIPH_BASE + 0x3C00)

#define SPI1_BASEADDR		(APB2PERIPH_BASE + 0x3000)
#define SPI4_BASEADDR		(APB2PERIPH_BASE + 0x3400)

#define SYSCFG_BASEADDR		(APB2PERIPH_BASE + 0x3800)

#define USART1_BASEADDR		(APB2PERIPH_BASE + 0x1000)
#define USART6_BASEADDR		(APB2PERIPH_BASE + 0x1400)
#define UART9_BASEADDR		(APB2PERIPH_BASE + 0x1800)
#define UART10_BASEADDR		(APB2PERIPH_BASE + 0x1C00)

/*
 * Peripheral register structure definition of GPIOx (x = A, B, ... H)
 * taken from section 7.4.11 on pg-202 in STM32F43_RM
 */
typedef struct {
	volatile uint32_t MODER;		// GPIO port mode register address offset is 0x00
	volatile uint32_t OTYPER;		// GPIO port output type register address offset is 0x04
	volatile uint32_t OSPEEDR;		// GPIO port output speed register address offset is 0x08
	volatile uint32_t PUPDR;		// GPIO port pull-up/pull-down register address offset is 0x0C
	volatile uint32_t IDR;			// GPIO port input data register address offset is 0x10
	volatile uint32_t ODR;			// GPIO port output data register address offset is 0x14
	volatile uint32_t BSRR;			// GPIO port bit set/reset register address offset is 0x18
	volatile uint32_t LCKR;			// GPIO port configuration lock register address offset is 0x1C
	volatile uint32_t AFR[2];		// GPIO alternate function low and high registers. Address offset of AFRL is 0x20 and AFRH is 0x24
} GPIO_RegDef_t;

/*
 * Peripheral register structure definition of RCC
 * taken from section 6.3.3.0 on pg-181 in STM32F4_RM
 */
typedef struct {
	volatile uint32_t CR;			// clock control register address offset is 0x00
	volatile uint32_t PLLCFGR;		// PLL configuration register address offset is 0x04
	volatile uint32_t CFGR;			// clock configuration register address offset is 0x08
	volatile uint32_t CIR;			// clock interrupt register address offset is 0x0C
	volatile uint32_t AHB1RSTR;		// AHB1 peripheral reset register address offset is 0x10
	volatile uint32_t AHB2RSTR;		// AHB2 peripheral reset register address offset is 0x14
	volatile uint32_t AHB3RSTR;		// AHB3 peripheral reset register address offset is 0x18

	volatile uint32_t RESERVED1;	// address offset is 0x1C

	volatile uint32_t APB1RSTR;		// APB1 peripheral reset register address offset is 0x20
	volatile uint32_t APB2RSTR;		// APB2 peripheral reset register address offset is 0x24

	volatile uint32_t RESERVED2;		// address offset is 0x28
	volatile uint32_t RESERVED3;		// address offset is 0x2C

	volatile uint32_t AHB1ENR;		// AHB1 peripheral clock enable register address offset is 0x30
	volatile uint32_t AHB2ENR;		// AHB2 peripheral clock enable register address offset is 0x34
	volatile uint32_t AHB3ENR;		// AHB3 peripheral clock enable register address offset is 0x38

	volatile uint32_t RESERVED4;	// address offset is 0x3C

	volatile uint32_t APB1ENR;		// APB1 peripheral clock enable register address offset is 0x40
	volatile uint32_t APB2ENR;		// APB2 peripheral clock enable register address offset is 0x44

	volatile uint32_t RESERVED5;	// address offset is 0x48
	volatile uint32_t RESERVED6;	// address offset is 0x4C

	volatile uint32_t AHB1LPENR;	// AHB1 peripheral clock enabled in low power mode register address offset is 0x50
	volatile uint32_t AHB2LPENR;	// AHB2 peripheral clock enabled in low power mode register address offset is 0x54
	volatile uint32_t AHB3LPENR;	// AHB3 peripheral clock enabled in low power mode register address offset is 0x58

	volatile uint32_t RESERVED7;	// address offset is 0x5C

	volatile uint32_t APB1LPENR;	// APB1 peripheral clock enabled in low power mode register address offset is 0x60
	volatile uint32_t APB2LPENR;	// APB2 peripheral clock enabled in low power mode register address offset is 0x64

	volatile uint32_t RESERVED8;	// address offset is 0x68
	volatile uint32_t RESERVED9;	// address offset is 0x6C

	volatile uint32_t BDCR;			// backup domain control register address offset is 0x70
	volatile uint32_t CSR;			// clock control & status register address offset is 0x74

	volatile uint32_t RESERVED10;	// address offset is 0x78
	volatile uint32_t RESERVED11;	// address offset is 0x7C

	volatile uint32_t SSCGR;		// spread spectrum clock generation register address offset is 0x80
	volatile uint32_t PLLI2SCFGR;	// PLLI2S configuration register address offset is 0x84

	volatile uint32_t RESERVED12;	// address offset is 0x88

	volatile uint32_t DCKCFGR;		// dedicated clocks configuration register 1 address offset is 0x8C
	volatile uint32_t CKGATENR;		// clocks gated enable register address offset is 0x90
	volatile uint32_t DCKCFGR2;		// dedicated clocks configuration register 2 address offset is 0x94
} RCC_RegDef_t;


/*
 * Peripheral register structure definition of EXTI
 * taken from section 10.3 on pg-258 in STM32F43_RM
 */
typedef struct {
	volatile uint32_t IMR;		// EXTI interrupt mask register address offset is 0x00
	volatile uint32_t EMR;		// EXTI event mask register address offset is 0x04
	volatile uint32_t RTSR;		// EXTI rising trigger selection register address offset is 0x08
	volatile uint32_t FTSR;		// EXTI falling trigger selection register address offset is 0x0C
	volatile uint32_t SWIER;	// EXTI software interrupt event register address offset is 0x10
	volatile uint32_t PR;		// EXTI pending register address offset is 0x14
} EXTI_RegDef_t;


/*
 * Peripheral register structure definition of SYSCFG
 * taken from section 8.2 on pg-204 in STM32F43_RM
 */
typedef struct {
	volatile uint32_t MEMRMP;		// SYSCFG memory remap register address offset is 0x00
	volatile uint32_t PMC;			// SYSCFG peripheral mode configuration register address offset is 0x04
	volatile uint32_t EXTICR[4];		// SYSCFG external interrupt configuration register 1 address offset is 0x08 - 0x14
//	volatile uint32_t EXTICR2;		// SYSCFG external interrupt configuration register 2 address offset is 0x0C
//	volatile uint32_t EXTICR3;		// SYSCFG external interrupt configuration register 3 address offset is 0x10
//	volatile uint32_t EXTICR4;		// SYSCFG external interrupt configuration register 4 address offset is 0x14
	volatile uint32_t CFGR2;		//  SYSCFG configuration register 2 address offset is 0x1C
	volatile uint32_t CMPCR;		//  SYSCFG compensation cell control register address offset is 0x20
	volatile uint32_t CFGR;			//  SYSCFG configuration register address offset is 0x2C
	volatile uint32_t MCHDLYCR;		//  SYSCFG DFSDM Multi-channel delay control register address offset is 0x30

} SYSCFG_RegDef_t;


/*
 * Peripheral register structure definition of SPI
 * taken from section 29.7 on pg-976 in STM32F43_RM
 */
typedef struct {
	volatile uint32_t CR1;		// SPI control register 1 address offset is 0x00
	volatile uint32_t CR2;		// SPI control register 2 address offset is 0x04
	volatile uint32_t SR;		// SPI status register address offset is 0x08
	volatile uint16_t DR;		// SPI data register address offset is 0x0C
	volatile uint32_t CRCPR;	// SPI CRC polynomial register address offset is 0x10
	volatile uint32_t RXCRCR;	// SPI RX CRC register address offset is 0x14
	volatile uint32_t TXCRCR;	// SPI TX CRC register address offset is 0x18
	volatile uint32_t I2SCFGR;	// SPI_I2S configuration register address offset is 0x1C
	volatile uint32_t I2SPR;	// SPI_I2S prescalar register address offset is 0x20
} SPI_RegDef_t;


/*
 * Peripheral register structure definition of I2C
 * taken from section 27.6 on pg-869 in STM32F43_RM
 */
typedef struct {
	volatile uint32_t CR1;		// I2C control register 1 address offset is 0x00
	volatile uint32_t CR2;		// I2C control register 2 address offset is 0x04
	volatile uint32_t OAR1;		// I2C own address register 1 address offset is 0x08
	volatile uint16_t OAR2;		// I2C own address register 2 address offset is 0x0C
	volatile uint32_t DR;		// I2C DR register address offset is 0x10
	volatile uint32_t SR1;		// I2C status register 1 address offset is 0x14
	volatile uint32_t SR2;		// I2C status register 2 address offset is 0x18
	volatile uint32_t CCR;		// I2C clock control register address offset is 0x1C
	volatile uint32_t TRISE;	// I2C TRISE register address offset is 0x20
	volatile uint32_t FLTR;		// I2C FLTR register address offset is 0x24
} I2C_RegDef_t;


/*
 * Peripheral structure register definition of USART
 * taken from section 28.6 on pg-924 in STM32F413XX_RM
 */
typedef struct {
	volatile uint32_t SR;		// USART status register address offset is 0x00
	volatile uint32_t DR;		// USART data register address offset is 0x04
	volatile uint32_t BRR;		// USART baud rate register address offset is 0x08
	volatile uint32_t CR1;		// USART control register 1 address offset is 0x0C
	volatile uint32_t CR2;		// USART control register 2 address offset is 0x10
	volatile uint32_t CR3;		// USART control register 3 address offset is 0x14
	volatile uint32_t GTPR;		// USART guart time and prescalar register address offset is 0x18
} USART_RegDef_t;

/* Peripheral definitions -- peripheral base addresses typecasted to xxx_RegDef_t */
#define GPIOA				((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB				((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC				((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD				((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE				((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF				((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG				((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH				((GPIO_RegDef_t*) GPIOH_BASEADDR)

#define RCC					((RCC_RegDef_t*) RCC_BASEADDR)
#define EXTI				((EXTI_RegDef_t*) EXTI_BASEADDR)
#define SYSCFG				((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1				((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2				((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3				((SPI_RegDef_t*) SPI3_BASEADDR)
#define SPI4				((SPI_RegDef_t*) SPI4_BASEADDR)

#define I2C1				((I2C_RegDef_t*) I2C1_BASEADDR)
#define I2C2				((I2C_RegDef_t*) I2C2_BASEADDR)
#define I2C3				((I2C_RegDef_t*) I2C3_BASEADDR)

#define USART1				((USART_RegDef_t*) USART1_BASEADDR)
#define USART2				((USART_RegDef_t*) USART2_BASEADDR)
#define USART3				((USART_RegDef_t*) USART3_BASEADDR)
#define USART6				((USART_RegDef_t*) USART6_BASEADDR)

#define UART4				((USART_RegDef_t*) UART4_BASEADDR)
#define UART5				((USART_RegDef_t*) UART5_BASEADDR)
#define UART7				((USART_RegDef_t*) UART7_BASEADDR)
#define UART8				((USART_RegDef_t*) UART8_BASEADDR)
#define UART9				((USART_RegDef_t*) UART9_BASEADDR)
#define UART10				((USART_RegDef_t*) UART10_BASEADDR)

/*
 * Clock enable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))

/*
 * Clock enable macros for I2C peripherals
 */
#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1 << 23))

/*
 * Clock enable macros for SPI peripherals
 */
#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()		(RCC->APB2ENR |= (1 << 13))

/*
 * Clock enable macros for USART peripherals
 */
#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1 << 18))
#define USART6_PCLK_EN()	(RCC->APB2ENR |= (1 << 5))

/*
 * Clock enable macros for UART peripherals
 */
#define UART4_PCLK_EN()		(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()		(RCC->APB1ENR |= (1 << 20))
#define UART7_PCLK_EN()		(RCC->APB1ENR |= (1 << 30))
#define UART8_PCLK_EN()		(RCC->APB1ENR |= (1 << 31))
#define UART9_PCLK_EN()		(RCC->APB2ENR |= (1 << 6))
#define UART10_PCLK_EN()	(RCC->APB2ENR |= (1 << 7))

/*
 * Clock enable macros for SYSCNG peripheral
 */
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))


/*
 * Clock disable macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 7))


/*
 * Clock disable macros for I2C peripherals
 */
#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 23))

/*
 * Clock disable macros for SPI peripherals
 */
#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 13))

/*
 * Clock enable macros for USART peripherals
 */
#define USART1_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 18))
#define USART6_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 5))

/*
 * Clock disable macros for UART peripherals
 */
#define UART4_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 20))
#define UART7_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 30))
#define UART8_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 31))
#define UART9_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 6))
#define UART10_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 7))

/*
 * Clock disable macros for SYSCNG peripheral
 */
#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 14))


/*
 * GPIO register reset macros
 * Resets by setting the corresponding bit-field to '1' then
 * clears the bit-field for the port to be used again
 */
#define GPIOA_REG_RESET()	do {(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));} while(0)
#define GPIOB_REG_RESET()	do {(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));} while(0)
#define GPIOC_REG_RESET()	do {(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));} while(0)
#define GPIOD_REG_RESET()	do {(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));} while(0)
#define GPIOE_REG_RESET()	do {(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));} while(0)
#define GPIOF_REG_RESET()	do {(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));} while(0)
#define GPIOG_REG_RESET()	do {(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));} while(0)
#define GPIOH_REG_RESET()	do {(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));} while(0)

/*
 * SPI register reset macros
 * Resets by setting the corresponding bit-field to '1' then
 * clears the bit-field for the peripheral to be used again
 */
#define SPI1_REG_RESET()	do {(RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12));} while(0)
#define SPI2_REG_RESET()	do {(RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14));} while(0)
#define SPI3_REG_RESET()	do {(RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15));} while(0)
#define SPI4_REG_RESET()	do {(RCC->APB2RSTR |= (1 << 13)); (RCC->APB2RSTR &= ~(1 << 13));} while(0)

/*
 * I2C register reset macros
 * Resets by setting the corresponding bit-field to '1' then
 * clears the bit-field for the peripheral to be used again
 */
#define I2C1_REG_RESET()	do {(RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21));} while(0)
#define I2C2_REG_RESET()	do {(RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22));} while(0)
#define I2C3_REG_RESET()	do {(RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR &= ~(1 << 23));} while(0)

/*
 * USART and UART register reset macros
 * Resets by setting the corresponding bit-field to '1' then
 * clears the bit-field for the peripheral to be used again
 */
#define USART1_REG_RESET()	do {(RCC->APB2RSTR |= (1 << 4)); (RCC->APB2RSTR &= ~(1 << 4));} while(0)
#define USART2_REG_RESET()	do {(RCC->APB1RSTR |= (1 << 17)); (RCC->APB1RSTR &= ~(1 << 17));} while(0)
#define USART3_REG_RESET()	do {(RCC->APB1RSTR |= (1 << 18)); (RCC->APB1RSTR &= ~(1 << 18));} while(0)
#define USART6_REG_RESET()	do {(RCC->APB2RSTR |= (1 << 5)); (RCC->APB2RSTR &= ~(1 << 5));} while(0)

#define UART4_REG_RESET()	do {(RCC->APB1RSTR |= (1 << 19)); (RCC->APB1RSTR &= ~(1 << 19));} while(0)
#define UART5_REG_RESET()	do {(RCC->APB1RSTR |= (1 << 20)); (RCC->APB1RSTR &= ~(1 << 20));} while(0)
#define UART7_REG_RESET()	do {(RCC->APB1RSTR |= (1 << 30)); (RCC->APB1RSTR &= ~(1 << 30));} while(0)
#define UART8_REG_RESET()	do {(RCC->APB1RSTR |= (1 << 31)); (RCC->APB1RSTR &= ~(1 << 31));} while(0)
#define UART9_REG_RESET()	do {(RCC->APB2RSTR |= (1 << 6)); (RCC->APB2RSTR &= ~(1 << 6));} while(0)
#define UART10_REG_RESET()	do {(RCC->APB2RSTR |= (1 << 7)); (RCC->APB2RSTR &= ~(1 << 7));} while(0)

/* Macros for IRQ numbers of EXTIx lines */
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40

/* Macros for IRQ numbers of SPIx lines */
#define IRQ_NO_SPI1			35
#define IRQ_NO_SPI2			36
#define IRQ_NO_SPI3			51
#define IRQ_NO_SPI4			84

/* Macros for IRQ numbers of I2Cx lines */
#define IRQ_NO_I2C1_EVT		31
#define IRQ_NO_I2C1_ERR		32
#define IRQ_NO_I2C2_EVT		33
#define IRQ_NO_I2C2_ERR		34
#define IRQ_NO_I2C3_EVT		72
#define IRQ_NO_I2C3_ERR		73

/* Macros for IRQ numbers of USARTx and UARTx lines */
#define IRQ_NO_USART1		37
#define IRQ_NO_USART2		38
#define IRQ_NO_USART3		39
#define IRQ_NO_USART6		71

#define IRQ_NO_UART4		52
#define IRQ_NO_UART5		53
#define IRQ_NO_UART7		82
#define IRQ_NO_UART8		83
#define IRQ_NO_UART9		88
#define IRQ_NO_UART10		89

/* Macros for NVIC interrupt priority levels */
#define NVIC_IRQ_PRIORITY0	0
#define NVIC_IRQ_PRIORITY1	1
#define NVIC_IRQ_PRIORITY2	2
#define NVIC_IRQ_PRIORITY3	3
#define NVIC_IRQ_PRIORITY4	4
#define NVIC_IRQ_PRIORITY5	5
#define NVIC_IRQ_PRIORITY6	6
#define NVIC_IRQ_PRIORITY7	7
#define NVIC_IRQ_PRIORITY8	8
#define NVIC_IRQ_PRIORITY9	9
#define NVIC_IRQ_PRIORITY10	10
#define NVIC_IRQ_PRIORITY11	11
#define NVIC_IRQ_PRIORITY12	12
#define NVIC_IRQ_PRIORITY13	13
#define NVIC_IRQ_PRIORITY14	14
#define NVIC_IRQ_PRIORITY15	15

/* General macros used by other files */
#define ENABLE			1
#define DISABLE			0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#define FLAG_RESET		RESET
#define FLAG_SET		SET

#define GPIO_BASEADDR_TO_PORT_CODE(x)	((x == GPIOA) ? 0 :\
										 (x == GPIOB) ? 1 :\
										 (x == GPIOC) ? 2 :\
										 (x == GPIOD) ? 3 :\
										 (x == GPIOE) ? 4 :\
										 (x == GPIOF) ? 5 :\
										 (x == GPIOG) ? 6 :\
										 (x == GPIOH) ? 7 : 0)

/* CR1 register bit position definitions of SPI peripheral */
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

/* CR2 register bit position definitions of SPI peripheral */
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

/* SR register bit position definitions of SPI peripheral */
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8

/* CR1 register bit position definitions of I2Cx (x = 1, 2, 3) peripheral */
#define I2C_CR1_PE			0
#define I2C_CR1_SMBUS		1
#define	I2C_CR1_SMBTYPE		3
#define I2C_CR1_ENARP		4
#define I2C_CR1_ENPEC		5
#define I2C_CR1_ENGC		6
#define I2C_CR1_NOSTRETCH	7
#define I2C_CR1_START		8
#define I2C_CR1_STOP		9
#define I2C_CR1_ACK			10
#define I2C_CR1_POS			11
#define I2C_CR1_PEC			12
#define I2C_CR1_ALERT		13
#define I2C_CR1_SWRST		15

/* CR2 register bit position definitions of I2Cx (x = 1, 2, 3) peripheral */
#define I2C_CR2_FREQ		0
#define I2C_CR2_ITERREN		8
#define I2C_CR2_ITEVTEN		9
#define I2C_CR2_ITBUFEN		10
#define I2C_CR2_DMAEN		11
#define I2C_CR2_LAST		12

/* SR1 register bit position definitions of I2Cx (x = 1, 2, 3) peripheral */
#define I2C_SR1_SB			0
#define I2C_SR1_ADDR		1
#define I2C_SR1_BTF			2
#define I2C_SR1_ADD10		3
#define I2C_SR1_STOPF		4
#define I2C_SR1_RXNE		6
#define I2C_SR1_TXE			7
#define I2C_SR1_BERR		8
#define I2C_SR1_ARLO		9
#define I2C_SR1_AF			10
#define I2C_SR1_OVR			11
#define I2C_SR1_PECERR		12
#define I2C_SR1_TIMEOUT		14
#define I2C_SR1_SMBALERT	15

/* SR2 register bit position definitions of I2Cx (x = 1, 2, 3) peripheral */
#define I2C_SR2_MSL			0
#define I2C_SR2_BUSY		1
#define I2C_SR2_TRA			2
#define I2C_SR2_GENCALL		4
#define I2C_SR2_SMBDEFAULT	5
#define I2C_SR2_SMBHOST		6
#define I2C_SR2_DUALF		7
#define I2C_SR2_PEC			8

/* CCR register bit position definitions of I2Cx (x = 1, 2, 3) peripheral */
#define I2C_CCR_CCR			0
#define I2C_CCR_DUTY		14
#define I2C_CCR_FS			15

/* TRISE register bit position definitions of I2Cx (x = 1, 2, 3) peripheral */
#define I2C_TRISE_TRISE		0

/*
 * SR register bit position definitions of USARTx (x = 1, 2, 3, 6)
 * and UARTx (x = 4, 5, 7, 8, 9, 10) peripheral
 */
#define USART_SR_PE			0
#define USART_SR_FE			1
#define USART_SR_NF			2
#define USART_SR_ORE		3
#define USART_SR_IDLE		4
#define USART_SR_RXNE		5
#define USART_SR_TC			6
#define USART_SR_TXE		7
#define USART_SR_LBD		8
#define USART_SR_CTS		9

/*
 * DR register bit position definitions of USARTx (x = 1, 2, 3, 6)
 * and UARTx (x = 4, 5, 7, 8, 9, 10) peripheral
 */
#define USART_DR		0

/*
 * BRR register bit position definitions of USARTx (x = 1, 2, 3, 6)
 * and UARTx (x = 4, 5, 7, 8, 9, 10) peripheral
 */
#define USART_BRR_DIV_FRACTION	0
#define USART_BRR_DIV_MANTISSA	4

/*
 * CR1 register bit position definitions of USARTx (x = 1, 2, 3, 6)
 * and UARTx (x = 4, 5, 7, 8, 9, 10) peripheral
 */
#define USART_CR1_SBK			0
#define USART_CR1_RWU			1
#define USART_CR1_RE			2
#define USART_CR1_TE			3
#define USART_CR1_IDLEIE		4
#define USART_CR1_RXNEIE		5
#define USART_CR1_TCIE			6
#define USART_CR1_TXIE			7
#define USART_CR1_PEIE			8
#define USART_CR1_PS			9
#define USART_CR1_PCE			10
#define USART_CR1_WAKE			11
#define USART_CR1_M				12
#define USART_CR1_UE			13
#define USART_CR1_OVER8			15

/*
 * CR3 register bit position definitions of USARTx (x = 1, 2, 3, 6)
 * and UARTx (x = 4, 5, 7, 8, 9, 10) peripheral
 */
#define USART_CR2_ADD			0
#define USART_CR2_LBDL			5
#define USART_CR2_LBDIE			6
#define USART_CR2_LBCL			8
#define USART_CR2_CPHA			9
#define USART_CR2_CPOL			10
#define USART_CR2_CLKEN			11
#define USART_CR2_STOP			12
#define USART_CR2_LINEN			14

/*
 * CR3 register bit position definitions of USARTx (x = 1, 2, 3, 6)
 * and UARTx (x = 4, 5, 7, 8, 9, 10) peripheral
 */
#define USART_CR3_EIE			0
#define USART_CR3_IREN			1
#define USART_CR3_IRLP			2
#define USART_CR3_HDSEL			3
#define USART_CR3_NACK			4
#define USART_CR3_SCEN			5
#define USART_CR3_DMAR			6
#define USART_CR3_DMAT			7
#define USART_CR3_RTSE			8
#define USART_CR3_CTSE			9
#define USART_CR3_CTSIE			10
#define USART_CR3_ONEBIT		11

/*
 *GTPR register bit position definitions of USARTx (x = 1, 2, 3, 6)
 * and UARTx (x = 4, 5, 7, 8, 9, 10) peripheral
 */
#define USART_GTPR_PSC			0
#define USART_GTPR_GT			7

/* Peripheral includes */
#include "usr_stm32f413xx_rcc.h"
#include "usr_stm32f413xx_gpio_driver.h"
#include "usr_stm32f413xx_spi_driver.h"
#include "usr_stm32f413xx_i2c_driver.h"
#include "usr_stm32f413xx_usart_driver.h"
#include <stddef.h>
#include <stdio.h>

#endif /* INC_UDEMY_STM32F413XX_H_ */
