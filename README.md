# STM32f413xx-drivers-from-scratch

The drivers for the following peripherals in any STM32F413XX are implemented:\
1.) GPIOx (x = A, B, C, D, E, F, G, H)\
2.) SPIx (x = 1, 2, 3, 4)\
3.) I2Cx (x = 1, 2, 3)\
4.) USARTx (x = 1, 2, 3, 6)\
5.) UARTx (x = 4, 5, 7, 8, 9, 10)

![Drivers](https://github.com/NatsuDrag9/STM32f413xx-drivers-from-scratch/assets/38008375/4df6c96d-089f-4864-8483-4b99ee9bdeff)

![GPIO_APIs](https://github.com/NatsuDrag9/STM32f413xx-drivers-from-scratch/assets/38008375/5636dc0a-2f87-444f-bacf-a1ad5066183b)

![I2C_APIs](https://github.com/NatsuDrag9/STM32f413xx-drivers-from-scratch/assets/38008375/c75318f2-41b4-4989-9de4-9a8ace466826)

![SPI_APIs](https://github.com/NatsuDrag9/STM32f413xx-drivers-from-scratch/assets/38008375/52c65bed-8ffe-4040-b86b-8f4c97c8e5eb)

![USART_APIs](https://github.com/NatsuDrag9/STM32f413xx-drivers-from-scratch/assets/38008375/0fb06062-3e4b-4a37-847f-d4f5e83cec41)

---

The MCU header file usr_stm32f413xx.h at ./drivers/inc/ contains:\
1.) Base addresses of flash, SRAM1, ROM, peripheral buses and the various peripherals\
2.) Peripheral register definition structure and peripheral definitions\
3.) System clock and peripheral clock enable/disable\
4.) Peripheral interrupt position number in the vector table\
5.) Bit positions of various registers for all peripherals

The header file for peripheral driver implementation at ./drivers/inc/ contains:\
1.) Peripheral handle structure\
2.) Peripheral configuration structure\
3.) API prototypes\
4.) And macros for various modes that need to be configured

![GPIO_Handle_Structure](https://github.com/NatsuDrag9/STM32f413xx-drivers-from-scratch/assets/38008375/0d869afd-3f0c-4b0f-8ffb-e681c032d042)

![SPI_Handle_Structure](https://github.com/NatsuDrag9/STM32f413xx-drivers-from-scratch/assets/38008375/fe6b1243-ce0e-47a7-a35a-10c582975397)

![I2C_Handle_Structure](https://github.com/NatsuDrag9/STM32f413xx-drivers-from-scratch/assets/38008375/aab726c1-97e4-4c83-b8f7-e329de8ecac4)

![USART_Handle_Structure](https://github.com/NatsuDrag9/STM32f413xx-drivers-from-scratch/assets/38008375/60bdfcf6-abda-4152-93d2-9b1fbf090f73)

The drivers were tested on STM32F413ZH Nucleo-144 containing ARM Cortex M4 and the communication protocols were tested with an Arduino Uno Rev3.

---

The reference manual RM0430 for ARM cortex M4 can be found <a href="https://www.st.com/en/microcontrollers-microprocessors/stm32f413zh.html#documentation)https://www.st.com/en/microcontrollers-microprocessors/stm32f413zh.html#documentation">here</a>. \
The links to datasheet and usermanual of the board can be found <a href="https://community.element14.com/products/devtools/product-pages/w/documents/22502/stm32-nucleo-144-development-board-with-stm32f413zh-mcu">here</a>.

I couldn't find all the documents on the board's product page at ST's official website.
