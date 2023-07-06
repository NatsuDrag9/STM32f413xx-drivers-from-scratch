################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/src/usr_stm32f413xx_gpio_driver.c \
../drivers/src/usr_stm32f413xx_i2c_driver.c \
../drivers/src/usr_stm32f413xx_rcc.c \
../drivers/src/usr_stm32f413xx_spi_driver.c \
../drivers/src/usr_stm32f413xx_usart_driver.c 

OBJS += \
./drivers/src/usr_stm32f413xx_gpio_driver.o \
./drivers/src/usr_stm32f413xx_i2c_driver.o \
./drivers/src/usr_stm32f413xx_rcc.o \
./drivers/src/usr_stm32f413xx_spi_driver.o \
./drivers/src/usr_stm32f413xx_usart_driver.o 

C_DEPS += \
./drivers/src/usr_stm32f413xx_gpio_driver.d \
./drivers/src/usr_stm32f413xx_i2c_driver.d \
./drivers/src/usr_stm32f413xx_rcc.d \
./drivers/src/usr_stm32f413xx_spi_driver.d \
./drivers/src/usr_stm32f413xx_usart_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/src/%.o: ../drivers/src/%.c drivers/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DNUCLEO_F413ZH -DSTM32F413ZHTx -DSTM32F4 -c -I../Inc -I"/home/rohitimandi/Desktop/Rohit/Personal/Partial_Work_and_Learning/Embedded Systems/STM32_Projects/Udemy_STM32F4xx_Drivers/Inc" -I"/home/rohitimandi/Desktop/Rohit/Personal/Partial_Work_and_Learning/Embedded Systems/STM32_Projects/Udemy_STM32F4xx_Drivers/drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-drivers-2f-src

clean-drivers-2f-src:
	-$(RM) ./drivers/src/usr_stm32f413xx_gpio_driver.d ./drivers/src/usr_stm32f413xx_gpio_driver.o ./drivers/src/usr_stm32f413xx_i2c_driver.d ./drivers/src/usr_stm32f413xx_i2c_driver.o ./drivers/src/usr_stm32f413xx_rcc.d ./drivers/src/usr_stm32f413xx_rcc.o ./drivers/src/usr_stm32f413xx_spi_driver.d ./drivers/src/usr_stm32f413xx_spi_driver.o ./drivers/src/usr_stm32f413xx_usart_driver.d ./drivers/src/usr_stm32f413xx_usart_driver.o

.PHONY: clean-drivers-2f-src

