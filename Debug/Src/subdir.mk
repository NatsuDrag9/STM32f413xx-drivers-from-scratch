################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/ex10_usart_with_arduino.c \
../Src/sysmem.c 

OBJS += \
./Src/ex10_usart_with_arduino.o \
./Src/sysmem.o 

C_DEPS += \
./Src/ex10_usart_with_arduino.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32 -DNUCLEO_F413ZH -DSTM32F413ZHTx -DSTM32F4 -c -I../Inc -I"/home/rohitimandi/Desktop/Rohit/Personal/Partial_Work_and_Learning/Embedded Systems/STM32_Projects/Udemy_STM32F4xx_Drivers/Inc" -I"/home/rohitimandi/Desktop/Rohit/Personal/Partial_Work_and_Learning/Embedded Systems/STM32_Projects/Udemy_STM32F4xx_Drivers/drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Src

clean-Src:
	-$(RM) ./Src/ex10_usart_with_arduino.d ./Src/ex10_usart_with_arduino.o ./Src/sysmem.d ./Src/sysmem.o

.PHONY: clean-Src

