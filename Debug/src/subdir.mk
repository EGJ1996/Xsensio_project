################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/main.c \
../src/nfc.c \
../src/sensor.c \
../src/spi_ll_com.c \
../src/stm32l4xx_hal_msp.c \
../src/stm32l4xx_it.c \
../src/syscalls.c \
../src/system_stm32l4xx.c \
../src/uart_com.c 

OBJS += \
./src/main.o \
./src/nfc.o \
./src/sensor.o \
./src/spi_ll_com.o \
./src/stm32l4xx_hal_msp.o \
./src/stm32l4xx_it.o \
./src/syscalls.o \
./src/system_stm32l4xx.o \
./src/uart_com.o 

C_DEPS += \
./src/main.d \
./src/nfc.d \
./src/sensor.d \
./src/spi_ll_com.d \
./src/stm32l4xx_hal_msp.d \
./src/stm32l4xx_it.d \
./src/syscalls.d \
./src/system_stm32l4xx.d \
./src/uart_com.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DSTM32 -DSTM32L4 -DSTM32L443CCTx -DDEBUG -DSTM32L443xx -DUSE_HAL_DRIVER -I"/home/emi/Desktop/STM32Toolchain/test16/HAL_Driver/Inc/Legacy" -I"/home/emi/Desktop/STM32Toolchain/test16/inc" -I"/home/emi/Desktop/STM32Toolchain/test16/CMSIS/device" -I"/home/emi/Desktop/STM32Toolchain/test16/CMSIS/core" -I"/home/emi/Desktop/STM32Toolchain/test16/HAL_Driver/Inc" -O0 -g3 -Wall -fmessage-length=0 -ffunction-sections -c -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


