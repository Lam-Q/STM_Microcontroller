################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/0015_Usart_tx_testing.c 

OBJS += \
./Src/0015_Usart_tx_testing.o 

C_DEPS += \
./Src/0015_Usart_tx_testing.d 


# Each subdirectory must supply rules for building sources it contributes
Src/0015_Usart_tx_testing.o: ../Src/0015_Usart_tx_testing.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"C:/Users/lamqi/OneDrive/Desktop/C_Programs/STM_Microcontroller/Stm32f407VG_drivers/drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/0015_Usart_tx_testing.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

