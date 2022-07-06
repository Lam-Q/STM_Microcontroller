################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/003Button_Interrupt.c 

OBJS += \
./Src/003Button_Interrupt.o 

C_DEPS += \
./Src/003Button_Interrupt.d 


# Each subdirectory must supply rules for building sources it contributes
Src/003Button_Interrupt.o: ../Src/003Button_Interrupt.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"C:/Users/lamqi/OneDrive/Desktop/C_Programs/STM_Microcontroller/Stm32f407VG_drivers/drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/003Button_Interrupt.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

