################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/src/stm32f407VG_Gpio_driver.c 

OBJS += \
./drivers/src/stm32f407VG_Gpio_driver.o 

C_DEPS += \
./drivers/src/stm32f407VG_Gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/src/stm32f407VG_Gpio_driver.o: ../drivers/src/stm32f407VG_Gpio_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I../Inc -I"C:/Users/lamqi/OneDrive/Desktop/C_Programs/STM_Microcontroller/Stm32f407VG_drivers/drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/src/stm32f407VG_Gpio_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

