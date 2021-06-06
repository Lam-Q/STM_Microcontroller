################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/STM32f4xx_gpio_driver.c 

OBJS += \
./drivers/Src/STM32f4xx_gpio_driver.o 

C_DEPS += \
./drivers/Src/STM32f4xx_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/STM32f4xx_gpio_driver.o: ../drivers/Src/STM32f4xx_gpio_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DSTM32F407VGTx -DDEBUG -c -I"C:/Users/lamqi/OneDrive/Desktop/STM32F407VG/EmbeddedSystem/MCU1/STM32f4xx_drivers/drivers/Inc" -I../Inc -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/STM32f4xx_gpio_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

