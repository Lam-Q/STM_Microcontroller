################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/001Led_Toggle.c \
../Src/002Led_Button.c \
../Src/003Button_Interrupt.c \
../Src/006spi_tx_Testing.c \
../Src/007spi_txonly_arduino.c 

OBJS += \
./Src/001Led_Toggle.o \
./Src/002Led_Button.o \
./Src/003Button_Interrupt.o \
./Src/006spi_tx_Testing.o \
./Src/007spi_txonly_arduino.o 

C_DEPS += \
./Src/001Led_Toggle.d \
./Src/002Led_Button.d \
./Src/003Button_Interrupt.d \
./Src/006spi_tx_Testing.d \
./Src/007spi_txonly_arduino.d 


# Each subdirectory must supply rules for building sources it contributes
Src/001Led_Toggle.o: ../Src/001Led_Toggle.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DSTM32 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/001Led_Toggle.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/002Led_Button.o: ../Src/002Led_Button.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DSTM32 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/002Led_Button.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/003Button_Interrupt.o: ../Src/003Button_Interrupt.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DSTM32 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/003Button_Interrupt.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/006spi_tx_Testing.o: ../Src/006spi_tx_Testing.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DSTM32 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/006spi_tx_Testing.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"
Src/007spi_txonly_arduino.o: ../Src/007spi_txonly_arduino.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -DSTM32 -DSTM32F4 -DSTM32F407VGTx -c -I../Inc -Os -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"Src/007spi_txonly_arduino.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

