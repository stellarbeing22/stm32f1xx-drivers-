################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/src/stm32f103xx_gpio_drivers.c \
../drivers/src/stm32f103xx_i2c_drivers.c \
../drivers/src/stm32f103xx_spi_drivers.c 

OBJS += \
./drivers/src/stm32f103xx_gpio_drivers.o \
./drivers/src/stm32f103xx_i2c_drivers.o \
./drivers/src/stm32f103xx_spi_drivers.o 

C_DEPS += \
./drivers/src/stm32f103xx_gpio_drivers.d \
./drivers/src/stm32f103xx_i2c_drivers.d \
./drivers/src/stm32f103xx_spi_drivers.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/src/%.o drivers/src/%.su drivers/src/%.cyclo: ../drivers/src/%.c drivers/src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m3 -std=gnu11 -g3 -DDEBUG -DSTM32F103RBTx -DSTM32 -DSTM32F1 -c -I../Inc -I"/home/stellarbeing22/Desktop/STM32F103xxDriversFiles/stm32f103xxDrivers/drivers/inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-drivers-2f-src

clean-drivers-2f-src:
	-$(RM) ./drivers/src/stm32f103xx_gpio_drivers.cyclo ./drivers/src/stm32f103xx_gpio_drivers.d ./drivers/src/stm32f103xx_gpio_drivers.o ./drivers/src/stm32f103xx_gpio_drivers.su ./drivers/src/stm32f103xx_i2c_drivers.cyclo ./drivers/src/stm32f103xx_i2c_drivers.d ./drivers/src/stm32f103xx_i2c_drivers.o ./drivers/src/stm32f103xx_i2c_drivers.su ./drivers/src/stm32f103xx_spi_drivers.cyclo ./drivers/src/stm32f103xx_spi_drivers.d ./drivers/src/stm32f103xx_spi_drivers.o ./drivers/src/stm32f103xx_spi_drivers.su

.PHONY: clean-drivers-2f-src

