################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm32f746xx_gpio_driver.c \
../drivers/Src/stm32f746xx_i2c_driver.c \
../drivers/Src/stm32f746xx_spi_driver.c 

OBJS += \
./drivers/Src/stm32f746xx_gpio_driver.o \
./drivers/Src/stm32f746xx_i2c_driver.o \
./drivers/Src/stm32f746xx_spi_driver.o 

C_DEPS += \
./drivers/Src/stm32f746xx_gpio_driver.d \
./drivers/Src/stm32f746xx_i2c_driver.d \
./drivers/Src/stm32f746xx_spi_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/%.o drivers/Src/%.su drivers/Src/%.cyclo: ../drivers/Src/%.c drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DSTM32 -DNUCLEO_F746ZG -DSTM32F746ZGTx -DSTM32F7 -c -I../Inc -I"D:/workspace_1.18.0/stm32f7xx_drivers/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-drivers-2f-Src

clean-drivers-2f-Src:
	-$(RM) ./drivers/Src/stm32f746xx_gpio_driver.cyclo ./drivers/Src/stm32f746xx_gpio_driver.d ./drivers/Src/stm32f746xx_gpio_driver.o ./drivers/Src/stm32f746xx_gpio_driver.su ./drivers/Src/stm32f746xx_i2c_driver.cyclo ./drivers/Src/stm32f746xx_i2c_driver.d ./drivers/Src/stm32f746xx_i2c_driver.o ./drivers/Src/stm32f746xx_i2c_driver.su ./drivers/Src/stm32f746xx_spi_driver.cyclo ./drivers/Src/stm32f746xx_spi_driver.d ./drivers/Src/stm32f746xx_spi_driver.o ./drivers/Src/stm32f746xx_spi_driver.su

.PHONY: clean-drivers-2f-Src

