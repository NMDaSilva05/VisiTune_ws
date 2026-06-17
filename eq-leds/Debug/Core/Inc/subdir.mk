################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Inc/color_values.c \
../Core/Inc/ws2812.c \
../Core/Inc/ws2812_demos.c 

OBJS += \
./Core/Inc/color_values.o \
./Core/Inc/ws2812.o \
./Core/Inc/ws2812_demos.o 

C_DEPS += \
./Core/Inc/color_values.d \
./Core/Inc/ws2812.d \
./Core/Inc/ws2812_demos.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Inc/%.o Core/Inc/%.su Core/Inc/%.cyclo: ../Core/Inc/%.c Core/Inc/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32L4R5xx -c -I../Core/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc -I../Drivers/STM32L4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32L4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Inc

clean-Core-2f-Inc:
	-$(RM) ./Core/Inc/color_values.cyclo ./Core/Inc/color_values.d ./Core/Inc/color_values.o ./Core/Inc/color_values.su ./Core/Inc/ws2812.cyclo ./Core/Inc/ws2812.d ./Core/Inc/ws2812.o ./Core/Inc/ws2812.su ./Core/Inc/ws2812_demos.cyclo ./Core/Inc/ws2812_demos.d ./Core/Inc/ws2812_demos.o ./Core/Inc/ws2812_demos.su

.PHONY: clean-Core-2f-Inc

