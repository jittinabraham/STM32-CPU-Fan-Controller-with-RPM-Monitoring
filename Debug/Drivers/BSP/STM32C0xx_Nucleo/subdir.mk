################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/BSP/STM32C0xx_Nucleo/stm32c0xx_nucleo.c 

OBJS += \
./Drivers/BSP/STM32C0xx_Nucleo/stm32c0xx_nucleo.o 

C_DEPS += \
./Drivers/BSP/STM32C0xx_Nucleo/stm32c0xx_nucleo.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/BSP/STM32C0xx_Nucleo/%.o Drivers/BSP/STM32C0xx_Nucleo/%.su Drivers/BSP/STM32C0xx_Nucleo/%.cyclo: ../Drivers/BSP/STM32C0xx_Nucleo/%.c Drivers/BSP/STM32C0xx_Nucleo/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m0plus -std=gnu11 -g3 -DDEBUG -DUSE_NUCLEO_64 -DUSE_HAL_DRIVER -DSTM32C031xx -c -I../Core/Inc -I../Drivers/STM32C0xx_HAL_Driver/Inc -I../Drivers/STM32C0xx_HAL_Driver/Inc/Legacy -I../Drivers/BSP/STM32C0xx_Nucleo -I../Drivers/CMSIS/Device/ST/STM32C0xx/Include -I../Drivers/CMSIS/Include -I../Middlewares/Third_Party/FreeRTOS/Source/include/ -I../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/ -I../Middlewares/Third_Party/CMSIS/RTOS2/Include/ -I../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM0/ -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

clean: clean-Drivers-2f-BSP-2f-STM32C0xx_Nucleo

clean-Drivers-2f-BSP-2f-STM32C0xx_Nucleo:
	-$(RM) ./Drivers/BSP/STM32C0xx_Nucleo/stm32c0xx_nucleo.cyclo ./Drivers/BSP/STM32C0xx_Nucleo/stm32c0xx_nucleo.d ./Drivers/BSP/STM32C0xx_Nucleo/stm32c0xx_nucleo.o ./Drivers/BSP/STM32C0xx_Nucleo/stm32c0xx_nucleo.su

.PHONY: clean-Drivers-2f-BSP-2f-STM32C0xx_Nucleo

