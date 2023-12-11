################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (10.3-2021.10)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/STM32G431/CMSIS/Core/Template/ARMv8-M/main_s.c \
../Drivers/STM32G431/CMSIS/Core/Template/ARMv8-M/tz_context.c 

C_DEPS += \
./Drivers/STM32G431/CMSIS/Core/Template/ARMv8-M/main_s.d \
./Drivers/STM32G431/CMSIS/Core/Template/ARMv8-M/tz_context.d 

OBJS += \
./Drivers/STM32G431/CMSIS/Core/Template/ARMv8-M/main_s.o \
./Drivers/STM32G431/CMSIS/Core/Template/ARMv8-M/tz_context.o 


# Each subdirectory must supply rules for building sources it contributes
Drivers/STM32G431/CMSIS/Core/Template/ARMv8-M/%.o Drivers/STM32G431/CMSIS/Core/Template/ARMv8-M/%.su Drivers/STM32G431/CMSIS/Core/Template/ARMv8-M/%.cyclo: ../Drivers/STM32G431/CMSIS/Core/Template/ARMv8-M/%.c Drivers/STM32G431/CMSIS/Core/Template/ARMv8-M/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32G491xx -DUSE_FULL_LL_DRIVER -c -I../Core/Inc -I../Core/Inc/can_houbolt -I../Drivers/STM32G491/STRHAL/Inc -I../Drivers/STM32G491/CMSIS/Include -I../Drivers/STM32G491/LL_Driver/Inc -I../Drivers/STM32G491/CMSIS/Device/ST/STM32G4xx/Include -I"/home/georg/Documents/ST/Lamarr/firmware_liquids/Drivers/STM32G474" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-STM32G431-2f-CMSIS-2f-Core-2f-Template-2f-ARMv8-2d-M

clean-Drivers-2f-STM32G431-2f-CMSIS-2f-Core-2f-Template-2f-ARMv8-2d-M:
	-$(RM) ./Drivers/STM32G431/CMSIS/Core/Template/ARMv8-M/main_s.cyclo ./Drivers/STM32G431/CMSIS/Core/Template/ARMv8-M/main_s.d ./Drivers/STM32G431/CMSIS/Core/Template/ARMv8-M/main_s.o ./Drivers/STM32G431/CMSIS/Core/Template/ARMv8-M/main_s.su ./Drivers/STM32G431/CMSIS/Core/Template/ARMv8-M/tz_context.cyclo ./Drivers/STM32G431/CMSIS/Core/Template/ARMv8-M/tz_context.d ./Drivers/STM32G431/CMSIS/Core/Template/ARMv8-M/tz_context.o ./Drivers/STM32G431/CMSIS/Core/Template/ARMv8-M/tz_context.su

.PHONY: clean-Drivers-2f-STM32G431-2f-CMSIS-2f-Core-2f-Template-2f-ARMv8-2d-M

