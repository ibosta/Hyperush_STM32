################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
S_SRCS += \
../756zg-IR9/Core/Startup/startup_stm32f756zgtx.s 

OBJS += \
./756zg-IR9/Core/Startup/startup_stm32f756zgtx.o 

S_DEPS += \
./756zg-IR9/Core/Startup/startup_stm32f756zgtx.d 


# Each subdirectory must supply rules for building sources it contributes
756zg-IR9/Core/Startup/%.o: ../756zg-IR9/Core/Startup/%.s 756zg-IR9/Core/Startup/subdir.mk
	arm-none-eabi-gcc -mcpu=cortex-m7 -g3 -DDEBUG -c -x assembler-with-cpp -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@" "$<"

clean: clean-756zg-2d-IR9-2f-Core-2f-Startup

clean-756zg-2d-IR9-2f-Core-2f-Startup:
	-$(RM) ./756zg-IR9/Core/Startup/startup_stm32f756zgtx.d ./756zg-IR9/Core/Startup/startup_stm32f756zgtx.o

.PHONY: clean-756zg-2d-IR9-2f-Core-2f-Startup

