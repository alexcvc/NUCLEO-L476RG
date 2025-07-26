################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
PWM3.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
PWM3.list \

OBJCOPY_BIN += \
PWM3.bin \


# All Target
all: PWM3.elf secondary-outputs

# Tool invocations
PWM3.elf: $(OBJS) $(USER_OBJS) C:\Users\Dogan\ Ibrahim\STM32CubeIDE\workspace_1.3.31\PWM3\STM32L476RGTX_FLASH.ld
	arm-none-eabi-gcc -o "PWM3.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\Dogan Ibrahim\STM32CubeIDE\workspace_1.3.31\PWM3\STM32L476RGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="PWM3.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

PWM3.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "PWM3.list"
	@echo 'Finished building: $@'
	@echo ' '

PWM3.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "PWM3.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
