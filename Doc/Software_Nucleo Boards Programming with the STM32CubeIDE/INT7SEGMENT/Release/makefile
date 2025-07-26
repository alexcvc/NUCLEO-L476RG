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
INT7SEGMENT.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
INT7SEGMENT.list \

OBJCOPY_BIN += \
INT7SEGMENT.bin \


# All Target
all: INT7SEGMENT.elf secondary-outputs

# Tool invocations
INT7SEGMENT.elf: $(OBJS) $(USER_OBJS) C:\Users\Dogan\ Ibrahim\STM32CubeIDE\workspace_1.3.12\INT7SEGMENT\STM32L476RGTX_FLASH.ld
	arm-none-eabi-gcc -o "INT7SEGMENT.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\Dogan Ibrahim\STM32CubeIDE\workspace_1.3.12\INT7SEGMENT\STM32L476RGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="INT7SEGMENT.map" -Wl,--gc-sections -static --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

INT7SEGMENT.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "INT7SEGMENT.list"
	@echo 'Finished building: $@'
	@echo ' '

INT7SEGMENT.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "INT7SEGMENT.bin"
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
