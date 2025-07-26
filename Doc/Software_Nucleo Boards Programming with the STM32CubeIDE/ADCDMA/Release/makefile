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
ADCDMA.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
ADCDMA.list \

OBJCOPY_BIN += \
ADCDMA.bin \


# All Target
all: ADCDMA.elf secondary-outputs

# Tool invocations
ADCDMA.elf: $(OBJS) $(USER_OBJS) C:\Users\Dogan\ Ibrahim\STM32CubeIDE\workspace_1.3.26\ADCDMA\STM32L476RGTX_FLASH.ld
	arm-none-eabi-gcc -o "ADCDMA.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\Dogan Ibrahim\STM32CubeIDE\workspace_1.3.26\ADCDMA\STM32L476RGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="ADCDMA.map" -Wl,--gc-sections -static -u_printf_float --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

ADCDMA.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "ADCDMA.list"
	@echo 'Finished building: $@'
	@echo ' '

ADCDMA.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "ADCDMA.bin"
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
