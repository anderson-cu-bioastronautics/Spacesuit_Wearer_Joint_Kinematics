################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Drivers/STM32F4xx_HAL_Driver/Src/subdir.mk
-include Core/Startup/subdir.mk
-include Core/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
lsm6dsl_output.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
lsm6dsl_output.list \

OBJCOPY_BIN += \
lsm6dsl_output.bin \


# All Target
all:
	+@make --no-print-directory main-build && make --no-print-directory post-build

# Main-build Target
main-build: lsm6dsl_output.elf secondary-outputs

# Tool invocations
lsm6dsl_output.elf: $(OBJS) $(USER_OBJS) C:\Users\Young-Young\Documents\Projects\Thesis\IMS\IMUs\Software\firmware\imuarray\lsm6dsl_output\STM32F401RETX_FLASH.ld
	arm-none-eabi-gcc -o "lsm6dsl_output.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"C:\Users\Young-Young\Documents\Projects\Thesis\IMS\IMUs\Software\firmware\imuarray\lsm6dsl_output\STM32F401RETX_FLASH.ld" --specs=nosys.specs -Wl,-Map="lsm6dsl_output.map" -Wl,--gc-sections -static -L../Libraries/lib --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

lsm6dsl_output.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "lsm6dsl_output.list"
	@echo 'Finished building: $@'
	@echo ' '

lsm6dsl_output.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "lsm6dsl_output.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

post-build:
	arm-none-eabi-objcopy -O ihex lsm6dsl_output.elf lsm6dsl_output.hex
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY: post-build

-include ../makefile.targets
