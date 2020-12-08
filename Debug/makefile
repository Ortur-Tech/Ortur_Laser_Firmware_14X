################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include stm32/USB_CFG/subdir.mk
-include stm32/STM32_USB_Device_Library/Core/Src/subdir.mk
-include stm32/STM32_USB_Device_Library/Class/CDC/Src/subdir.mk
-include stm32/STM32F1xx_HAL_Driver/Src/subdir.mk
-include stm32/BMA2x2_driver/subdir.mk
-include stm32/subdir.mk
-include grbl/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
LaserMasterGrbl.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
LaserMasterGrbl.list \

OBJCOPY_BIN += \
LaserMasterGrbl.bin \


# All Target
all: LaserMasterGrbl.elf secondary-outputs

# Tool invocations
LaserMasterGrbl.elf: $(OBJS) $(USER_OBJS) E:\MyGit\GrblWorkSpace\LaserMasterGrbl\STM32F103C8TX_FLASH.ld
	arm-none-eabi-g++ -o "LaserMasterGrbl.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m3 -T"E:\MyGit\GrblWorkSpace\LaserMasterGrbl\STM32F103C8TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="LaserMasterGrbl.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -lstdc++ -lsupc++ -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

LaserMasterGrbl.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "LaserMasterGrbl.list"
	@echo 'Finished building: $@'
	@echo ' '

LaserMasterGrbl.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "LaserMasterGrbl.bin"
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
