################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include gecko_sdk_4.0.0/platform/service/system/src/subdir.mk
-include gecko_sdk_4.0.0/platform/service/device_init/src/subdir.mk
-include gecko_sdk_4.0.0/platform/emlib/src/subdir.mk
-include gecko_sdk_4.0.0/platform/emdrv/ustimer/src/subdir.mk
-include gecko_sdk_4.0.0/platform/common/toolchain/src/subdir.mk
-include gecko_sdk_4.0.0/platform/Device/SiliconLabs/EFR32MG21/Source/GCC/subdir.mk
-include gecko_sdk_4.0.0/platform/Device/SiliconLabs/EFR32MG21/Source/subdir.mk
-include gecko_sdk_4.0.0/hardware/board/src/subdir.mk
-include autogen/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: I2C_SCD30.axf

# Tool invocations
I2C_SCD30.axf: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GNU ARM C Linker'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m33 -mthumb -T "C:\Users\samue\SimplicityStudio\v5_workspace\I2C_SCD30/autogen/linkerfile.ld" -Xlinker --gc-sections -Xlinker -Map="I2C_SCD30.map" -mfpu=fpv5-sp-d16 -mfloat-abi=hard --specs=nano.specs -o I2C_SCD30.axf -Wl,--start-group "./app.o" "./main.o" "./autogen/sl_board_default_init.o" "./autogen/sl_device_init_clocks.o" "./autogen/sl_event_handler.o" "./gecko_sdk_4.0.0/hardware/board/src/sl_board_control_gpio.o" "./gecko_sdk_4.0.0/hardware/board/src/sl_board_init.o" "./gecko_sdk_4.0.0/platform/Device/SiliconLabs/EFR32MG21/Source/GCC/startup_efr32mg21.o" "./gecko_sdk_4.0.0/platform/Device/SiliconLabs/EFR32MG21/Source/system_efr32mg21.o" "./gecko_sdk_4.0.0/platform/common/toolchain/src/sl_memory.o" "./gecko_sdk_4.0.0/platform/emdrv/ustimer/src/ustimer.o" "./gecko_sdk_4.0.0/platform/emlib/src/em_assert.o" "./gecko_sdk_4.0.0/platform/emlib/src/em_cmu.o" "./gecko_sdk_4.0.0/platform/emlib/src/em_core.o" "./gecko_sdk_4.0.0/platform/emlib/src/em_emu.o" "./gecko_sdk_4.0.0/platform/emlib/src/em_gpio.o" "./gecko_sdk_4.0.0/platform/emlib/src/em_i2c.o" "./gecko_sdk_4.0.0/platform/emlib/src/em_system.o" "./gecko_sdk_4.0.0/platform/emlib/src/em_timer.o" "./gecko_sdk_4.0.0/platform/service/device_init/src/sl_device_init_emu_s2.o" "./gecko_sdk_4.0.0/platform/service/device_init/src/sl_device_init_hfrco.o" "./gecko_sdk_4.0.0/platform/service/device_init/src/sl_device_init_hfxo_s2.o" "./gecko_sdk_4.0.0/platform/service/device_init/src/sl_device_init_lfxo_s2.o" "./gecko_sdk_4.0.0/platform/service/device_init/src/sl_device_init_nvic.o" "./gecko_sdk_4.0.0/platform/service/system/src/sl_system_init.o" "./gecko_sdk_4.0.0/platform/service/system/src/sl_system_process_action.o" -lgcc -lc -lm -lnosys -Wl,--end-group -Wl,--start-group -lgcc -lc -lnosys -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

	@echo 'Building hex file: I2C_SCD30.hex'
	arm-none-eabi-objcopy -O ihex "I2C_SCD30.axf" "I2C_SCD30.hex"
	@echo ' '

	@echo 'Building bin file: I2C_SCD30.bin'
	arm-none-eabi-objcopy -O binary "I2C_SCD30.axf" "I2C_SCD30.bin"
	@echo ' '

	@echo 'Building s37 file: I2C_SCD30.s37'
	arm-none-eabi-objcopy -O srec "I2C_SCD30.axf" "I2C_SCD30.s37"
	@echo ' '

	@echo 'Running size tool'
	arm-none-eabi-size "I2C_SCD30.axf" -A
	@echo ' '

# Other Targets
clean:
	-@$(foreach FILE, $(EXECUTABLES), $(shell $(RM) $(FILE)) )
	-@$(foreach FILE, $(OBJS), $(shell $(RM) $(FILE)) )
	-@$(foreach FILE, $(C_DEPS), $(shell $(RM) $(FILE)) )
	-$(RM) I2C_SCD30.axf
	-@echo ' '

.PHONY: all clean dependents

-include ../makefile.targets
