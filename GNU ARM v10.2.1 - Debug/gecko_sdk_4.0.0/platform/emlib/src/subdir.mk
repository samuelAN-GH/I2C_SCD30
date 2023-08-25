################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk/platform/emlib/src/em_assert.c \
C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk/platform/emlib/src/em_cmu.c \
C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk/platform/emlib/src/em_core.c \
C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk/platform/emlib/src/em_emu.c \
C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk/platform/emlib/src/em_gpio.c \
C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk/platform/emlib/src/em_i2c.c \
C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk/platform/emlib/src/em_system.c \
C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk/platform/emlib/src/em_timer.c 

OBJS += \
./gecko_sdk_4.0.0/platform/emlib/src/em_assert.o \
./gecko_sdk_4.0.0/platform/emlib/src/em_cmu.o \
./gecko_sdk_4.0.0/platform/emlib/src/em_core.o \
./gecko_sdk_4.0.0/platform/emlib/src/em_emu.o \
./gecko_sdk_4.0.0/platform/emlib/src/em_gpio.o \
./gecko_sdk_4.0.0/platform/emlib/src/em_i2c.o \
./gecko_sdk_4.0.0/platform/emlib/src/em_system.o \
./gecko_sdk_4.0.0/platform/emlib/src/em_timer.o 

C_DEPS += \
./gecko_sdk_4.0.0/platform/emlib/src/em_assert.d \
./gecko_sdk_4.0.0/platform/emlib/src/em_cmu.d \
./gecko_sdk_4.0.0/platform/emlib/src/em_core.d \
./gecko_sdk_4.0.0/platform/emlib/src/em_emu.d \
./gecko_sdk_4.0.0/platform/emlib/src/em_gpio.d \
./gecko_sdk_4.0.0/platform/emlib/src/em_i2c.d \
./gecko_sdk_4.0.0/platform/emlib/src/em_system.d \
./gecko_sdk_4.0.0/platform/emlib/src/em_timer.d 


# Each subdirectory must supply rules for building sources it contributes
gecko_sdk_4.0.0/platform/emlib/src/em_assert.o: C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk/platform/emlib/src/em_assert.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFR32MG21A020F1024IM32=1' '-DSL_BOARD_NAME="BRD4180B"' '-DSL_BOARD_REV="A03"' '-DSL_COMPONENT_CATALOG_PRESENT=1' -I"C:\Users\samue\SimplicityStudio\v5_workspace\I2C_SCD30\config" -I"C:\Users\samue\SimplicityStudio\v5_workspace\I2C_SCD30\autogen" -I"C:\Users\samue\SimplicityStudio\v5_workspace\I2C_SCD30" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/Device/SiliconLabs/EFR32MG21/Include" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//hardware/board/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/CMSIS/Include" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/service/device_init/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/common/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/emlib/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/emlib/host/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/common/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/common/toolchain/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/service/system/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/ustimer/inc" -Os -Wall -Wextra -fno-builtin -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv5-sp-d16 -mfloat-abi=hard -c -fmessage-length=0 -MMD -MP -MF"gecko_sdk_4.0.0/platform/emlib/src/em_assert.d" -MT"gecko_sdk_4.0.0/platform/emlib/src/em_assert.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

gecko_sdk_4.0.0/platform/emlib/src/em_cmu.o: C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk/platform/emlib/src/em_cmu.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFR32MG21A020F1024IM32=1' '-DSL_BOARD_NAME="BRD4180B"' '-DSL_BOARD_REV="A03"' '-DSL_COMPONENT_CATALOG_PRESENT=1' -I"C:\Users\samue\SimplicityStudio\v5_workspace\I2C_SCD30\config" -I"C:\Users\samue\SimplicityStudio\v5_workspace\I2C_SCD30\autogen" -I"C:\Users\samue\SimplicityStudio\v5_workspace\I2C_SCD30" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/Device/SiliconLabs/EFR32MG21/Include" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//hardware/board/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/CMSIS/Include" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/service/device_init/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/common/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/emlib/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/emlib/host/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/common/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/common/toolchain/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/service/system/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/ustimer/inc" -Os -Wall -Wextra -fno-builtin -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv5-sp-d16 -mfloat-abi=hard -c -fmessage-length=0 -MMD -MP -MF"gecko_sdk_4.0.0/platform/emlib/src/em_cmu.d" -MT"gecko_sdk_4.0.0/platform/emlib/src/em_cmu.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

gecko_sdk_4.0.0/platform/emlib/src/em_core.o: C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk/platform/emlib/src/em_core.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFR32MG21A020F1024IM32=1' '-DSL_BOARD_NAME="BRD4180B"' '-DSL_BOARD_REV="A03"' '-DSL_COMPONENT_CATALOG_PRESENT=1' -I"C:\Users\samue\SimplicityStudio\v5_workspace\I2C_SCD30\config" -I"C:\Users\samue\SimplicityStudio\v5_workspace\I2C_SCD30\autogen" -I"C:\Users\samue\SimplicityStudio\v5_workspace\I2C_SCD30" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/Device/SiliconLabs/EFR32MG21/Include" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//hardware/board/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/CMSIS/Include" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/service/device_init/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/common/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/emlib/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/emlib/host/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/common/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/common/toolchain/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/service/system/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/ustimer/inc" -Os -Wall -Wextra -fno-builtin -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv5-sp-d16 -mfloat-abi=hard -c -fmessage-length=0 -MMD -MP -MF"gecko_sdk_4.0.0/platform/emlib/src/em_core.d" -MT"gecko_sdk_4.0.0/platform/emlib/src/em_core.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

gecko_sdk_4.0.0/platform/emlib/src/em_emu.o: C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk/platform/emlib/src/em_emu.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFR32MG21A020F1024IM32=1' '-DSL_BOARD_NAME="BRD4180B"' '-DSL_BOARD_REV="A03"' '-DSL_COMPONENT_CATALOG_PRESENT=1' -I"C:\Users\samue\SimplicityStudio\v5_workspace\I2C_SCD30\config" -I"C:\Users\samue\SimplicityStudio\v5_workspace\I2C_SCD30\autogen" -I"C:\Users\samue\SimplicityStudio\v5_workspace\I2C_SCD30" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/Device/SiliconLabs/EFR32MG21/Include" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//hardware/board/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/CMSIS/Include" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/service/device_init/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/common/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/emlib/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/emlib/host/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/common/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/common/toolchain/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/service/system/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/ustimer/inc" -Os -Wall -Wextra -fno-builtin -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv5-sp-d16 -mfloat-abi=hard -c -fmessage-length=0 -MMD -MP -MF"gecko_sdk_4.0.0/platform/emlib/src/em_emu.d" -MT"gecko_sdk_4.0.0/platform/emlib/src/em_emu.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

gecko_sdk_4.0.0/platform/emlib/src/em_gpio.o: C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk/platform/emlib/src/em_gpio.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFR32MG21A020F1024IM32=1' '-DSL_BOARD_NAME="BRD4180B"' '-DSL_BOARD_REV="A03"' '-DSL_COMPONENT_CATALOG_PRESENT=1' -I"C:\Users\samue\SimplicityStudio\v5_workspace\I2C_SCD30\config" -I"C:\Users\samue\SimplicityStudio\v5_workspace\I2C_SCD30\autogen" -I"C:\Users\samue\SimplicityStudio\v5_workspace\I2C_SCD30" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/Device/SiliconLabs/EFR32MG21/Include" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//hardware/board/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/CMSIS/Include" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/service/device_init/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/common/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/emlib/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/emlib/host/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/common/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/common/toolchain/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/service/system/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/ustimer/inc" -Os -Wall -Wextra -fno-builtin -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv5-sp-d16 -mfloat-abi=hard -c -fmessage-length=0 -MMD -MP -MF"gecko_sdk_4.0.0/platform/emlib/src/em_gpio.d" -MT"gecko_sdk_4.0.0/platform/emlib/src/em_gpio.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

gecko_sdk_4.0.0/platform/emlib/src/em_i2c.o: C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk/platform/emlib/src/em_i2c.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFR32MG21A020F1024IM32=1' '-DSL_BOARD_NAME="BRD4180B"' '-DSL_BOARD_REV="A03"' '-DSL_COMPONENT_CATALOG_PRESENT=1' -I"C:\Users\samue\SimplicityStudio\v5_workspace\I2C_SCD30\config" -I"C:\Users\samue\SimplicityStudio\v5_workspace\I2C_SCD30\autogen" -I"C:\Users\samue\SimplicityStudio\v5_workspace\I2C_SCD30" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/Device/SiliconLabs/EFR32MG21/Include" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//hardware/board/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/CMSIS/Include" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/service/device_init/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/common/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/emlib/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/emlib/host/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/common/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/common/toolchain/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/service/system/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/ustimer/inc" -Os -Wall -Wextra -fno-builtin -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv5-sp-d16 -mfloat-abi=hard -c -fmessage-length=0 -MMD -MP -MF"gecko_sdk_4.0.0/platform/emlib/src/em_i2c.d" -MT"gecko_sdk_4.0.0/platform/emlib/src/em_i2c.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

gecko_sdk_4.0.0/platform/emlib/src/em_system.o: C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk/platform/emlib/src/em_system.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFR32MG21A020F1024IM32=1' '-DSL_BOARD_NAME="BRD4180B"' '-DSL_BOARD_REV="A03"' '-DSL_COMPONENT_CATALOG_PRESENT=1' -I"C:\Users\samue\SimplicityStudio\v5_workspace\I2C_SCD30\config" -I"C:\Users\samue\SimplicityStudio\v5_workspace\I2C_SCD30\autogen" -I"C:\Users\samue\SimplicityStudio\v5_workspace\I2C_SCD30" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/Device/SiliconLabs/EFR32MG21/Include" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//hardware/board/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/CMSIS/Include" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/service/device_init/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/common/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/emlib/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/emlib/host/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/common/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/common/toolchain/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/service/system/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/ustimer/inc" -Os -Wall -Wextra -fno-builtin -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv5-sp-d16 -mfloat-abi=hard -c -fmessage-length=0 -MMD -MP -MF"gecko_sdk_4.0.0/platform/emlib/src/em_system.d" -MT"gecko_sdk_4.0.0/platform/emlib/src/em_system.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

gecko_sdk_4.0.0/platform/emlib/src/em_timer.o: C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk/platform/emlib/src/em_timer.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-DDEBUG_EFM=1' '-DEFR32MG21A020F1024IM32=1' '-DSL_BOARD_NAME="BRD4180B"' '-DSL_BOARD_REV="A03"' '-DSL_COMPONENT_CATALOG_PRESENT=1' -I"C:\Users\samue\SimplicityStudio\v5_workspace\I2C_SCD30\config" -I"C:\Users\samue\SimplicityStudio\v5_workspace\I2C_SCD30\autogen" -I"C:\Users\samue\SimplicityStudio\v5_workspace\I2C_SCD30" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/Device/SiliconLabs/EFR32MG21/Include" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//hardware/board/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/CMSIS/Include" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/service/device_init/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/common/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/emlib/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/emlib/host/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/common/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/common/toolchain/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/service/system/inc" -I"C:/Users/samue/SimplicityStudio/SDKs/gecko_sdk//platform/emdrv/ustimer/inc" -Os -Wall -Wextra -fno-builtin -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv5-sp-d16 -mfloat-abi=hard -c -fmessage-length=0 -MMD -MP -MF"gecko_sdk_4.0.0/platform/emlib/src/em_timer.d" -MT"gecko_sdk_4.0.0/platform/emlib/src/em_timer.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


