################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"D:/ti/ccs2020/ccs/tools/compiler/ti-cgt-armllvm_4.0.3.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"C:/Users/Marti/workspace_ccstheia/G3507_ADC_MAX_FREQ_OLED_KEY_TIMER_UART/OLED" -I"C:/Users/Marti/workspace_ccstheia/G3507_ADC_MAX_FREQ_OLED_KEY_TIMER_UART" -I"C:/Users/Marti/workspace_ccstheia/G3507_ADC_MAX_FREQ_OLED_KEY_TIMER_UART/Debug" -I"D:/ti/mspm0_sdk_2_04_00_06/source/third_party/CMSIS/Core/Include" -I"D:/ti/mspm0_sdk_2_04_00_06/source" -I"D:/ti/mspm0_sdk_2_04_00_06/source/third_party/CMSIS/Core/Include" -I"D:/ti/mspm0_sdk_2_04_00_06/source/third_party/CMSIS/DSP/Include" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-175068703: ../G3507_Signal_Analysis_Device.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"D:/ti/ccs2020/ccs/utils/sysconfig_1.24.0/sysconfig_cli.bat" --script "C:/Users/Marti/workspace_ccstheia/G3507_ADC_MAX_FREQ_OLED_KEY_TIMER_UART/G3507_Signal_Analysis_Device.syscfg" -o "." -s "D:/ti/mspm0_sdk_2_04_00_06/.metadata/product.json" --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

device_linker.cmd: build-175068703 ../G3507_Signal_Analysis_Device.syscfg
device.opt: build-175068703
device.cmd.genlibs: build-175068703
ti_msp_dl_config.c: build-175068703
ti_msp_dl_config.h: build-175068703
Event.dot: build-175068703

%.o: ./%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"D:/ti/ccs2020/ccs/tools/compiler/ti-cgt-armllvm_4.0.3.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"C:/Users/Marti/workspace_ccstheia/G3507_ADC_MAX_FREQ_OLED_KEY_TIMER_UART/OLED" -I"C:/Users/Marti/workspace_ccstheia/G3507_ADC_MAX_FREQ_OLED_KEY_TIMER_UART" -I"C:/Users/Marti/workspace_ccstheia/G3507_ADC_MAX_FREQ_OLED_KEY_TIMER_UART/Debug" -I"D:/ti/mspm0_sdk_2_04_00_06/source/third_party/CMSIS/Core/Include" -I"D:/ti/mspm0_sdk_2_04_00_06/source" -I"D:/ti/mspm0_sdk_2_04_00_06/source/third_party/CMSIS/Core/Include" -I"D:/ti/mspm0_sdk_2_04_00_06/source/third_party/CMSIS/DSP/Include" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

startup_mspm0g350x_ticlang.o: D:/ti/mspm0_sdk_2_04_00_06/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"D:/ti/ccs2020/ccs/tools/compiler/ti-cgt-armllvm_4.0.3.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"C:/Users/Marti/workspace_ccstheia/G3507_ADC_MAX_FREQ_OLED_KEY_TIMER_UART/OLED" -I"C:/Users/Marti/workspace_ccstheia/G3507_ADC_MAX_FREQ_OLED_KEY_TIMER_UART" -I"C:/Users/Marti/workspace_ccstheia/G3507_ADC_MAX_FREQ_OLED_KEY_TIMER_UART/Debug" -I"D:/ti/mspm0_sdk_2_04_00_06/source/third_party/CMSIS/Core/Include" -I"D:/ti/mspm0_sdk_2_04_00_06/source" -I"D:/ti/mspm0_sdk_2_04_00_06/source/third_party/CMSIS/Core/Include" -I"D:/ti/mspm0_sdk_2_04_00_06/source/third_party/CMSIS/DSP/Include" -gdwarf-3 -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


