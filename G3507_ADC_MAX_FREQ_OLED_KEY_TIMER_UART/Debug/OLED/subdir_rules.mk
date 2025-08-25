################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
OLED/%.o: ../OLED/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"D:/ti/ccs2020/ccs/tools/compiler/ti-cgt-armllvm_4.0.3.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"C:/Users/Marti/workspace_ccstheia/G3507_ADC_MAX_FREQ_OLED_KEY_TIMER_UART/OLED" -I"C:/Users/Marti/workspace_ccstheia/G3507_ADC_MAX_FREQ_OLED_KEY_TIMER_UART" -I"C:/Users/Marti/workspace_ccstheia/G3507_ADC_MAX_FREQ_OLED_KEY_TIMER_UART/Debug" -I"D:/ti/mspm0_sdk_2_04_00_06/source/third_party/CMSIS/Core/Include" -I"D:/ti/mspm0_sdk_2_04_00_06/source" -I"D:/ti/mspm0_sdk_2_04_00_06/source/third_party/CMSIS/Core/Include" -I"D:/ti/mspm0_sdk_2_04_00_06/source/third_party/CMSIS/DSP/Include" -gdwarf-3 -MMD -MP -MF"OLED/$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


