# Signal Analysis Device Based on MSPM0G3507

## Project Overview

This project implements a comprehensive signal analysis system based on the MSPM0G3507 microcontroller. The system provides AC signal conditioning, measurement, and wireless transmission capabilities. It features signal conditioning circuits using on-chip programmable operational amplifiers, frequency measurement across multiple ranges, waveform analysis, and wireless data transmission.

### Key Features

- **Signal Conditioning**: 60mVpp~1000mVpp AC signal conditioning to 0~3.3V range
- **Multi-range Frequency Measurement**: 1Hz~100kHz with optimized algorithms
- **Waveform Analysis**: Peak-to-peak measurement and waveform classification
- **Wireless Communication**: Bluetooth data transmission via HC-05 module
- **Audio Signal Recognition**: Threshold-based audio signal identification
- **Real-time Display**: OLED screen for data visualization

## System Block Diagram

```
┌─────────────┐    ┌──────────────┐    ┌─────────────┐    ┌──────────────┐
│   Signal    │    │    Signal    │    │    ADC &    │    │    Signal    │
│   Input     │───▶│ Conditioning │───▶│  Sampling   │───▶│  Processing  │
│ 60mV-1000mV │    │   Circuit    │    │   System    │    │   & Analysis │
└─────────────┘    └──────────────┘    └─────────────┘    └──────────────┘
                           │                    │                   │
                           ▼                    ▼                   ▼
┌─────────────┐    ┌──────────────┐    ┌─────────────┐    ┌──────────────┐
│  Bluetooth  │    │   Reference  │    │   Timer &   │    │    OLED      │
│   Module    │◀───│   Voltage    │    │  Capture    │    │   Display    │
│   (HC-05)   │    │  Generation  │    │   System    │    │   System     │
└─────────────┘    └──────────────┘    └─────────────┘    └──────────────┘
```

## Signal Conditioning Circuit

```
Input Signal     ┌───────────────┐     ┌─────────────┐     ADC Input
(60mV-1000mV) ───┤ Inverting Amp │────▶│ DC Bias     │────▶ (0-3.3V)
   AC            │ Gain: -3x     │     │ +1.65V      │     0-3.3V Range
                 └───────────────┘     └─────────────┘
                         │                     │
                         ▼                     ▼
                 ┌───────────────┐     ┌─────────────┐
                 │   Internal    │     │ Internal    │
                 │     OPA       │     │     DAC     │
                 │  (PA22/PA16)  │     │  Reference  │
                 └───────────────┘     └─────────────┘
```

## Hardware Configuration

### Operational Amplifiers (OPA)

The system utilizes two on-chip operational amplifiers configured through the SysConfig GUI:

**OPA_0 & OPA_1 Configuration:**
- **Mode**: Inverting amplifier configuration
- **Gain Setting**: -3x (providing signal amplification and inversion)
- **Positive Input**: Connected to internal DAC output (1.65V DC bias)
- **Negative Input**: External signal input via resistor tap network
- **M-MUX Selection**: External negative input (IN1_NEG)
- **Bandwidth**: High bandwidth mode enabled for better frequency response
- **Chopping Mode**: ADC averaging mode for noise reduction
- **Output Pins**: PA22 (OPA_0), PA16 (OPA_1)
- **Input Pins**: PA24 (OPA_0 negative), PA17 (OPA_1 negative)

### Comparators (COMP)

Three comparators are configured for signal processing and reference generation:

**COMP_0 & COMP_1 (Reference Generation):**
- **Reference Source**: Internal VREF DAC
- **Control Mode**: Software controlled DAC
- **DAC Code**: 0x49 (generates 1.65V reference voltage)
- **Function**: Provides stable DC bias for signal conditioning

**COMP_2 (Signal Processing):**
- **Channel Configuration**: Both positive and negative channels enabled
- **Hysteresis**: 20mV hysteresis for noise immunity
- **Output Filter**: 1200ns delay filter for signal stability
- **Output Enable**: Digital output enabled on PB27
- **Input Pins**: PB21 (positive), PB22 (negative)
- **Function**: Audio signal detection and threshold comparison

### ADC Configuration

Dual 12-bit ADCs with DMA support for high-speed sampling:

**ADC12_0 (Primary Signal Channel):**
- **Trigger Source**: Event-triggered conversion
- **Repeat Mode**: Continuous sampling enabled
- **Sample Time**: 62.5ns for maximum sampling rate
- **DMA Configuration**: Full channel repeat single mode
- **Transfer Size**: 1024 samples
- **Data Length**: 16-bit half-word transfers
- **Address Mode**: Fixed-to-buffer addressing
- **Input Pin**: PA27
- **Function**: Main signal waveform sampling

**ADC12_1 (Audio Channel):**
- **Trigger Source**: Event-triggered conversion
- **Repeat Mode**: Continuous sampling enabled
- **Sample Time**: 62.5ns for high-speed audio sampling
- **DMA Configuration**: Full channel repeat single mode
- **Transfer Size**: 4096 samples (larger buffer for audio analysis)
- **Data Length**: 16-bit half-word transfers
- **Input Pin**: PA15
- **Function**: Audio signal sampling and recognition

### Timer Configuration

Multiple timers configured for different frequency measurement strategies:

**TIMER_0 (High-Frequency Sampling):**
- **Mode**: Periodic timer mode
- **Period**: 4μs (250kHz effective sampling rate)
- **Function**: Triggers high-frequency ADC sampling
- **Event Publisher**: Channel 1 for ADC triggering

**TIMER_1 (Low-Frequency Counting):**
- **Clock Prescaler**: 256
- **Clock Divider**: 2
- **Period**: 1 second
- **Function**: Gate time for low-frequency edge counting
- **Frequency Range**: 1Hz - 1kHz

**TIMER_2 (Medium-Frequency Sampling):**
- **Period**: 1ms
- **Event Publisher**: Channel 2
- **Function**: Medium-frequency signal sampling trigger
- **Frequency Range**: 1kHz - 10kHz

**CAPTURE_0 (Frequency Capture):**
- **Mode**: Combined capture mode
- **Timer Period**: 8ms
- **Clock Prescaler**: 32
- **Capture Pin**: PA12
- **Interrupts**: CC1_DN and ZERO events enabled
- **Function**: Direct frequency measurement via period capture

**COMPARE_0 (Edge Counting):**
- **Mode**: Edge count up mode
- **Timer Period**: 65535 counts
- **Clock Divider**: 8
- **Clock Prescaler**: 200
- **Output Pin**: PB0
- **Function**: Counts signal edges for low-frequency measurement

### Communication Interfaces

**I2C Interface (OLED Display):**
- **Mode**: Controller mode enabled
- **Data Pin**: PA28 (SDA)
- **Clock Pin**: PA31 (SCL)
- **Interrupts**: Arbitration lost, NACK, RX/TX FIFO triggers, transfer done
- **Function**: Communicates with 128x64 OLED display

**UART Interface (Bluetooth Communication):**
- **FIFO**: Enabled for buffered communication
- **Transmit Pin**: PA10
- **Receive Pin**: PA11
- **Baud Rate**: Configured for HC-05 module compatibility
- **Function**: Wireless data transmission to external devices

### Voltage Reference (VREF)

**VREF Configuration:**
- **Reference Output**: PA23 (VREF+)
- **Ready Check**: Enabled for stable reference
- **Advanced Clock**: Bus clock source selected
- **Function**: Provides stable voltage reference for ADC and DAC operations

### GPIO Configuration

**User Interface:**
- **LED**: PA0 (USER_LED_1) - Status indication
- **Switch**: PA18 (USER_SWITCH_2) - Mode selection with pull-up resistor
- **Interrupt**: Rising/falling edge detection for user input

## Signal Processing Algorithms

### Frequency Measurement Strategy

The system employs a three-tier frequency measurement approach:

**1. Low Frequency Range (1Hz - 1kHz):**
- **Method**: Edge counting with 1-second gate time
- **Implementation**: COMPARE_0 timer counts signal transitions
- **Accuracy**: High precision for slow signals
- **Calculation**: Frequency = Edge_Count / Gate_Time

**2. Medium Frequency Range (1kHz - 10kHz):**
- **Method**: Timer capture with period measurement
- **Implementation**: CAPTURE_0 measures signal period directly
- **Accuracy**: Optimized for mid-range frequencies
- **Calculation**: Frequency = 1 / Measured_Period

**3. High Frequency Range (10kHz - 100kHz):**
- **Method**: FFT spectrum analysis
- **Implementation**: 1024-point radix-4 FFT using ARM DSP library
- **Sampling Rate**: 250kHz effective rate
- **Frequency Resolution**: 244Hz per bin
- **Calculation**: Peak detection in frequency domain

### Waveform Analysis

**Peak-to-Peak Measurement:**
- Analyzes ADC sample buffer to find maximum and minimum values
- Converts ADC counts to voltage: `Voltage = (ADC_Value / 4096) * 3300mV / 3`
- Accounts for signal conditioning gain factor

**RMS Calculation:**
- Removes DC component by calculating sample average
- Computes AC RMS value: `RMS = sqrt(Σ(sample - DC)² / N)`
- Provides true effective value measurement

**Waveform Classification:**
Based on RMS-to-peak-to-peak ratio analysis:
- **Sine Wave**: Ratio = 0.32 - 0.4 (theoretical: 0.354)
- **Square Wave**: Ratio = 0.4 - 0.6 (theoretical: 0.5)
- **Triangle Wave**: Ratio = 0.2 - 0.32 (theoretical: 0.289)
- **Unknown**: Ratio outside defined ranges

### Audio Signal Recognition

**Dual-Threshold Detection:**
- **Primary Threshold**: 3000 ADC counts for strong audio signals
- **Secondary Threshold**: 2400 ADC counts for weaker audio signals
- **Sample Buffer**: 4096 samples for comprehensive analysis
- **Pattern Matching**: Duration-based classification into 8 audio categories

**Recognition Algorithm:**
1. Scans entire 4096-sample buffer
2. Counts samples exceeding each threshold
3. Records maximum threshold counts during audio activity
4. Classifies audio based on threshold count patterns
5. Resets counters after audio period ends

## Pin Configuration Summary

| Function | Pin | Configuration | Description |
|----------|-----|---------------|-------------|
| ADC0 Input | PA27 | Analog Input | Primary signal channel |
| ADC1 Input | PA15 | Analog Input | Audio signal channel |
| OPA0 Output | PA22 | Analog Output | Conditioned signal output |
| OPA0 Input- | PA24 | Analog Input | External signal input |
| OPA1 Output | PA16 | Analog Output | Secondary conditioned output |
| OPA1 Input- | PA17 | Analog Input | Secondary signal input |
| COMP2 Output | PB27 | Digital Output | Comparator result |
| COMP2 Input+ | PB21 | Analog Input | Positive comparison |
| COMP2 Input- | PB22 | Analog Input | Negative comparison |
| CAPTURE Input | PA12 | Digital Input | Frequency capture |
| COMPARE Output | PB0 | Digital Output | Edge counting |
| I2C SDA | PA28 | Open-Drain | OLED data line |
| I2C SCL | PA31 | Open-Drain | OLED clock line |
| UART TX | PA10 | Digital Output | Bluetooth transmit |
| UART RX | PA11 | Digital Input | Bluetooth receive |
| VREF+ | PA23 | Analog Output | Voltage reference |
| User LED | PA0 | Digital Output | Status indicator |
| User Switch | PA18 | Digital Input | Mode selection |

## Performance Specifications

### Measurement Accuracy
- **Frequency Accuracy**: ±0.1% ± 0.5Hz (meeting competition requirements)
- **Peak-to-Peak Accuracy**: ±1% ± 1mV (meeting competition requirements)
- **Input Voltage Range**: 60mVpp - 1000mVpp (as specified)
- **Frequency Range**: 1Hz - 100kHz (extended from basic 1kHz-10kHz requirement)

### System Performance
- **ADC Resolution**: 12-bit (4096 levels)
- **Sampling Rate**: Up to 250kHz effective
- **FFT Points**: 1024 for high-frequency analysis
- **Audio Buffer**: 4096 samples for recognition
- **Display Update**: Real-time OLED refresh
- **Wireless Transmission**: Continuous data streaming

## Build and Usage Instructions

### Prerequisites
- Code Composer Studio (CCS) with MSPM0 support
- MSPM0 SDK version 2.04.00.06 or later
- ARM DSP library for FFT functions
- HC-05 Bluetooth module
- 128x64 I2C OLED display

### Hardware Setup
1. Connect signal source to PA24 (OPA0 negative input)
2. Connect OLED display to I2C pins (PA28/PA31)
3. Connect HC-05 module to UART pins (PA10/PA11)
4. Ensure proper power supply (3.3V) connections
5. Connect ground references between all modules

### Software Compilation
1. Import project into Code Composer Studio
2. Verify MSPM0 SDK installation and version
3. Build project using provided makefile
4. Resolve any dependency issues with ARM DSP library
5. Program MSPM0G3507 device via debugger

### Operation Instructions
1. Power on the system and verify OLED display initialization
2. Connect signal source to input terminals
3. System automatically detects frequency range and selects optimal measurement method
4. View real-time measurements on OLED display:
   - Line 1: Frequency value and unit
   - Line 2: Peak-to-peak voltage
   - Line 3: Waveform type (正弦波/方波/三角波)
   - Line 4: Audio recognition result
5. Monitor Bluetooth output for data logging and remote monitoring

### File Structure

```
G3507_ADC_MAX_FREQ_OLED_KEY_TIMER_UART/
├── G3507_Signal_Analysis_Device.c          # Main application code
├── G3507_Signal_Analysis_Device.syscfg     # System configuration file
├── ti_msp_dl_config.h                      # Generated configuration header
├── ti_msp_dl_config.c                      # Generated configuration source
├── OLED/                                   # OLED driver files
│   ├── oled.c                             # OLED driver implementation
│   ├── oled.h                             # OLED driver header
│   ├── oledfont.h                         # Font definitions
│   └── bmp.h                              # Bitmap definitions
├── Debug/                                  # Build output directory
│   ├── *.out                              # Executable files
│   ├── *.map                              # Memory map files
│   └── *.o                                # Object files
└── targetConfigs/                          # Target configuration
    ├── MSPM0G3507.ccxml                   # Target configuration
    └── readme.txt                         # Configuration notes
```

## Competition Requirements Compliance

This design fully implements the competition requirements as specified:

### Basic Requirements ✅
- **Input Signal Frequency Range**: 1kHz-10kHz ✅
- **Input Signal Peak-to-Peak Range**: 60mVpp-1000mVpp ✅
- **Frequency Measurement Function**: Precision 0.1%+0.5Hz ✅
- **Peak-to-Peak Measurement Function**: Precision 1%+1mV ✅

### Extended Features ✅
- **Extended Frequency Range**: 1Hz-100kHz (beyond basic 1kHz-10kHz) ✅
- **Audio Recognition Function**: Contact-triggered audio identification ✅
- **Additional Features**: OLED display, Bluetooth transmission, automatic range switching ✅

### Innovation Points
- Adaptive frequency measurement using three different algorithms
- Dual-threshold audio recognition system
- Real-time waveform classification
- Wireless data transmission capability
- Comprehensive signal conditioning using on-chip resources

---

# MSPM0G3507信号分析设备

## 项目概述

本项目基于MSPM0G3507微控制器实现了一个综合性信号分析系统。系统提供交流信号调理、测量和无线传输功能，特色在于使用片内可编程运算放大器、多频段频率测量、波形分析和无线数据传输。

### 主要特性

- **信号调理**: 将60mVpp~1000mVpp交流信号调理至0~3.3V范围
- **多频段频率测量**: 1Hz~100kHz范围内的优化算法测量
- **波形分析**: 峰峰值测量和波形分类识别
- **无线通信**: 通过HC-05蓝牙模块进行数据传输
- **音频信号识别**: 基于阈值的音频信号识别算法
- **实时显示**: OLED屏幕数据可视化

## 系统框图

```
┌─────────────┐    ┌──────────────┐    ┌─────────────┐    ┌──────────────┐
│   信号      │    │    信号      │    │   ADC和     │    │    信号      │
│   输入      │───▶│   调理电路   │───▶│  采样系统   │───▶│  处理与分析  │
│ 60mV-1000mV │    │              │    │             │    │              │
└─────────────┘    └──────────────┘    └─────────────┘    └──────────────┘
                           │                    │                   │
                           ▼                    ▼                   ▼
┌─────────────┐    ┌──────────────┐    ┌─────────────┐    ┌──────────────┐
│   蓝牙      │    │   基准电压   │    │   定时器和  │    │    OLED      │
│   模块      │◀───│   产生电路   │    │  捕获系统   │    │   显示系统   │
│  (HC-05)    │    │              │    │             │    │              │
└─────────────┘    └──────────────┘    └─────────────┘    └──────────────┘
```

## 信号调理电路

```
输入信号          ┌───────────────┐     ┌─────────────┐     ADC输入
(60mV-1000mV) ───┤   反相放大器  │────▶│   直流偏置  │────▶ (0-3.3V)
    交流          │   增益:-3x    │     │   +1.65V    │     0-3.3V范围
                 └───────────────┘     └─────────────┘
                         │                     │
                         ▼                     ▼
                 ┌───────────────┐     ┌─────────────┐
                 │     片内      │     │    片内     │
                 │    运放OPA    │     │   DAC基准   │
                 │  (PA22/PA16)  │     │    电路     │
                 └───────────────┘     └─────────────┘
```

## 硬件配置详解

### 运算放大器(OPA)配置

系统使用两个片内运算放大器，通过SysConfig图形界面配置：

**OPA_0和OPA_1配置：**
- **工作模式**: 反相放大器配置
- **增益设置**: -3x（提供信号放大和反相）
- **正输入端**: 连接到内部DAC输出（1.65V直流偏置）
- **负输入端**: 通过电阻分压网络连接外部信号输入
- **M-MUX选择**: 外部负输入（IN1_NEG）
- **带宽设置**: 高带宽模式，获得更好的频率响应
- **斩波模式**: ADC平均模式，用于降噪
- **输出引脚**: PA22（OPA_0）、PA16（OPA_1）
- **输入引脚**: PA24（OPA_0负端）、PA17（OPA_1负端）

### 比较器(COMP)配置

三个比较器用于信号处理和基准电压产生：

**COMP_0和COMP_1（基准电压生成）：**
- **基准源**: 内部VREF DAC
- **控制模式**: 软件控制DAC
- **DAC码值**: 0x49（生成1.65V基准电压）
- **功能**: 为信号调理提供稳定的直流偏置

**COMP_2（信号处理）：**
- **通道配置**: 正负两个通道均启用
- **迟滞设置**: 20mV迟滞，提高抗噪声能力
- **输出滤波**: 1200ns延迟滤波器，保证信号稳定性
- **输出使能**: 在PB27引脚输出数字信号
- **输入引脚**: PB21（正端）、PB22（负端）
- **功能**: 音频信号检测和阈值比较

### ADC配置

双12位ADC配置，支持DMA高速采样：

**ADC12_0（主信号通道）：**
- **触发源**: 事件触发转换
- **重复模式**: 连续采样模式启用
- **采样时间**: 62.5ns，达到最大采样速率
- **DMA配置**: 全通道重复单次模式
- **传输大小**: 1024个采样点
- **数据长度**: 16位半字传输
- **地址模式**: 固定到缓冲区寻址
- **输入引脚**: PA27
- **功能**: 主信号波形采样

**ADC12_1（音频通道）：**
- **触发源**: 事件触发转换
- **重复模式**: 连续采样模式启用
- **采样时间**: 62.5ns，高速音频采样
- **DMA配置**: 全通道重复单次模式
- **传输大小**: 4096个采样点（音频分析用的更大缓冲区）
- **数据长度**: 16位半字传输
- **输入引脚**: PA15
- **功能**: 音频信号采样和识别

### 定时器配置

多个定时器配置用于不同的频率测量策略：

**TIMER_0（高频采样）：**
- **模式**: 周期定时器模式
- **周期**: 4μs（250kHz有效采样率）
- **功能**: 触发高频ADC采样
- **事件发布**: 通道1用于ADC触发

**TIMER_1（低频计数）：**
- **时钟预分频**: 256
- **时钟分频**: 2
- **周期**: 1秒
- **功能**: 低频边沿计数的门控时间
- **频率范围**: 1Hz - 1kHz

**TIMER_2（中频采样）：**
- **周期**: 1ms
- **事件发布**: 通道2
- **功能**: 中频信号采样触发
- **频率范围**: 1kHz - 10kHz

**CAPTURE_0（频率捕获）：**
- **模式**: 组合捕获模式
- **定时器周期**: 8ms
- **时钟预分频**: 32
- **捕获引脚**: PA12
- **中断**: 启用CC1_DN和ZERO事件
- **功能**: 通过周期捕获直接测量频率

**COMPARE_0（边沿计数）：**
- **模式**: 边沿向上计数模式
- **定时器周期**: 65535个计数
- **时钟分频**: 8
- **时钟预分频**: 200
- **输出引脚**: PB0
- **功能**: 为低频测量计算信号边沿

### 通信接口

**I2C接口（OLED显示）：**
- **模式**: 控制器模式启用
- **数据引脚**: PA28（SDA）
- **时钟引脚**: PA31（SCL）
- **中断**: 仲裁丢失、NACK、RX/TX FIFO触发、传输完成
- **功能**: 与128x64 OLED显示屏通信

**UART接口（蓝牙通信）：**
- **FIFO**: 启用缓冲通信
- **发送引脚**: PA10
- **接收引脚**: PA11
- **波特率**: 配置为与HC-05模块兼容
- **功能**: 向外部设备进行无线数据传输

### 电压基准(VREF)

**VREF配置：**
- **基准输出**: PA23（VREF+）
- **就绪检查**: 启用稳定基准
- **高级时钟**: 选择总线时钟源
- **功能**: 为ADC和DAC操作提供稳定的电压基准

### GPIO配置

**用户接口：**
- **LED**: PA0（USER_LED_1）- 状态指示
- **按键**: PA18（USER_SWITCH_2）- 模式选择，带上拉电阻
- **中断**: 上升/下降沿检测用户输入

## 信号处理算法

### 频率测量策略

系统采用三层频率测量方法：

**1. 低频段（1Hz - 1kHz）：**
- **方法**: 1秒门控时间内的边沿计数
- **实现**: COMPARE_0定时器计算信号跳变
- **精度**: 对慢速信号具有高精度
- **计算**: 频率 = 边沿计数 / 门控时间

**2. 中频段（1kHz - 10kHz）：**
- **方法**: 定时器捕获直接测量周期
- **实现**: CAPTURE_0直接测量信号周期
- **精度**: 针对中频段优化
- **计算**: 频率 = 1 / 测量周期

**3. 高频段（10kHz - 100kHz）：**
- **方法**: FFT频谱分析
- **实现**: 使用ARM DSP库的1024点基4 FFT
- **采样率**: 250kHz有效速率
- **频率分辨率**: 每个频点244Hz
- **计算**: 频域峰值检测

### 波形分析

**峰峰值测量：**
- 分析ADC采样缓冲区找到最大值和最小值
- 转换ADC计数为电压：`电压 = (ADC值 / 4096) * 3300mV / 3`
- 考虑信号调理增益因子

**RMS计算：**
- 通过计算采样平均值去除直流分量
- 计算交流有效值：`RMS = sqrt(Σ(采样值 - 直流)² / N)`
- 提供真正的有效值测量

**波形分类：**
基于RMS与峰峰值比值分析：
- **正弦波**: 比值 = 0.32 - 0.4（理论值：0.354）
- **方波**: 比值 = 0.4 - 0.6（理论值：0.5）
- **三角波**: 比值 = 0.2 - 0.32（理论值：0.289）
- **未知**: 比值超出定义范围

### 音频信号识别

**双阈值检测：**
- **主阈值**: 3000 ADC计数，用于强音频信号
- **辅助阈值**: 2400 ADC计数，用于弱音频信号
- **采样缓冲**: 4096个采样点进行全面分析
- **模式匹配**: 基于持续时间分类为8种音频类别

**识别算法：**
1. 扫描整个4096采样点缓冲区
2. 计算超过各阈值的采样点数
3. 记录音频活动期间的最大阈值计数
4. 基于阈值计数模式对音频分类
5. 音频周期结束后重置计数器

## 引脚配置汇总

| 功能 | 引脚 | 配置 | 描述 |
|------|------|------|------|
| ADC0输入 | PA27 | 模拟输入 | 主信号通道 |
| ADC1输入 | PA15 | 模拟输入 | 音频信号通道 |
| OPA0输出 | PA22 | 模拟输出 | 调理信号输出 |
| OPA0负输入 | PA24 | 模拟输入 | 外部信号输入 |
| OPA1输出 | PA16 | 模拟输出 | 辅助调理输出 |
| OPA1负输入 | PA17 | 模拟输入 | 辅助信号输入 |
| COMP2输出 | PB27 | 数字输出 | 比较器结果 |
| COMP2正输入 | PB21 | 模拟输入 | 正比较输入 |
| COMP2负输入 | PB22 | 模拟输入 | 负比较输入 |
| 捕获输入 | PA12 | 数字输入 | 频率捕获 |
| 比较输出 | PB0 | 数字输出 | 边沿计数 |
| I2C数据 | PA28 | 开漏 | OLED数据线 |
| I2C时钟 | PA31 | 开漏 | OLED时钟线 |
| UART发送 | PA10 | 数字输出 | 蓝牙发送 |
| UART接收 | PA11 | 数字输入 | 蓝牙接收 |
| 电压基准 | PA23 | 模拟输出 | 电压基准 |
| 用户LED | PA0 | 数字输出 | 状态指示 |
| 用户按键 | PA18 | 数字输入 | 模式选择 |

## 性能规格

### 测量精度
- **频率精度**: ±0.1% ± 0.5Hz（满足竞赛要求）
- **峰峰值精度**: ±1% ± 1mV（满足竞赛要求）
- **输入电压范围**: 60mVpp - 1000mVpp（符合规定）
- **频率范围**: 1Hz - 100kHz（从基本1kHz-10kHz要求扩展）

### 系统性能
- **ADC分辨率**: 12位（4096级）
- **采样率**: 高达250kHz有效速率
- **FFT点数**: 1024点用于高频分析
- **音频缓冲**: 4096个采样点用于识别
- **显示更新**: OLED实时刷新
- **无线传输**: 连续数据流传输

## 编译和使用说明

### 先决条件
- 支持MSPM0的Code Composer Studio（CCS）
- MSPM0 SDK版本2.04.00.06或更高
- 用于FFT功能的ARM DSP库
- HC-05蓝牙模块
- 128x64 I2C OLED显示屏

### 硬件设置
1. 将信号源连接到PA24（OPA0负输入）
2. 将OLED显示屏连接到I2C引脚（PA28/PA31）
3. 将HC-05模块连接到UART引脚（PA10/PA11）
4. 确保正确的电源（3.3V）连接
5. 连接所有模块之间的地线

### 软件编译
1. 将项目导入Code Composer Studio
2. 验证MSPM0 SDK安装和版本
3. 使用提供的makefile编译项目
4. 解决ARM DSP库的任何依赖问题
5. 通过调试器对MSPM0G3507设备编程

### 操作说明
1. 系统上电并验证OLED显示初始化
2. 将信号源连接到输入端子
3. 系统自动检测频率范围并选择最佳测量方法
4. 在OLED显示屏上查看实时测量：
   - 第1行：频率值和单位
   - 第2行：峰峰值电压
   - 第3行：波形类型（正弦波/方波/三角波）
   - 第4行：音频识别结果
5. 监控蓝牙输出进行数据记录和远程监控

### 文件结构

```
G3507_ADC_MAX_FREQ_OLED_KEY_TIMER_UART/
├── G3507_Signal_Analysis_Device.c          # 主应用程序代码
├── G3507_Signal_Analysis_Device.syscfg     # 系统配置文件
├── ti_msp_dl_config.h                      # 生成的配置头文件
├── ti_msp_dl_config.c                      # 生成的配置源文件
├── OLED/                                   # OLED驱动文件
│   ├── oled.c                             # OLED驱动实现
│   ├── oled.h                             # OLED驱动头文件
│   ├── oledfont.h                         # 字体定义
│   └── bmp.h                              # 位图定义
├── Debug/                                  # 编译输出目录
│   ├── *.out                              # 可执行文件
│   ├── *.map                              # 内存映射文件
│   └── *.o                                # 目标文件
└── targetConfigs/                          # 目标配置
    ├── MSPM0G3507.ccxml                   # 目标配置
    └── readme.txt                         # 配置说明
```

## 竞赛要求符合性

本设计完全实现了竞赛要求：

### 基本要求 ✅
- **输入信号频率范围**: 1kHz-10kHz ✅
- **输入信号峰峰值范围**: 60mVpp-1000mVpp ✅
- **频率测量功能**: 精度0.1%+0.5Hz ✅
- **峰峰值测量功能**: 精度1%+1mV ✅

### 发挥部分 ✅
- **扩展频率范围**: 1Hz-100kHz（超越基本1kHz-10kHz要求）✅
- **音频识别功能**: 接触触发音频识别 ✅
- **附加功能**: OLED显示、蓝牙传输、自动量程切换 ✅

### 创新点
- 使用三种不同算法的自适应频率测量
- 双阈值音频识别系统
- 实时波形分类
- 无线数据传输能力
- 使用片内资源的综合信号调理

该系统通过精心设计的硬件配置和算法优化，完全满足竞赛的技术指标要求，并在发挥部分实现了显著创新。
