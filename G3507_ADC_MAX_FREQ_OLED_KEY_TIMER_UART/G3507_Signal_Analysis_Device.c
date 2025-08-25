#include "arm_const_structs.h"
#include "arm_math.h"
#include "ti_msp_dl_config.h"
#include "stdio.h"
#include "bmp.h"
#include "oled.h"

//================FREQ================
unsigned long ms_Tick = 0;
volatile uint32_t gCaptureCnt;
volatile bool gSynced;
uint32_t freq_cnt = 0;
uint32_t gLoadValue;
unsigned char state_flag = 0;
unsigned char state_before = 0;
volatile static uint32_t pwmPeriod;
float32_t freq_capture=0.0;
float32_t freq_timer = 0.0;
float32_t freq_fft = 0.0;

//==============ADC==================
#define SINE_WAVE 0x0a
#define RECT_WAVE 0x0b
#define TRIANGLE_WAVE 0x0c
#define UNKNOWN_WAVE 0x0d
#define ADC_SAMPLE_SIZE (1024)
#define ADC_SAMPLE_SIZE_AUDIO (4096)
uint16_t gADCSamples[ADC_SAMPLE_SIZE];
uint16_t gADCSamples_Audio[ADC_SAMPLE_SIZE_AUDIO];
float32_t peakToPeak;
float rms;
uint16_t waveformType;

//=========KEY===============
#define SIGNAL_MODE 0x10
#define AUDIO_MODE 0x20
uint32_t Key_Value = 0;
uint32_t Key_Down = 0;
uint32_t Key_Old = 0;
void Key_Proc();

//===========FFT===============
#define FFT_LENGTH  1024
#define pi  3.1415f
volatile uint16_t maxindex; //最大值的索引
float32_t maxvalue; //最大值
/* Array with FFT results */
volatile int16_t gFFTOutput[ADC_SAMPLE_SIZE * 2];
/* Maximum amplitude of FFT */
volatile uint32_t gFFTmaxValue;
/* Index of maximum frequency */
volatile uint32_t gFFTmaxFreqIndex;
arm_cfft_radix4_instance_f32 scfft; //FFT实例结构体
float32_t FFT_InputBuf[FFT_LENGTH * 2]; //FFT输入数组
float32_t FFT_OutputBuf[FFT_LENGTH]; //FFT输出数组

//=============音频识别============
//音频检测添加全局变量
#define THRESHOLD 3000
#define THRESHOLD_2 2400
#define NUM_AUDIO_FILES 8
volatile bool gTriggered = false;
volatile bool gTriggered_2 = false;
uint32_t gThresholdCount = 0;
uint32_t gThresholdCount_2=0;

// 定义状态变量
volatile uint32_t gMaxThreshold1 = 0;      // 记录当前音频期间gThresholdCount的最大值
volatile uint32_t gMaxThreshold2 = 0;      // 记录当前音频期间gThresholdCount_2的最大值
volatile bool gAudioActive = false;        // 标记音频是否在采集过程中
uint8_t matchIndex = 0;

//==============添加音频检测函数===============
void detect_audio_match(void) {
    gTriggered = false;
    gTriggered_2=false;
    gThresholdCount = 0;
    gThresholdCount_2=0;
    bool counting= false;
    bool counting_2=false;
    // 遍历整个采样数组
    for (int i = 0; i < ADC_SAMPLE_SIZE_AUDIO; i++) {
        // 检测首次触发
        if (!counting && (gADCSamples_Audio[i] > THRESHOLD)) {
            counting = true;
            gTriggered = true;
        }    
        // 触发后开始计数
        if (counting && (gADCSamples_Audio[i] > THRESHOLD)) {
            gThresholdCount++;
        }

         if (!counting_2 && (gADCSamples_Audio[i] > THRESHOLD_2)) {
            counting_2 = true;
            gTriggered_2 = true;
        }
        // 触发后开始计数
        if (counting_2 && (gADCSamples_Audio[i] > THRESHOLD_2)) {
            gThresholdCount_2++;
        }
    }
      if (gTriggered) {
        if (gThresholdCount > gMaxThreshold1) {
            gMaxThreshold1 = gThresholdCount; // 更新最大值
        }
        gAudioActive = true; // 标记音频正在采集
        gTriggered = false;
    }
     if (gTriggered_2) {
        if (gThresholdCount_2 > gMaxThreshold2) {
            gMaxThreshold2 = gThresholdCount_2; // 更新最大值
        }
        gAudioActive = true; // 标记音频正在采集
        gTriggered_2 = false;
    }
        if (gAudioActive && (gThresholdCount == 0 || gThresholdCount_2 == 0)) 
        {
        gAudioActive = false; // 标记音频结束
    }     
}

//==============UART===============
char Uart_Send_String[11];
void UART_SendString(char *str)
{
    while (*str!='\0')
    {
        while(DL_UART_isBusy(UART_0_INST)==true);
        DL_UART_Main_transmitData(UART_0_INST,*str++);
    }
}


//========峰峰值、有效值、波形判断==========
// 函数：计算峰峰值
float32_t calculatePeakToPeak(uint16_t *samples, size_t size) {
    uint16_t maxVal = samples[0];
    uint16_t minVal = samples[0];
    float32_t ppV = 0.0;
    for (size_t i = 1; i < size; i++) {
        if (samples[i] > maxVal) {
            maxVal = samples[i];
        }
        if (samples[i] < minVal) {
            minVal = samples[i];
        }
    }
    ppV = ((maxVal - minVal)/4096.0)*3300/3.0;
    return ppV;
}


// 函数：计算RMS（有效值）
float calculateRMS(uint16_t *samples, size_t size) {
    // 减去直流分量后
    // 1. 计算直流分量（平均值）
    uint32_t sum = 0;
    for (size_t i = 0; i < size; i++) {
        sum += samples[i];
    }
    float dc = (float)sum / size;

    // 2. 计算去除直流分量后的有效值
    float sumOfSquares = 0.0f;
    for (size_t i = 0; i < size; i++) {
        float ac = samples[i] - dc;
        sumOfSquares += ac * ac;
    }

    return (sqrt(sumOfSquares / size)/4096.0)*3300/3.0;
}

// 函数：判断波形类型
uint16_t determineWaveformType(double rms, double peakToPeak) {
    double ratio = rms / peakToPeak; // RMS与峰值的比值

    // 根据比值判断波形类型
    if (ratio >= 0.4 && ratio < 0.6) {
        return RECT_WAVE;
    } else if (ratio < 0.32 && ratio > 0.2) {
        return TRIANGLE_WAVE; 
    } else if (ratio >= 0.32 && ratio < 0.4) {
        return SINE_WAVE;
    } else {
        return UNKNOWN_WAVE;
    }
}

//=============FFT===================
void Do_FTT(void) //FFT变换
{
    for (uint16_t i = 0; i < FFT_LENGTH; i ++)
    {
        FFT_InputBuf[2 * i] = gADCSamples[i] * 3.3f / 4096; //实部，ADC转换值
        FFT_InputBuf[2 * i + 1] = 0; //虚部，0
    }
    arm_cfft_radix4_init_f32(&scfft, FFT_LENGTH, 0, 1); //初始化，正向FFT且启用位反转
    arm_cfft_radix4_f32(&scfft, FFT_InputBuf); //执行FFT，结果覆盖FFT_InputBuf
    arm_cmplx_mag_f32(FFT_InputBuf, FFT_OutputBuf, FFT_LENGTH); //计算每个频率点的幅值，存入FFT_OutputBuf

    //找到最大值
    maxvalue = 0;
    for (uint16_t i = 2; i < FFT_LENGTH / 2; i ++)
    {
        if (FFT_OutputBuf[i] > maxvalue)
        {
            maxvalue = FFT_OutputBuf[i];
            maxindex = i;
        }
    }
    
    //计算频率及峰峰值
    //采样率为250kHz/(1+arr)，频率精度受限于FFT点数(1024)及LoadValue
    freq_fft = (maxindex * 250000.0f) / 1024.0f;
    
    if(state_flag==SIGNAL_MODE){
        if (freq_fft>10000) {
            OLED_ShowNum(40, 0, freq_fft, 6, 16);
            printf("Frequency: %.2lf Hz\n", freq_fft);       
        }else if((freq_fft<1000)|(freq_capture<1000)){
            OLED_ShowNum(40, 0, freq_timer, 6, 16);
            printf("Frequency: %.2lf Hz\n", freq_timer);  
        }else {
            OLED_ShowNum(40, 0, freq_capture, 6, 16);
            printf("Frequency: %.2lf Hz\n", freq_capture);  
        }
        OLED_ShowString(110, 0, (uint8_t *)"Hz", 16);        
    }
    if ((freq_capture<1000)|(freq_fft<1000)) {
        DL_Timer_setLoadValue(TIMER_0_INST, 32000);
    }else{
        DL_Timer_setLoadValue(TIMER_0_INST, 128);
    }
}

//=================main====================
int main(void)
{
    SYSCFG_DL_init();

    //============TIMER==================
    gLoadValue = DL_TimerG_getLoadValue(CAPTURE_0_INST);
    gSynced        = false;
    DL_TimerG_setCoreHaltBehavior(CAPTURE_0_INST, DL_TIMER_CORE_HALT_IMMEDIATE);
    NVIC_EnableIRQ(CAPTURE_0_INST_INT_IRQN);
    DL_TimerG_startCounter(CAPTURE_0_INST);
    DL_TimerA_startCounter(COMPARE_0_INST);
    NVIC_EnableIRQ(TIMER_1_INST_INT_IRQN);
    DL_TimerA_startCounter(TIMER_1_INST);

    //==============OLED======================
    state_flag = SIGNAL_MODE;

    OLED_Init(); // 初始化OLED
    OLED_DrawBMP(0, 0, 128, 64, BMP1);
    OLED_Clear();
    OLED_ShowChinese(0, 0, 0, 16);
    OLED_ShowChinese(18, 0, 1, 16);  
    OLED_ShowChinese(0, 2, 2, 16);
    OLED_ShowChinese(18, 2, 2, 16);
    OLED_ShowChinese(36, 2, 3, 16);
    OLED_ShowChinese(72, 4, 14, 16);
    OLED_ShowChinese(90, 4, 15, 16);

    //===================ADC=================

    // ADC0配置，采样信号波形
    DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID,
        (uint32_t) 0x40556280);
    DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t) &gADCSamples[0]);
    DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);
    NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);
    DL_ADC12_startConversion(ADC12_0_INST);

    // ADC1配置，采样音频波形
    DL_DMA_setSrcAddr(DMA, DMA_CH1_CHAN_ID,
        (uint32_t) 0x40558280);
    DL_DMA_setDestAddr(DMA, DMA_CH1_CHAN_ID, (uint32_t) &gADCSamples_Audio[0]);
    DL_DMA_enableChannel(DMA, DMA_CH1_CHAN_ID);
    NVIC_EnableIRQ(ADC12_1_INST_INT_IRQN);
    DL_ADC12_startConversion(ADC12_1_INST);

    while (1) {
        pwmPeriod = gLoadValue - gCaptureCnt;
        freq_capture = 1/(pwmPeriod*1*0.000001);
        //Key_Proc();
        Do_FTT();
        printf("RMS Value: %.2fmV\n", rms);
        printf("Peak-to-Peak: %.2fmV\n", peakToPeak);
        if(state_flag==SIGNAL_MODE){
            OLED_ShowNum(60, 2, peakToPeak, 6, 16);
            OLED_ShowString(110, 2, (uint8_t *)"mV", 16);
            // 判断波形类型
            waveformType = determineWaveformType(rms, peakToPeak);

            switch (waveformType) {
                case SINE_WAVE:
                    printf("Waveform Type: 正弦波\n");
                    OLED_ShowChinese(0, 4, 8, 16);
                    OLED_ShowChinese(18, 4, 9, 16);
                    OLED_ShowChinese(36, 4, 10, 16);
                    break;
                case RECT_WAVE:
                    printf("Waveform Type: 方波\n");
                    OLED_ShowChinese(0, 4, 11, 16);
                    OLED_ShowChinese(18, 4, 29, 16);
                    OLED_ShowChinese(36, 4, 10, 16);
                    break;
                case TRIANGLE_WAVE:
                    printf("Waveform Type: 三角波\n");
                    OLED_ShowChinese(0, 4, 12, 16);
                    OLED_ShowChinese(18, 4, 13, 16);
                    OLED_ShowChinese(36, 4, 10, 16);
                    break;
                case UNKNOWN_WAVE:
                    printf("Waveform Type: 未知波形\n");
                    break;
                default:
                    break;
            }
            detect_audio_match();
            if (!gAudioActive && (gMaxThreshold1 > 0 || gMaxThreshold2 > 0))
            {               
                if(gMaxThreshold1 >100) {
                    matchIndex = 8;
                }
                else if (gMaxThreshold1>55) {
                    matchIndex = 6;
                }else if (gMaxThreshold1>45) {
                    matchIndex = 5;
                }else if (gMaxThreshold1>30) {
                    matchIndex = 1;
                }else if (gMaxThreshold1>15&&gMaxThreshold1<30) {
                    matchIndex = 7;
                }else if (gMaxThreshold1<15)
                 {
                    if (gMaxThreshold2>330) {
                        matchIndex=3;
                    }
                    else if (gMaxThreshold2>240) {
                        matchIndex=2;
                    }
                    else {
                        matchIndex=4;
                    }
                }
                printf("匹配结果: Audio%d (阈值1最大值=%d, 阈值2最大值=%d) \r\n", 
                matchIndex, gMaxThreshold1, gMaxThreshold2);
                OLED_ShowNum(108, 4, matchIndex, 1, 16);
                // 重置状态
                gMaxThreshold1 = 0;
                gMaxThreshold2 = 0;
            }
        }
    }
    return 0;
}
 
//===============Key===============
// void GROUP1_IRQHandler(void)
// {
//     uint32_t gpioA = DL_GPIO_getEnabledInterruptStatus(GPIOA, GPIO_SWITCHES_USER_SWITCH_2_PIN);

//     if ((gpioA & GPIO_SWITCHES_USER_SWITCH_2_PIN) == GPIO_SWITCHES_USER_SWITCH_2_PIN) {
//         state_flag = AUDIO_MODE;
//     }

//     // uint32_t gpioB = DL_GPIO_getEnabledInterruptStatus(GPIOB, GPIO_SWITCHES_USER_SWITCH_1_PIN);

//     // if ((gpioB & GPIO_SWITCHES_USER_SWITCH_1_PIN) == GPIO_SWITCHES_USER_SWITCH_1_PIN) {
//     //     state_flag = SIGNAL_MODE;
//     // }
// }

void Key_Proc()
{
    Key_Value = DL_GPIO_readPins(GPIO_SWITCHES_PORT,GPIO_SWITCHES_USER_SWITCH_2_PIN);
    Key_Down = Key_Value & (Key_Old ^ Key_Value);
    Key_Old = Key_Value;
    switch (Key_Down) {
       case 0x00040000:
            // printf("SW1 PRESS\n");
            // state_flag = AUDIO_MODE;
            // DL_GPIO_togglePins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);
            break;
        default:
            break;
    }
}

//==============ADC中断=================

//=============测信号ADC================
void ADC12_0_INST_IRQHandler(void)
{
    switch (DL_ADC12_getPendingInterrupt(ADC12_0_INST)) {
        case DL_ADC12_IIDX_DMA_DONE:
            DL_TimerA_stopCounter(TIMER_0_INST);
            //DL_ADC12_disableConversions(ADC12_0_INST);
            //__BKPT(0);
            rms = calculateRMS(gADCSamples, ADC_SAMPLE_SIZE);
            peakToPeak = calculatePeakToPeak(gADCSamples, ADC_SAMPLE_SIZE);
            //DL_ADC12_enableConversions(ADC12_0_INST);
            //DL_ADC12_startConversion(ADC12_0_INST);
            DL_TimerA_startCounter(TIMER_0_INST);
            break;
        default:
            break;
    }
}

//==============测音频ADC==============
void ADC12_1_INST_IRQHandler(void)
{
    switch (DL_ADC12_getPendingInterrupt(ADC12_1_INST)) {
        case DL_ADC12_IIDX_DMA_DONE:
            // DL_TimerG_stopCounter(TIMER_2_INST);
            // //DL_ADC12_disableConversions(ADC12_0_INST);
            // //DL_ADC12_enableConversions(ADC12_0_INST);
            // //DL_ADC12_startConversion(ADC12_0_INST);
            // DL_TimerG_startCounter(TIMER_2_INST);
            break;
        default:
            break;
    }
}

//==============测频率 定时器捕获中断============
void CAPTURE_0_INST_IRQHandler(void)
{
    switch (DL_TimerG_getPendingInterrupt(CAPTURE_0_INST)) {
        case DL_TIMERG_IIDX_CC1_DN:
            if (gSynced == true) {
                gCaptureCnt = DL_TimerG_getCaptureCompareValue(
                    CAPTURE_0_INST, DL_TIMER_CC_1_INDEX);
            } else {
                gSynced = true;
            }
            DL_TimerG_setTimerCount(CAPTURE_0_INST, gLoadValue);
            break;
        case DL_TIMERG_IIDX_ZERO:
            gSynced = false;
            break;
        default:
            break;
    }
}



//===============串口 定时器中断================
void TIMER_1_INST_IRQHandler(void)
{
    ms_Tick++;
    switch (DL_TimerA_getPendingInterrupt(TIMER_1_INST)) {
        case DL_TIMER_IIDX_ZERO:
            if(ms_Tick%1==0){
                sprintf(Uart_Send_String,"Type:%x\n", waveformType);
                UART_SendString(Uart_Send_String);
                sprintf(Uart_Send_String,"Ampl:%.2f\n", peakToPeak/2.0);
                UART_SendString(Uart_Send_String);
                if(state_flag==SIGNAL_MODE){
                    if (freq_fft>10000) {
                        sprintf(Uart_Send_String,"Freq:%.2lf\n", freq_fft); 
                    }else if(freq_fft<1000){
                        sprintf(Uart_Send_String,"Freq:%.2lf\n", freq_timer); 
                    }else {
                        sprintf(Uart_Send_String,"Freq:%.2lf\n", freq_capture); 
                    }       
                }else {
                    sprintf(Uart_Send_String,"Freq:%.2lf\n", 0.0);
                }
                UART_SendString(Uart_Send_String);
                sprintf(Uart_Send_String,"audio%d\n", matchIndex);
                UART_SendString(Uart_Send_String);                
            }
            DL_TimerA_stopCounter(COMPARE_0_INST);
            freq_cnt = DL_Timer_getTimerCount(COMPARE_0_INST);
            DL_Timer_setTimerCount(COMPARE_0_INST,0);
            freq_timer = freq_cnt;
            DL_TimerA_startCounter(COMPARE_0_INST);
            break;
        default:
            break;
    }
}