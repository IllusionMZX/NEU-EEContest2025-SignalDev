/*
 * Copyright (c) 2023, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.c =============
 *  Configured MSPM0 DriverLib module definitions
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */

#include "ti_msp_dl_config.h"

DL_TimerA_backupConfig gCOMPARE_0Backup;
DL_TimerG_backupConfig gTIMER_0Backup;
DL_TimerA_backupConfig gTIMER_1Backup;
DL_TimerG_backupConfig gTIMER_2Backup;

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform any initialization needed before using any board APIs
 */
SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    /* Module-Specific Initializations*/
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_CAPTURE_0_init();
    SYSCFG_DL_COMPARE_0_init();
    SYSCFG_DL_TIMER_0_init();
    SYSCFG_DL_TIMER_1_init();
    SYSCFG_DL_TIMER_2_init();
    SYSCFG_DL_I2C_OLED_init();
    SYSCFG_DL_UART_0_init();
    SYSCFG_DL_ADC12_0_init();
    SYSCFG_DL_ADC12_1_init();
    SYSCFG_DL_COMP_0_init();
    SYSCFG_DL_COMP_1_init();
    SYSCFG_DL_COMP_2_init();
    SYSCFG_DL_VREF_init();
    SYSCFG_DL_OPA_0_init();
    SYSCFG_DL_OPA_1_init();
    SYSCFG_DL_DMA_init();
    /* Ensure backup structures have no valid state */

	gCOMPARE_0Backup.backupRdy 	= false;
	gTIMER_0Backup.backupRdy 	= false;
	gTIMER_1Backup.backupRdy 	= false;
	gTIMER_2Backup.backupRdy 	= false;


}
/*
 * User should take care to save and restore register configuration in application.
 * See Retention Configuration section for more details.
 */
SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_saveConfiguration(COMPARE_0_INST, &gCOMPARE_0Backup);
	retStatus &= DL_TimerG_saveConfiguration(TIMER_0_INST, &gTIMER_0Backup);
	retStatus &= DL_TimerA_saveConfiguration(TIMER_1_INST, &gTIMER_1Backup);
	retStatus &= DL_TimerG_saveConfiguration(TIMER_2_INST, &gTIMER_2Backup);

    return retStatus;
}


SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_restoreConfiguration(COMPARE_0_INST, &gCOMPARE_0Backup, false);
	retStatus &= DL_TimerG_restoreConfiguration(TIMER_0_INST, &gTIMER_0Backup, false);
	retStatus &= DL_TimerA_restoreConfiguration(TIMER_1_INST, &gTIMER_1Backup, false);
	retStatus &= DL_TimerG_restoreConfiguration(TIMER_2_INST, &gTIMER_2Backup, false);

    return retStatus;
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_TimerG_reset(CAPTURE_0_INST);
    DL_TimerA_reset(COMPARE_0_INST);
    DL_TimerG_reset(TIMER_0_INST);
    DL_TimerA_reset(TIMER_1_INST);
    DL_TimerG_reset(TIMER_2_INST);
    DL_I2C_reset(I2C_OLED_INST);
    DL_UART_Main_reset(UART_0_INST);
    DL_ADC12_reset(ADC12_0_INST);
    DL_ADC12_reset(ADC12_1_INST);
    DL_COMP_reset(COMP_0_INST);
    DL_COMP_reset(COMP_1_INST);
    DL_COMP_reset(COMP_2_INST);
    DL_VREF_reset(VREF);
    DL_OPA_reset(OPA_0_INST);
    DL_OPA_reset(OPA_1_INST);


    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_TimerG_enablePower(CAPTURE_0_INST);
    DL_TimerA_enablePower(COMPARE_0_INST);
    DL_TimerG_enablePower(TIMER_0_INST);
    DL_TimerA_enablePower(TIMER_1_INST);
    DL_TimerG_enablePower(TIMER_2_INST);
    DL_I2C_enablePower(I2C_OLED_INST);
    DL_UART_Main_enablePower(UART_0_INST);
    DL_ADC12_enablePower(ADC12_0_INST);
    DL_ADC12_enablePower(ADC12_1_INST);
    DL_COMP_enablePower(COMP_0_INST);
    DL_COMP_enablePower(COMP_1_INST);
    DL_COMP_enablePower(COMP_2_INST);
    DL_VREF_enablePower(VREF);
    DL_OPA_enablePower(OPA_0_INST);
    DL_OPA_enablePower(OPA_1_INST);

    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{

    DL_GPIO_initPeripheralInputFunction(GPIO_CAPTURE_0_C0_IOMUX,GPIO_CAPTURE_0_C0_IOMUX_FUNC);

    DL_GPIO_initPeripheralInputFunction(GPIO_COMPARE_0_C0_IOMUX,GPIO_COMPARE_0_C0_IOMUX_FUNC);

    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_OLED_IOMUX_SDA,
        GPIO_I2C_OLED_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_OLED_IOMUX_SCL,
        GPIO_I2C_OLED_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_I2C_OLED_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_I2C_OLED_IOMUX_SCL);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_UART_0_IOMUX_TX, GPIO_UART_0_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_UART_0_IOMUX_RX, GPIO_UART_0_IOMUX_RX_FUNC);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_COMP_2_IOMUX_OUT, GPIO_COMP_2_IOMUX_OUT_FUNC);

    
	DL_GPIO_setAnalogInternalResistor(GPIO_OPA_0_IOMUX_IN1NEG, DL_GPIO_RESISTOR_PULL_UP);

    DL_GPIO_initDigitalInputFeatures(GPIO_SWITCHES_USER_SWITCH_2_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(GPIO_LEDS_USER_LED_1_IOMUX);

    DL_GPIO_setPins(GPIOA, GPIO_LEDS_USER_LED_1_PIN);
    DL_GPIO_enableOutput(GPIOA, GPIO_LEDS_USER_LED_1_PIN);
    DL_GPIO_setUpperPinsPolarity(GPIOA, DL_GPIO_PIN_18_EDGE_RISE_FALL);

}



SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{

	//Low Power Mode is configured to be SLEEP0
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);

    
	DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
	/* Set default configuration */
	DL_SYSCTL_disableHFXT();
	DL_SYSCTL_disableSYSPLL();
    DL_SYSCTL_enableMFPCLK();
	DL_SYSCTL_setMFPCLKSource(DL_SYSCTL_MFPCLK_SOURCE_SYSOSC);

}



/*
 * Timer clock configuration to be sourced by BUSCLK /  (32000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   1000000 Hz = 32000000 Hz / (1 * (31 + 1))
 */
static const DL_TimerG_ClockConfig gCAPTURE_0ClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 31U
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * CAPTURE_0_INST_LOAD_VALUE = (8ms * 1000000 Hz) - 1
 */
static const DL_TimerG_CaptureCombinedConfig gCAPTURE_0CaptureConfig = {
    .captureMode    = DL_TIMER_CAPTURE_COMBINED_MODE_PULSE_WIDTH_AND_PERIOD,
    .period         = CAPTURE_0_INST_LOAD_VALUE,
    .startTimer     = DL_TIMER_STOP,
    .inputChan      = DL_TIMER_INPUT_CHAN_0,
    .inputInvMode   = DL_TIMER_CC_INPUT_INV_NOINVERT,
};

SYSCONFIG_WEAK void SYSCFG_DL_CAPTURE_0_init(void) {

    DL_TimerG_setClockConfig(CAPTURE_0_INST,
        (DL_TimerG_ClockConfig *) &gCAPTURE_0ClockConfig);

    DL_TimerG_initCaptureCombinedMode(CAPTURE_0_INST,
        (DL_TimerG_CaptureCombinedConfig *) &gCAPTURE_0CaptureConfig);
    DL_TimerG_enableInterrupt(CAPTURE_0_INST , DL_TIMERG_INTERRUPT_CC1_DN_EVENT |
		DL_TIMERG_INTERRUPT_ZERO_EVENT);

    DL_TimerG_enableClock(CAPTURE_0_INST);

}


/*
 * Timer clock configuration to be sourced by BUSCLK /  (4000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   20000 Hz = 4000000 Hz / (8 * (199 + 1))
 */
static const DL_TimerA_ClockConfig gCOMPARE_0ClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_8,
    .prescale = 199U
};

static const DL_TimerA_CompareConfig gCOMPARE_0CompareConfig = {
    .compareMode    = DL_TIMER_COMPARE_MODE_EDGE_COUNT_UP,
    .count          = 65534,
    .startTimer     = DL_TIMER_STOP,
    .edgeDetectMode = DL_TIMER_COMPARE_EDGE_DETECTION_MODE_RISING,
    .inputChan      = DL_TIMER_INPUT_CHAN_0,
    .inputInvMode   = DL_TIMER_CC_INPUT_INV_NOINVERT,
};

SYSCONFIG_WEAK void SYSCFG_DL_COMPARE_0_init(void) {

    DL_TimerA_setClockConfig(COMPARE_0_INST,
        (DL_TimerA_ClockConfig *) &gCOMPARE_0ClockConfig);
    DL_TimerA_initCompareMode(COMPARE_0_INST,
        (DL_TimerA_CompareConfig *) &gCOMPARE_0CompareConfig);

    DL_TimerA_enableClock(COMPARE_0_INST);




}



/*
 * Timer clock configuration to be sourced by BUSCLK /  (32000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   32000000 Hz = 32000000 Hz / (1 * (0 + 1))
 */
static const DL_TimerG_ClockConfig gTIMER_0ClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale    = 0U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * TIMER_0_INST_LOAD_VALUE = (4us * 32000000 Hz) - 1
 */
static const DL_TimerG_TimerConfig gTIMER_0TimerConfig = {
    .period     = TIMER_0_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_TIMER_0_init(void) {

    DL_TimerG_setClockConfig(TIMER_0_INST,
        (DL_TimerG_ClockConfig *) &gTIMER_0ClockConfig);

    DL_TimerG_initTimerMode(TIMER_0_INST,
        (DL_TimerG_TimerConfig *) &gTIMER_0TimerConfig);
    DL_TimerG_enableClock(TIMER_0_INST);


    DL_TimerG_enableEvent(TIMER_0_INST, DL_TIMERG_EVENT_ROUTE_1, (DL_TIMERG_EVENT_ZERO_EVENT));

    DL_TimerG_setPublisherChanID(TIMER_0_INST, DL_TIMERG_PUBLISHER_INDEX_0, TIMER_0_INST_PUB_0_CH);



}

/*
 * Timer clock configuration to be sourced by BUSCLK /  (16000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   62500 Hz = 16000000 Hz / (2 * (255 + 1))
 */
static const DL_TimerA_ClockConfig gTIMER_1ClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_2,
    .prescale    = 255U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * TIMER_1_INST_LOAD_VALUE = (1s * 62500 Hz) - 1
 */
static const DL_TimerA_TimerConfig gTIMER_1TimerConfig = {
    .period     = TIMER_1_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC,
    .startTimer = DL_TIMER_STOP,
};

SYSCONFIG_WEAK void SYSCFG_DL_TIMER_1_init(void) {

    DL_TimerA_setClockConfig(TIMER_1_INST,
        (DL_TimerA_ClockConfig *) &gTIMER_1ClockConfig);

    DL_TimerA_initTimerMode(TIMER_1_INST,
        (DL_TimerA_TimerConfig *) &gTIMER_1TimerConfig);
    DL_TimerA_enableInterrupt(TIMER_1_INST , DL_TIMERA_INTERRUPT_ZERO_EVENT);
    DL_TimerA_enableClock(TIMER_1_INST);





}

/*
 * Timer clock configuration to be sourced by BUSCLK /  (32000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   32000000 Hz = 32000000 Hz / (1 * (0 + 1))
 */
static const DL_TimerG_ClockConfig gTIMER_2ClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale    = 0U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * TIMER_2_INST_LOAD_VALUE = (1ms * 32000000 Hz) - 1
 */
static const DL_TimerG_TimerConfig gTIMER_2TimerConfig = {
    .period     = TIMER_2_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_TIMER_2_init(void) {

    DL_TimerG_setClockConfig(TIMER_2_INST,
        (DL_TimerG_ClockConfig *) &gTIMER_2ClockConfig);

    DL_TimerG_initTimerMode(TIMER_2_INST,
        (DL_TimerG_TimerConfig *) &gTIMER_2TimerConfig);
    DL_TimerG_enableClock(TIMER_2_INST);


    DL_TimerG_enableEvent(TIMER_2_INST, DL_TIMERG_EVENT_ROUTE_1, (DL_TIMERG_EVENT_ZERO_EVENT));

    DL_TimerG_setPublisherChanID(TIMER_2_INST, DL_TIMERG_PUBLISHER_INDEX_0, TIMER_2_INST_PUB_0_CH);



}


static const DL_I2C_ClockConfig gI2C_OLEDClockConfig = {
    .clockSel = DL_I2C_CLOCK_BUSCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
};

SYSCONFIG_WEAK void SYSCFG_DL_I2C_OLED_init(void) {

    DL_I2C_setClockConfig(I2C_OLED_INST,
        (DL_I2C_ClockConfig *) &gI2C_OLEDClockConfig);
    DL_I2C_setAnalogGlitchFilterPulseWidth(I2C_OLED_INST,
        DL_I2C_ANALOG_GLITCH_FILTER_WIDTH_50NS);
    DL_I2C_enableAnalogGlitchFilter(I2C_OLED_INST);

    /* Configure Controller Mode */
    DL_I2C_resetControllerTransfer(I2C_OLED_INST);
    /* Set frequency to 100000 Hz*/
    DL_I2C_setTimerPeriod(I2C_OLED_INST, 31);
    DL_I2C_setControllerTXFIFOThreshold(I2C_OLED_INST, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(I2C_OLED_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(I2C_OLED_INST);

    /* Configure Interrupts */
    DL_I2C_enableInterrupt(I2C_OLED_INST,
                           DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST |
                           DL_I2C_INTERRUPT_CONTROLLER_NACK |
                           DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER |
                           DL_I2C_INTERRUPT_CONTROLLER_RX_DONE |
                           DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);


    /* Enable module */
    DL_I2C_enableController(I2C_OLED_INST);


}


static const DL_UART_Main_ClockConfig gUART_0ClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gUART_0Config = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_UART_0_init(void)
{
    DL_UART_Main_setClockConfig(UART_0_INST, (DL_UART_Main_ClockConfig *) &gUART_0ClockConfig);

    DL_UART_Main_init(UART_0_INST, (DL_UART_Main_Config *) &gUART_0Config);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 9600
     *  Actual baud rate: 9600.24
     */
    DL_UART_Main_setOversampling(UART_0_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(UART_0_INST, UART_0_IBRD_32_MHZ_9600_BAUD, UART_0_FBRD_32_MHZ_9600_BAUD);


    /* Configure FIFOs */
    DL_UART_Main_enableFIFOs(UART_0_INST);
    DL_UART_Main_setRXFIFOThreshold(UART_0_INST, DL_UART_RX_FIFO_LEVEL_1_2_FULL);
    DL_UART_Main_setTXFIFOThreshold(UART_0_INST, DL_UART_TX_FIFO_LEVEL_1_2_EMPTY);

    DL_UART_Main_enable(UART_0_INST);
}

/* ADC12_0 Initialization */
static const DL_ADC12_ClockConfig gADC12_0ClockConfig = {
    .clockSel       = DL_ADC12_CLOCK_SYSOSC,
    .divideRatio    = DL_ADC12_CLOCK_DIVIDE_1,
    .freqRange      = DL_ADC12_CLOCK_FREQ_RANGE_24_TO_32,
};
SYSCONFIG_WEAK void SYSCFG_DL_ADC12_0_init(void)
{
    DL_ADC12_setClockConfig(ADC12_0_INST, (DL_ADC12_ClockConfig *) &gADC12_0ClockConfig);
    DL_ADC12_initSingleSample(ADC12_0_INST,
        DL_ADC12_REPEAT_MODE_ENABLED, DL_ADC12_SAMPLING_SOURCE_AUTO, DL_ADC12_TRIG_SRC_EVENT,
        DL_ADC12_SAMP_CONV_RES_12_BIT, DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);
    DL_ADC12_configConversionMem(ADC12_0_INST, ADC12_0_ADCMEM_0,
        DL_ADC12_INPUT_CHAN_0, DL_ADC12_REFERENCE_VOLTAGE_VDDA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_TRIGGER_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_setPowerDownMode(ADC12_0_INST,DL_ADC12_POWER_DOWN_MODE_MANUAL);
    DL_ADC12_setSampleTime0(ADC12_0_INST,2);
    DL_ADC12_enableDMA(ADC12_0_INST);
    DL_ADC12_setDMASamplesCnt(ADC12_0_INST,1);
    DL_ADC12_enableDMATrigger(ADC12_0_INST,(DL_ADC12_DMA_MEM0_RESULT_LOADED));
    DL_ADC12_setSubscriberChanID(ADC12_0_INST,ADC12_0_INST_SUB_CH);
    /* Enable ADC12 interrupt */
    DL_ADC12_clearInterruptStatus(ADC12_0_INST,(DL_ADC12_INTERRUPT_DMA_DONE));
    DL_ADC12_enableInterrupt(ADC12_0_INST,(DL_ADC12_INTERRUPT_DMA_DONE));
    NVIC_SetPriority(ADC12_0_INST_INT_IRQN, 3);
    DL_ADC12_enableConversions(ADC12_0_INST);
}
/* ADC12_1 Initialization */
static const DL_ADC12_ClockConfig gADC12_1ClockConfig = {
    .clockSel       = DL_ADC12_CLOCK_SYSOSC,
    .divideRatio    = DL_ADC12_CLOCK_DIVIDE_1,
    .freqRange      = DL_ADC12_CLOCK_FREQ_RANGE_24_TO_32,
};
SYSCONFIG_WEAK void SYSCFG_DL_ADC12_1_init(void)
{
    DL_ADC12_setClockConfig(ADC12_1_INST, (DL_ADC12_ClockConfig *) &gADC12_1ClockConfig);
    DL_ADC12_initSingleSample(ADC12_1_INST,
        DL_ADC12_REPEAT_MODE_ENABLED, DL_ADC12_SAMPLING_SOURCE_AUTO, DL_ADC12_TRIG_SRC_EVENT,
        DL_ADC12_SAMP_CONV_RES_12_BIT, DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);
    DL_ADC12_configConversionMem(ADC12_1_INST, ADC12_1_ADCMEM_0,
        DL_ADC12_INPUT_CHAN_0, DL_ADC12_REFERENCE_VOLTAGE_VDDA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_TRIGGER_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_setPowerDownMode(ADC12_1_INST,DL_ADC12_POWER_DOWN_MODE_MANUAL);
    DL_ADC12_setSampleTime0(ADC12_1_INST,2);
    DL_ADC12_enableDMA(ADC12_1_INST);
    DL_ADC12_setDMASamplesCnt(ADC12_1_INST,1);
    DL_ADC12_enableDMATrigger(ADC12_1_INST,(DL_ADC12_DMA_MEM0_RESULT_LOADED));
    DL_ADC12_setSubscriberChanID(ADC12_1_INST,ADC12_1_INST_SUB_CH);
    /* Enable ADC12 interrupt */
    DL_ADC12_clearInterruptStatus(ADC12_1_INST,(DL_ADC12_INTERRUPT_DMA_DONE));
    DL_ADC12_enableInterrupt(ADC12_1_INST,(DL_ADC12_INTERRUPT_DMA_DONE));
    NVIC_SetPriority(ADC12_1_INST_INT_IRQN, 3);
    DL_ADC12_enableConversions(ADC12_1_INST);
}

/* COMP_0 Initialization */
static const DL_COMP_Config gCOMP_0Config = {
    .channelEnable = DL_COMP_ENABLE_CHANNEL_NONE,
    .mode          = DL_COMP_MODE_FAST,
    .negChannel    = DL_COMP_IMSEL_CHANNEL_0,
    .posChannel    = DL_COMP_IPSEL_CHANNEL_0,
    .hysteresis    = DL_COMP_HYSTERESIS_NONE,
    .polarity      = DL_COMP_POLARITY_NON_INV
};
static const DL_COMP_RefVoltageConfig gCOMP_0VRefConfig = {
    .mode           = DL_COMP_REF_MODE_STATIC,
    .source         = DL_COMP_REF_SOURCE_VREF_DAC,
    .terminalSelect = DL_COMP_REF_TERMINAL_SELECT_POS,
    .controlSelect  = DL_COMP_DAC_CONTROL_SW,
    .inputSelect    = DL_COMP_DAC_INPUT_DACCODE0
};

SYSCONFIG_WEAK void SYSCFG_DL_COMP_0_init(void)
{
    DL_COMP_init(COMP_0_INST, (DL_COMP_Config *) &gCOMP_0Config);
    DL_COMP_refVoltageInit(COMP_0_INST, (DL_COMP_RefVoltageConfig *) &gCOMP_0VRefConfig);
    DL_COMP_setDACCode0(COMP_0_INST, COMP_0_DACCODE0);

    DL_COMP_enable(COMP_0_INST);

}

/* COMP_1 Initialization */
static const DL_COMP_Config gCOMP_1Config = {
    .channelEnable = DL_COMP_ENABLE_CHANNEL_NONE,
    .mode          = DL_COMP_MODE_FAST,
    .negChannel    = DL_COMP_IMSEL_CHANNEL_0,
    .posChannel    = DL_COMP_IPSEL_CHANNEL_0,
    .hysteresis    = DL_COMP_HYSTERESIS_NONE,
    .polarity      = DL_COMP_POLARITY_NON_INV
};
static const DL_COMP_RefVoltageConfig gCOMP_1VRefConfig = {
    .mode           = DL_COMP_REF_MODE_STATIC,
    .source         = DL_COMP_REF_SOURCE_VREF_DAC,
    .terminalSelect = DL_COMP_REF_TERMINAL_SELECT_POS,
    .controlSelect  = DL_COMP_DAC_CONTROL_SW,
    .inputSelect    = DL_COMP_DAC_INPUT_DACCODE0
};

SYSCONFIG_WEAK void SYSCFG_DL_COMP_1_init(void)
{
    DL_COMP_init(COMP_1_INST, (DL_COMP_Config *) &gCOMP_1Config);
    DL_COMP_refVoltageInit(COMP_1_INST, (DL_COMP_RefVoltageConfig *) &gCOMP_1VRefConfig);
    DL_COMP_setDACCode0(COMP_1_INST, COMP_1_DACCODE0);

    DL_COMP_enable(COMP_1_INST);

}

/* COMP_2 Initialization */
static const DL_COMP_Config gCOMP_2Config = {
    .channelEnable = DL_COMP_ENABLE_CHANNEL_POS_NEG,
    .mode          = DL_COMP_MODE_FAST,
    .negChannel    = DL_COMP_IMSEL_CHANNEL_0,
    .posChannel    = DL_COMP_IPSEL_CHANNEL_0,
    .hysteresis    = DL_COMP_HYSTERESIS_20,
    .polarity      = DL_COMP_POLARITY_NON_INV
};
static const DL_COMP_RefVoltageConfig gCOMP_2VRefConfig = {
    .mode           = DL_COMP_REF_MODE_STATIC,
    .source         = DL_COMP_REF_SOURCE_NONE,
    .terminalSelect = DL_COMP_REF_TERMINAL_SELECT_POS,
    .controlSelect  = DL_COMP_DAC_CONTROL_COMP_OUT,
    .inputSelect    = DL_COMP_DAC_INPUT_DACCODE0
};

SYSCONFIG_WEAK void SYSCFG_DL_COMP_2_init(void)
{
    DL_COMP_init(COMP_2_INST, (DL_COMP_Config *) &gCOMP_2Config);
    DL_COMP_enableOutputFilter(COMP_2_INST,DL_COMP_FILTER_DELAY_1200);
    DL_COMP_refVoltageInit(COMP_2_INST, (DL_COMP_RefVoltageConfig *) &gCOMP_2VRefConfig);

    DL_COMP_enable(COMP_2_INST);

}



static const DL_VREF_ClockConfig gVREFClockConfig = {
    .clockSel = DL_VREF_CLOCK_BUSCLK,
    .divideRatio = DL_VREF_CLOCK_DIVIDE_1,
};
static const DL_VREF_Config gVREFConfig = {
    .vrefEnable     = DL_VREF_ENABLE_ENABLE,
    .bufConfig      = DL_VREF_BUFCONFIG_OUTPUT_1_4V,
    .shModeEnable   = DL_VREF_SHMODE_DISABLE,
    .holdCycleCount = DL_VREF_HOLD_MIN,
    .shCycleCount   = DL_VREF_SH_MIN,
};

SYSCONFIG_WEAK void SYSCFG_DL_VREF_init(void) {
    DL_VREF_setClockConfig(VREF,
        (DL_VREF_ClockConfig *) &gVREFClockConfig);
    DL_VREF_configReference(VREF,
        (DL_VREF_Config *) &gVREFConfig);
    delay_cycles(VREF_READY_DELAY);
}


static const DL_OPA_Config gOPA_0Config0 = {
    .pselChannel    = DL_OPA_PSEL_DAC8_OUT,
    .nselChannel    = DL_OPA_NSEL_RTAP,
    .mselChannel    = DL_OPA_MSEL_IN1_NEG,
    .gain           = DL_OPA_GAIN_N3_P4,
    .outputPinState = DL_OPA_OUTPUT_PIN_ENABLED,
    .choppingMode   = DL_OPA_CHOPPING_MODE_ADC_AVERAGING,
};

SYSCONFIG_WEAK void SYSCFG_DL_OPA_0_init(void)
{
    DL_OPA_init(OPA_0_INST, (DL_OPA_Config *) &gOPA_0Config0);
    DL_OPA_setGainBandwidth(OPA_0_INST, DL_OPA_GBW_HIGH);

    DL_OPA_enable(OPA_0_INST);
}
static const DL_OPA_Config gOPA_1Config0 = {
    .pselChannel    = DL_OPA_PSEL_DAC8_OUT,
    .nselChannel    = DL_OPA_NSEL_RTAP,
    .mselChannel    = DL_OPA_MSEL_IN1_NEG,
    .gain           = DL_OPA_GAIN_N3_P4,
    .outputPinState = DL_OPA_OUTPUT_PIN_ENABLED,
    .choppingMode   = DL_OPA_CHOPPING_MODE_ADC_AVERAGING,
};

SYSCONFIG_WEAK void SYSCFG_DL_OPA_1_init(void)
{
    DL_OPA_init(OPA_1_INST, (DL_OPA_Config *) &gOPA_1Config0);
    DL_OPA_setGainBandwidth(OPA_1_INST, DL_OPA_GBW_HIGH);

    DL_OPA_enable(OPA_1_INST);
}

static const DL_DMA_Config gDMA_CH0Config = {
    .transferMode   = DL_DMA_FULL_CH_REPEAT_SINGLE_TRANSFER_MODE,
    .extendedMode   = DL_DMA_NORMAL_MODE,
    .destIncrement  = DL_DMA_ADDR_INCREMENT,
    .srcIncrement   = DL_DMA_ADDR_UNCHANGED,
    .destWidth      = DL_DMA_WIDTH_HALF_WORD,
    .srcWidth       = DL_DMA_WIDTH_HALF_WORD,
    .trigger        = ADC12_0_INST_DMA_TRIGGER,
    .triggerType    = DL_DMA_TRIGGER_TYPE_EXTERNAL,
};

SYSCONFIG_WEAK void SYSCFG_DL_DMA_CH0_init(void)
{
    DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, 1024);
    DL_DMA_initChannel(DMA, DMA_CH0_CHAN_ID , (DL_DMA_Config *) &gDMA_CH0Config);
}
static const DL_DMA_Config gDMA_CH1Config = {
    .transferMode   = DL_DMA_FULL_CH_REPEAT_SINGLE_TRANSFER_MODE,
    .extendedMode   = DL_DMA_NORMAL_MODE,
    .destIncrement  = DL_DMA_ADDR_INCREMENT,
    .srcIncrement   = DL_DMA_ADDR_UNCHANGED,
    .destWidth      = DL_DMA_WIDTH_HALF_WORD,
    .srcWidth       = DL_DMA_WIDTH_HALF_WORD,
    .trigger        = ADC12_1_INST_DMA_TRIGGER,
    .triggerType    = DL_DMA_TRIGGER_TYPE_EXTERNAL,
};

SYSCONFIG_WEAK void SYSCFG_DL_DMA_CH1_init(void)
{
    DL_DMA_setTransferSize(DMA, DMA_CH1_CHAN_ID, 4096);
    DL_DMA_initChannel(DMA, DMA_CH1_CHAN_ID , (DL_DMA_Config *) &gDMA_CH1Config);
}
SYSCONFIG_WEAK void SYSCFG_DL_DMA_init(void){
    SYSCFG_DL_DMA_CH0_init();
    SYSCFG_DL_DMA_CH1_init();
}


