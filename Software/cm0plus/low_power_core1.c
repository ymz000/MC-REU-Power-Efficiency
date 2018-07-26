/*
 * The Clear BSD License
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided
 *  that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS LICENSE.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "board.h"
#include "fsl_mailbox.h"
#include "static_queue.h"
#include "low_power.h"
#include "fsl_adc.h"
#include "fsl_utick.h"
#include "fsl_clock.h"
#include "fsl_power.h"

#include "fsl_common.h"
#include "pin_mux.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define CORE_CLK_FREQ (CLOCK_GetFreq(kCLOCK_MainClk)/4)
#define APP_I2C_MASTER (I2C0)

#define QUEUE_ELEMENT_SIZE 40
#define QUEUE_ELEMENT_COUNT 200

#define DEMO_ADC_BASE ADC0
#define DEMO_ADC_CURRENT_CHANNEL_NUMBER 1U
#define DEMO_ADC_VOLTAGE_CHANNEL_NUMBER 7U
#define DEMO_ADC_IRQ_ID ADC0_SEQA_IRQn
#define DEMO_ADC_IRQ_HANDLER_FUNC ADC0_SEQA_IRQHandler

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
uint32_t volatile g_queue_buffer[QUEUE_ELEMENT_COUNT];
bool volatile g_readSensorsData = true;
bool volatile g_flashPowerUp = true;
volatile bool gAdcConvSeqAIntFlag;
static adc_result_info_t gAdcResultInfoStruct;
adc_result_info_t *volatile gAdcResultInfoPtr = &gAdcResultInfoStruct;


/*******************************************************************************
 * Code
 ******************************************************************************/

void MAILBOX_IRQHandler()
{
    core_cmd_t cmd = (core_cmd_t)MAILBOX_GetValue(MAILBOX, kMAILBOX_CM0Plus);
    if (cmd == kTurnOffFlash)
    {
        POWER_PowerDownFlash();
        g_flashPowerUp = false;
    }
    MAILBOX_ClearValueBits(MAILBOX, kMAILBOX_CM0Plus, 0xffffffff);
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

/*
 * ISR for ADC conversion sequence A done.
 */
void DEMO_ADC_IRQ_HANDLER_FUNC(void)
{
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

static void ADC_ClockPower_Configuration(void)
{
    /* SYSCON power. */
    POWER_DisablePD(kPDRUNCFG_PD_ADC0);  /* Power on the ADC converter. */
    POWER_DisablePD(kPDRUNCFG_PD_VDDA);  /* Power on the analog power supply. */
    POWER_DisablePD(kPDRUNCFG_PD_VREFP); /* Power on the reference voltage source. */
    /* Enable the clock. */
    CLOCK_AttachClk(kSYS_PLL_OUT_to_MAIN_CLK);

    CLOCK_AttachClk(kMAIN_CLK_to_ADC_CLK); 
    /* Sync clock source is not used. Using sync clock source and would be divided by 2.
     * The divider would be set when configuring the converter.
     */

    CLOCK_EnableClock(kCLOCK_Adc0); /* SYSCON->AHBCLKCTRL[0] |= SYSCON_AHBCLKCTRL_ADC0_MASK; */
}

void ADC_Configuration(void)
{
    adc_config_t adcConfigStruct;
    adc_conv_seq_config_t adcConvSeqConfigStruct;

    /* Configure the converter. */
#if defined(FSL_FEATURE_ADC_HAS_CTRL_ASYNMODE) & FSL_FEATURE_ADC_HAS_CTRL_ASYNMODE
    adcConfigStruct.clockMode = kADC_ClockSynchronousMode; /* Using sync clock source. */
#endif/* FSL_FEATURE_ADC_HAS_CTRL_ASYNMODE */
    adcConfigStruct.clockDividerNumber = 1;                /* The divider for sync clock is 2. */
#if defined(FSL_FEATURE_ADC_HAS_CTRL_RESOL) & FSL_FEATURE_ADC_HAS_CTRL_RESOL
    adcConfigStruct.resolution = kADC_Resolution12bit;
#endif/* FSL_FEATURE_ADC_HAS_CTRL_RESOL */
#if defined(FSL_FEATURE_ADC_HAS_CTRL_BYPASSCAL) & FSL_FEATURE_ADC_HAS_CTRL_BYPASSCAL
    adcConfigStruct.enableBypassCalibration = false;
#endif/* FSL_FEATURE_ADC_HAS_CTRL_BYPASSCAL */
#if defined(FSL_FEATURE_ADC_HAS_CTRL_TSAMP) & FSL_FEATURE_ADC_HAS_CTRL_TSAMP
    adcConfigStruct.sampleTimeNumber = 0U;
#endif/* FSL_FEATURE_ADC_HAS_CTRL_TSAMP */
#if defined(FSL_FEATURE_ADC_HAS_CTRL_LPWRMODE) & FSL_FEATURE_ADC_HAS_CTRL_LPWRMODE
    adcConfigStruct.enableLowPowerMode = false;
#endif/* FSL_FEATURE_ADC_HAS_CTRL_LPWRMODE */
#if defined(FSL_FEATURE_ADC_HAS_TRIM_REG) & FSL_FEATURE_ADC_HAS_TRIM_REG
   adcConfigStruct.voltageRange = kADC_HighVoltageRange;
#endif/* FSL_FEATURE_ADC_HAS_TRIM_REG */
    ADC_Init(DEMO_ADC_BASE, &adcConfigStruct);

#if !(defined(FSL_FEATURE_ADC_HAS_NO_INSEL) && FSL_FEATURE_ADC_HAS_NO_INSEL)
    /* Use the temperature sensor input to channel 0. */
    ADC_EnableTemperatureSensor(DEMO_ADC_BASE, true);
#endif /* FSL_FEATURE_ADC_HAS_NO_INSEL. */

    /* Enable channel DEMO_ADC_SAMPLE_CHANNEL_NUMBER's conversion in Sequence A. */
    adcConvSeqConfigStruct.channelMask = 
        (1U << DEMO_ADC_CURRENT_CHANNEL_NUMBER) | (1U << DEMO_ADC_VOLTAGE_CHANNEL_NUMBER); /* Includes channel DEMO_ADC_SAMPLE_CHANNEL_NUMBER. */
    adcConvSeqConfigStruct.triggerMask = 0U;
    adcConvSeqConfigStruct.triggerPolarity = kADC_TriggerPolarityPositiveEdge;
    adcConvSeqConfigStruct.enableSingleStep = false;
    adcConvSeqConfigStruct.enableSyncBypass = false;
    adcConvSeqConfigStruct.interruptMode = kADC_InterruptForEachSequence;
    ADC_SetConvSeqAConfig(DEMO_ADC_BASE, &adcConvSeqConfigStruct);
    ADC_EnableConvSeqA(DEMO_ADC_BASE, true); /* Enable the conversion sequence A. */
    /* Clear the result register. */
    ADC_DoSoftwareTriggerConvSeqA(DEMO_ADC_BASE);
    while (!ADC_GetChannelConversionResult(DEMO_ADC_BASE, DEMO_ADC_CURRENT_CHANNEL_NUMBER, &gAdcResultInfoStruct))
    {
    }
    while (!ADC_GetChannelConversionResult(DEMO_ADC_BASE, DEMO_ADC_VOLTAGE_CHANNEL_NUMBER, &gAdcResultInfoStruct))
    {
    }
    ADC_GetConvSeqAGlobalConversionResult(DEMO_ADC_BASE, &gAdcResultInfoStruct);
}

/*!
 * @brief Main function
 */
int main(void)
{
    /* Init board hardware. */

    BOARD_InitPins_Core1();

    //installTimeDelay();
    
    ADC_ClockPower_Configuration();
    if (!ADC_DoSelfCalibration(DEMO_ADC_BASE))
    {
        while(1);
    }
    
    ADC_Configuration();
    
    //ADC_EnableInterrupts(DEMO_ADC_BASE, kADC_ConvSeqAInterruptEnable);
    //NVIC_EnableIRQ(DEMO_ADC_IRQ_ID);

    /* Init Mailbox */
    MAILBOX_Init(MAILBOX);

    /* Enable mailbox interrupt */
    NVIC_EnableIRQ(MAILBOX_IRQn);

    /* send to core0 address of shared memory - static queue (ring buffer) */
    MAILBOX_SetValue(MAILBOX, kMAILBOX_CM4, (uint32_t)(&g_queue_buffer));

    uint32_t samp_since = 0;
    uint32_t adc_dat;
    uint32_t adc_samp_sum = 0;
    uint32_t adc_volt_sum = 0;

    ADC_EnableConvSeqABurstMode(DEMO_ADC_BASE, true);
    
    while (1)
    {
        //ADC_DoSoftwareTriggerConvSeqA(DEMO_ADC_BASE);

        do{
            adc_dat = DEMO_ADC_BASE->DAT[DEMO_ADC_CURRENT_CHANNEL_NUMBER];
        }
        while (0U == (ADC_DAT_DATAVALID_MASK & adc_dat));
        adc_samp_sum += (adc_dat & ADC_DAT_RESULT_MASK) >> ADC_DAT_RESULT_SHIFT;
        
        do{
            adc_dat = DEMO_ADC_BASE->DAT[DEMO_ADC_VOLTAGE_CHANNEL_NUMBER];
        }
        while (0U == (ADC_DAT_DATAVALID_MASK & adc_dat));
        adc_volt_sum += (adc_dat & ADC_DAT_RESULT_MASK) >> ADC_DAT_RESULT_SHIFT;
        
        samp_since ++;

        if (samp_since > QUEUE_ELEMENT_COUNT_TRIG)
        {
            MAILBOX_SetValue(MAILBOX, kMAILBOX_CM4, kProcessData);
            g_queue_buffer[0] = adc_samp_sum;
            g_queue_buffer[1] = adc_volt_sum;
            adc_samp_sum = 0;
            adc_volt_sum = 0;
            samp_since = 0;
        }
    }
}
