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

#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_mailbox.h"
#include "static_queue.h"
#include "low_power.h"
#include "fsl_power.h"
#include "fsl_utick.h"
#include <stdio.h>

#include "fsl_common.h"
#include "pin_mux.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define APP_EXCLUDE_FROM_DEEPSLEEP                                                                           \
    (SYSCON_PDRUNCFG_PDEN_SRAM0A_MASK | SYSCON_PDRUNCFG_PDEN_SRAM0B_MASK | SYSCON_PDRUNCFG_PDEN_SRAM1_MASK | \
     SYSCON_PDRUNCFG_PDEN_WDT_OSC_MASK)

#define APP_DISABLE_PINS   \
    IOCON->PIO[0][12] = 0; \
    IOCON->PIO[0][13] = 0; \
    IOCON->PIO[0][14] = 0; \
    IOCON->PIO[0][18] = 0; \
    IOCON->PIO[0][19] = 0; \
    IOCON->PIO[0][20] = 0; \
    IOCON->PIO[0][27] = 0; \
    IOCON->PIO[0][28] = 0; \
    IOCON->PIO[0][30] = 0; \
    IOCON->PIO[1][3] = 0;  \
    IOCON->PIO[1][4] = 0;

/* Address of RAM, where the image for core1 should be copied */
#define CORE1_BOOT_ADDRESS (void *)0x02010000

#if defined(__CC_ARM)
extern uint32_t Image$$CORE1_REGION$$Base;
extern uint32_t Image$$CORE1_REGION$$Length;
#define CORE1_IMAGE_START &Image$$CORE1_REGION$$Base
#elif defined(__ICCARM__)
extern unsigned char core1_image_start[];
#define CORE1_IMAGE_START core1_image_start
#elif defined(__GNUC__)
extern const char m0_image_start[];
extern const char* m0_image_end;
extern int m0_image_size;
#define CORE1_IMAGE_START ((void *)m0_image_start)
#define CORE1_IMAGE_SIZE ((void *)m0_image_size)
#endif

#define QUEUE_ELEMENT_COUNT_TRIG 524288
#define UTICK_TIME 10000
#define CORE_CLK_FREQ (CLOCK_GetFreq(kCLOCK_MainClk)/4)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

#ifdef CORE1_IMAGE_COPY_TO_RAM
uint32_t get_core1_image_size(void);
#endif

/*******************************************************************************
 * Variables
 ******************************************************************************/
uint32_t *volatile g_dataBuff = NULL;
bool volatile g_goToDeepSleep = false;
bool volatile g_readyToProcess = false;

/*******************************************************************************
 * Code
 ******************************************************************************/

#ifdef CORE1_IMAGE_COPY_TO_RAM
uint32_t get_core1_image_size()
{
    uint32_t core1_image_size;
#if defined(__CC_ARM)
    core1_image_size = (uint32_t)&Image$$CORE1_REGION$$Length;
#elif defined(__ICCARM__)
#pragma section = "__sec_core"
    core1_image_size = (uint32_t)__section_end("__sec_core") - (uint32_t)&core1_image_start;
#elif defined(__GNUC__)
    core1_image_size = (uint32_t)m0_image_size;
#endif
    return core1_image_size;
}
#endif


/*!
 * @brief Time delay
 * Function waits desired time with accuracy 1us.
 */
void timeDelay_us(uint32_t delay)
{
    delay++;
    /* Wait desired time */
    while (delay > 0)
    {
        if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
        {
            delay--;
        }
    }
}

void timeDelay(uint32_t miliseconds)
{
    timeDelay_us(miliseconds * 1000);
}

void timeDelay16(uint16_t miliseconds)
{
    timeDelay_us(miliseconds * 1000);
}

void installTimeDelay()
{
    /* Disable SysTick timer */
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    /* Initialize Reload value to 1us */
    SysTick->LOAD = CORE_CLK_FREQ / 1000000;
    /* Set clock source to processor clock */
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;
    /* Enable SysTick timer */
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

void utick_cb()
{

}

void processData()
{
    uint32_t total = g_dataBuff[0];
    float avg = (float)total / (float)QUEUE_ELEMENT_COUNT_TRIG;
    avg = avg /4096 * 3000 / 2500;
    PRINTF("Raw: %d, Average: %.4f A\r\n", total, avg);
}

void MAILBOX_IRQHandler()
{
    if (g_dataBuff == NULL)
    {
        g_dataBuff = (uint32_t *)MAILBOX_GetValue(MAILBOX, kMAILBOX_CM4);
    }
    else
    {
        core_cmd_t cmd = (core_cmd_t)MAILBOX_GetValue(MAILBOX, kMAILBOX_CM4);
        if (cmd == kGoToDeepSleep)
        {
            g_goToDeepSleep = true;
        }
        else if (cmd == kProcessData)
        {
            g_readyToProcess = true;
        }
    }
    MAILBOX_ClearValueBits(MAILBOX, kMAILBOX_CM4, 0xffffffff);
    /* Add for ARM errata 838869, affects Cortex-M4, Cortex-M4F Store immediate overlapping
      exception return operation might vector to incorrect interrupt */
#if defined __CORTEX_M && (__CORTEX_M == 4U)
    __DSB();
#endif
}

/*!
 * @brief Main function
 */
int main(void)
{
    /* Init board hardware.*/
    /* attach 12 MHz clock to USART0 (debug console) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);

    BOARD_InitPins_Core0();
    //BOARD_BootClockIRC12M();
    BOARD_BootClockPLL96M();
    BOARD_InitDebugConsole();

    /* TURN OFF FLASH Clock (only needed for FLASH programming, will be turned on by ROM API) */
    CLOCK_DisableClock(kCLOCK_Flash);

    PRINTF("CORE0 is running\r\n");

    /* Minimize power consumption by switching several pins to analog mode */
    //APP_DISABLE_PINS;

    /* Init Mailbox */
    MAILBOX_Init(MAILBOX);

    /* Enable mailbox interrupt */
    NVIC_EnableIRQ(MAILBOX_IRQn);
    
    installTimeDelay();
    
    /* Initialize UTICK */
    UTICK_Init(UTICK0);

    /* Set the UTICK timer to wake up the device from reduced power mode */
    UTICK_SetTick(UTICK0, kUTICK_Repeat, UTICK_TIME, utick_cb);

#ifdef CORE1_IMAGE_COPY_TO_RAM
    /* Calculate size of the image */
    uint32_t core1_image_size;
    core1_image_size = get_core1_image_size();
    PRINTF("Copy CORE1 image to address: 0x%x, size: %d\r\n", CORE1_BOOT_ADDRESS, core1_image_size);

    /* Copy application from FLASH to RAM */
    memcpy(CORE1_BOOT_ADDRESS, (void *)CORE1_IMAGE_START, core1_image_size);
#endif

    PRINTF("Starting CORE1\r\n");

    RESET_SlaveCoreReset(*(uint32_t*)(((uint8_t*)(CORE1_BOOT_ADDRESS))+4), *(uint32_t*)(CORE1_BOOT_ADDRESS));

    /* Wait for initializing data buffer from core1 */
    while (g_dataBuff == NULL)
    {
    }

    PRINTF("CORE1 is running.\r\n");

    while (1)
    {
        while(!g_readyToProcess);
        processData();
        g_readyToProcess = false;
        //timeDelay(1000);
    }
}
