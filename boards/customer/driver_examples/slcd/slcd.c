/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
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

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "board.h"
#include "fsl_common.h"
#include "fsl_slcd.h"
#include "fsl_debug_console.h"

#include "pin_mux.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/


/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*!
 * @brief SLCD time delay.
 * @param count The timedelay counter number.
 */
static void SLCD_TimeDelay(uint32_t count);

/*******************************************************************************
 * Code
 ******************************************************************************/

static void SLCD_TimeDelay(uint32_t count)
{
    while (count--)
    {
        __NOP();
    }
}

int main(void)
{
    uint8_t waveForm = 0;
    slcd_config_t config;
    slcd_clock_config_t clkConfig =
    {

        kSLCD_AlternateClk1,   //  MCGIRCLK  wenxue

        kSLCD_AltClkDivFactor256, // MCGIRCLK =8M  8M/256=32.15k

        kSLCD_ClkPrescaler01,    // wenxue  LCLK=1
			
#if FSL_FEATURE_SLCD_HAS_FAST_FRAME_RATE
        false
#endif
    };

    /* Hardware initialize. */
    BOARD_InitPins();  // wenxue 要修改
		
    BOARD_BootClockRUN();
		
    BOARD_InitDebugConsole();
		
    /* Enable the MCGIRCLK */
    MCG->C1 |= MCG_C1_IRCLKEN_MASK;

    PRINTF("\r\nSLCD Example Starts.\r\n");

    /* SLCD get default configure. */
    /*
     * config.displayMode = kSLCD_NormalMode;
     * config.powerSupply = kSLCD_InternalVll3UseChargePump;
     * config.voltageTrim = kSLCD_RegulatedVolatgeTrim00;
     * config.lowPowerBehavior = kSLCD_EnabledInWaitStop;
     * config.frameFreqIntEnable = false;
     * config.faultConfig = NULL;
     */
    SLCD_GetDefaultConfig(&config);

    /* Verify and Complete the configuration structure. */
    config.clkConfig = &clkConfig;
    config.loadAdjust = kSLCD_HighLoadOrSlowestClkSrc;
    config.dutyCycle = kSLCD_1Div4DutyCycle;
		
		/*  全部LCD 引脚配置 */
    config.slcdLowPinEnabled =  0xFFF0FF7FU;        /* LCD_P14/15/20/24/26/27 相应位置为1   */
    config.slcdHighPinEnabled = 0x00000F80U;        /* LCD_P8/10/11/12/27/28+32= LCD_P40/42/43/44/59/60.  相应位置为1 */
		
		/* COM 口配置 */
    config.backPlaneLowPin =  0x0000010EU;    /* LCD_P1/2/3/8 */
    config.backPlaneHighPin = 0x00000000U;   /*  */
    config.faultConfig = NULL;
		
    /* SLCD Initialize. */
    SLCD_Init(LCD, &config);

   /* Set SLCD front plane phase to show: all segments on . */
    waveForm = (kSLCD_PhaseAActivate | kSLCD_PhaseBActivate | kSLCD_PhaseCActivate | kSLCD_PhaseDActivate);
    /* Set SLCD back plane phase. */  /*  COM 口 wenxue   */
    SLCD_SetBackPlanePhase(LCD, 1, kSLCD_PhaseAActivate); /* SLCD COM1 --- LCD_P1. */
    SLCD_SetBackPlanePhase(LCD, 2, kSLCD_PhaseBActivate); /* SLCD COM2 --- LCD_P2. */
    SLCD_SetBackPlanePhase(LCD, 3, kSLCD_PhaseCActivate); /* SLCD COM3 --- LCD_P3. */
    SLCD_SetBackPlanePhase(LCD, 8, kSLCD_PhaseDActivate); /* SLCD COM4 --- LCD_P8. */
		
		
		
    /* Set SLCD front plane phase to show. */    /*  Seg 口 wenxue   */
    SLCD_SetFrontPlaneSegments(LCD, 20, waveForm); /* SLCD P05 --- LCD_P20. */
    SLCD_SetFrontPlaneSegments(LCD, 24, waveForm); /* SLCD P06 --- LCD_P24. */
    SLCD_SetFrontPlaneSegments(LCD, 26, waveForm); /* SLCD P07 --- LCD_P26. */
    SLCD_SetFrontPlaneSegments(LCD, 27, waveForm); /* SLCD P08 --- LCD_P27. */
    SLCD_SetFrontPlaneSegments(LCD, 40, waveForm); /* SLCD P09 --- LCD_P40. */
    SLCD_SetFrontPlaneSegments(LCD, 42, waveForm); /* SLCD P10 --- LCD_P42. */
    SLCD_SetFrontPlaneSegments(LCD, 43, waveForm); /* SLCD P11 --- LCD_P43. */
    SLCD_SetFrontPlaneSegments(LCD, 44, waveForm); /* SLCD P12 --- LCD_P44. */

    /* Starts SLCD display. */
    SLCD_StartDisplay(LCD);
    PRINTF("\r\nSLCD Displays All Segments.\r\n");
    SLCD_TimeDelay(0xFFFFFFU);

    PRINTF("\r\nSLCD Starts Blink Mode.\r\n");

    /* Blink mode Display. */
    SLCD_StartBlinkMode(LCD, kSLCD_BlankDisplayBlink, kSLCD_BlinkRate01);
    SLCD_TimeDelay(0xFFFFFFU);

    PRINTF("\r\nSLCD Stops Blink Mode.\r\n");
    /* Stops SLCD blink display mode. */
    SLCD_StopBlinkMode(LCD);
    SLCD_TimeDelay(0xFFFFFFU);

    PRINTF("\r\nSLCD Stops Display.\r\n");
    /* Stops SLCD display. */
    SLCD_StopDisplay(LCD);

    PRINTF("\r\nSLCD Example Ends.\r\n");

    while (1)
    {
    }
}
