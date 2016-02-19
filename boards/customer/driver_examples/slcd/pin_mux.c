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

#include "fsl_port.h"

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Initialize all pins used in this example
 *
 */
void BOARD_InitPins(void)
{
    /* Ungate the port clock */
    CLOCK_EnableClock(kCLOCK_PortA);
    /* Affects PORTA_PCR1 register */
    PORT_SetPinMux(PORTA, 1u, kPORT_MuxAlt2);
    /* Affects PORTA_PCR2 register */
    PORT_SetPinMux(PORTA, 2u, kPORT_MuxAlt2);

	  /******************COM Setting  Wenxue ***********************************/
    /* Ungates the port clock */ /* LCD_P1/2/3/8 */
    CLOCK_EnableClock(kCLOCK_PortB);
    /* LCD_P1 */
    PORT_SetPinMux(PORTB, 1u, kPORT_PinDisabledOrAnalog);
    /* LCD_P2 */
    PORT_SetPinMux(PORTB, 2u, kPORT_PinDisabledOrAnalog);
    /* LCD_P3 */
    PORT_SetPinMux(PORTB, 3u, kPORT_PinDisabledOrAnalog);
    /* LCD_P8 */
    PORT_SetPinMux(PORTB, 8u, kPORT_PinDisabledOrAnalog);
	 /*****************************************************************/
	
  	/******************SEG Setting  Wenxue ***********************************/
		/*LCD_P0/9/10/11/12/13/14/15/20/21/22/23/24/25/26/27/28/29/30/31/40/41/42/43 */
		/* Ungates the port clock */
    CLOCK_EnableClock(kCLOCK_PortB);
    /* LCD_P0 */
    PORT_SetPinMux(PORTB, 0u, kPORT_PinDisabledOrAnalog);
    /* LCD_P9 */
    PORT_SetPinMux(PORTB, 9u, kPORT_PinDisabledOrAnalog);
    /* LCD_P10 */
    PORT_SetPinMux(PORTB, 10u, kPORT_PinDisabledOrAnalog);
    /* LCD_P11 */
    PORT_SetPinMux(PORTB, 11u, kPORT_PinDisabledOrAnalog);
		/* LCD_P12 */
    PORT_SetPinMux(PORTB, 16u, kPORT_PinDisabledOrAnalog);
			/* LCD_P13 */
    PORT_SetPinMux(PORTB, 17u, kPORT_PinDisabledOrAnalog);
			/* LCD_P14 */
    PORT_SetPinMux(PORTB, 18u, kPORT_PinDisabledOrAnalog);
			/* LCD_P15 */
    PORT_SetPinMux(PORTB, 19u, kPORT_PinDisabledOrAnalog);
		
		
    /* Ungates the port clock */
    CLOCK_EnableClock(kCLOCK_PortC);
    /* LCD_P20 */
    PORT_SetPinMux(PORTC, 0u, kPORT_PinDisabledOrAnalog);
    /* LCD_P21 */
    PORT_SetPinMux(PORTC, 2u, kPORT_PinDisabledOrAnalog);
    /* LCD_P22 */
    PORT_SetPinMux(PORTC, 2u, kPORT_PinDisabledOrAnalog);
    /* LCD_P23 */
    PORT_SetPinMux(PORTC, 3u, kPORT_PinDisabledOrAnalog);
		/* LCD_P24 */
    PORT_SetPinMux(PORTC, 4u, kPORT_PinDisabledOrAnalog);
    /* LCD_P25 */
    PORT_SetPinMux(PORTC, 5u, kPORT_PinDisabledOrAnalog);
    /* LCD_P26 */
    PORT_SetPinMux(PORTC, 6u, kPORT_PinDisabledOrAnalog);
    /* LCD_P27 */
    PORT_SetPinMux(PORTC, 7u, kPORT_PinDisabledOrAnalog);
		/* LCD_P28 */
    PORT_SetPinMux(PORTC, 8u, kPORT_PinDisabledOrAnalog);
    /* LCD_P29 */
    PORT_SetPinMux(PORTC, 9u, kPORT_PinDisabledOrAnalog);
    /* LCD_P30 */
    PORT_SetPinMux(PORTC, 10u, kPORT_PinDisabledOrAnalog);
    /* LCD_P31 */
    PORT_SetPinMux(PORTC, 11u, kPORT_PinDisabledOrAnalog);
		/* LCD_P5 */
    PORT_SetPinMux(PORTC, 21u, kPORT_PinDisabledOrAnalog); // VLL1
		/* LCD_P4 */
    PORT_SetPinMux(PORTC, 20u, kPORT_PinDisabledOrAnalog); // VLL2
			/* LCD_P39 */
    PORT_SetPinMux(PORTC, 23u, kPORT_PinDisabledOrAnalog); // VCAP1
		/* LCD_P6 */
    PORT_SetPinMux(PORTC, 22u, kPORT_PinDisabledOrAnalog); // VCAP2
		
    /* Ungates the port clock */
    CLOCK_EnableClock(kCLOCK_PortD);
    /* LCD_P40 */
    PORT_SetPinMux(PORTD, 0u, kPORT_PinDisabledOrAnalog);
    /* LCD_P41 */
    PORT_SetPinMux(PORTD, 1u, kPORT_PinDisabledOrAnalog);
    /* LCD_P42 */
    PORT_SetPinMux(PORTD, 2u, kPORT_PinDisabledOrAnalog);
    /* LCD_P43 */
    PORT_SetPinMux(PORTD, 3u, kPORT_PinDisabledOrAnalog);
}
