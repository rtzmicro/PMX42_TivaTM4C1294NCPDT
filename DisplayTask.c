/*
 * Copyright (c) 2014, Texas Instruments Incorporated
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
 *    ======== tcpEcho.c ========
 *    Contains BSD sockets code.
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/System.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Gate.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Mailbox.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>
//#include <ti/sysbios/fatfs/ff.h>

/* TI-RTOS Driver files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SDSPI.h>
#include <ti/drivers/I2C.h>

/* NDK BSD support */
#include <sys/socket.h>

#include <file.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <stdbool.h>

/* Graphiclib Header file */
#include <grlib/grlib.h>
#include "drivers/fema128x64.h"

/* PMX42 Board Header file */
#include "Board.h"
#include "PMX42.h"
#include "AD7793.h"
#include "DisplayTask.h"

/* Global context for drawing */
extern tContext g_context;

extern tFont *g_psFontWDseg7bold24pt;

/* Handles created dynamically */
extern Mailbox_Handle g_mailboxDisplay;

extern SYSDATA g_sysData;

/* Static Module Globals */
uint32_t s_uScreenNum = 0;
uint32_t s_uSampleCount = 0;
uint32_t s_adc1 = 0;
uint32_t s_adc2 = 0;

bool s_isFahrenheit = false;

/* Static Function Prototypes */
static int GetHexStr(char* textbuf, uint8_t* databuf, int datalen);

//*****************************************************************************
// Format a data buffer into an ascii hex string.
//*****************************************************************************

#if 1
int GetHexStr(char* textbuf, uint8_t* databuf, int datalen)
{
    char *p = textbuf;
    uint8_t *d;
    uint32_t i;
    int32_t l;

    const uint32_t wordSize = 4;

    /* Null output text buffer initially */
    *textbuf = 0;

    /* Make sure buffer length is not zero */
    if (!datalen)
        return 0;

    /* Read data bytes in reverse order so we print most significant byte first */
    d = databuf + (datalen-1);

    for (i=0; i < datalen; i++)
    {
        l = sprintf(p, "%02X", *d--);
        p += l;

        if (((i % wordSize) == (wordSize-1)) && (i != (datalen-1)))
        {
            l = sprintf(p, "-");
            p += l;
        }
    }

    return strlen(textbuf);
}
#else
int GetHexStr(char* textbuf, uint8_t* databuf, int datalen)
{
    char fmt[8];
    uint32_t i;
    int32_t l;

    const uint32_t wordSize = 4;

    *textbuf = 0;
    strcpy(fmt, "%02X");

    for (i=0; i < datalen; i++)
    {
        l = sprintf(textbuf, fmt, *databuf++);
        textbuf += l;

        if (((i % wordSize) == (wordSize-1)) && (i != (datalen-1)))
        {
            l = sprintf(textbuf, "-");
            textbuf += l;
        }
    }

    return strlen(textbuf);
}
#endif

//*****************************************************************************
//
//*****************************************************************************

void ClearDisplay()
{
    tRectangle rect = {0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1};
    GrContextForegroundSetTranslated(&g_context, 0);
    GrContextBackgroundSetTranslated(&g_context, 0);
    GrRectFill(&g_context, &rect);
}

//*****************************************************************************
//
//*****************************************************************************

void DisplayWelcome()
{
	char buf[64];

	/* Set foreground pixel color on to 0x01 */
	GrContextForegroundSetTranslated(&g_context, 1);
	GrContextBackgroundSetTranslated(&g_context, 0);

    tRectangle rect = {0, 0, SCREEN_WIDTH-1, SCREEN_HEIGHT-1};
    GrRectDraw(&g_context, &rect);

    /* Setup font */

	uint32_t y;
	uint32_t height;
	uint32_t spacing = 2;

    /* Display the program version/revision */
    y = 4;
	GrContextFontSet(&g_context, g_psFontCm28b);
    height = GrStringHeightGet(&g_context);
    GrStringDraw(&g_context, "PMX42", -1, 21, y, 0);
    y += height;

    /* Switch to fixed system font */
    GrContextFontSet(&g_context, g_psFontFixed6x8);
    height = GrStringHeightGet(&g_context);

    sprintf(buf, "Firmware v%d.%02d", FIRMWARE_VER, FIRMWARE_REV);
    GrStringDraw(&g_context, buf, -1, 25, y, 0);
    y += height + spacing + 2;

    /* Get the serial number string and display it */

    GetHexStr(buf, &g_sysData.ui8SerialNumber[0], 8);
    GrStringDraw(&g_context, buf, -1, 8, y, 0);
    y += height + spacing;

    GetHexStr(buf, &g_sysData.ui8SerialNumber[8], 8);
    GrStringDraw(&g_context, buf, -1, 8, y, 0);
    y += height + spacing;

    GrFlush(&g_context);
}

//*****************************************************************************
// Display the curreent measurement screen data
//*****************************************************************************

#define LAST_SCREEN					1

void DrawScreen(uint32_t uScreenNum)
{
	char tempScaleChar;
	int len;
	uint32_t y = 0;
	uint32_t x = 0;
	uint32_t height;
	uint32_t width;
	uint32_t spacing = 0;
	tRectangle rect;
	UInt key;
	static char buf[128];

	ClearDisplay();

	/* Set foreground pixel color on to 0x01 */
	GrContextForegroundSetTranslated(&g_context, 1);
	GrContextBackgroundSetTranslated(&g_context, 0);

	/* RTD standards
	 *
	 * There are two standards for platinum RTDs: the European standard
	 * (also known as the DIN or IEC standard) and the American standard.
	 * The European standard, also known as the DIN or IEC standard, is
	 * considered the world-wide standard for platinum RTDs. This standard,
	 * DIN/IEC 60751 (or simply IEC751), requires the RTD to have an electrical
	 * resistance of 100.00 ohm at 0°C and a temperature coefficient of
	 * resistance (TCR) of 0.00385 ohm/ohm/°C between 0 and 100°C.
	 *
	 * There are two resistance tolerances specified in DIN/IEC751:
	 *
	 * Class A = ±(0.15 + 0.002*t)°C or 100.00 ±0.06 ohm at 0ºC
	 * Class B = ±(0.3 + 0.005*t)°C or 100.00 ±0.12 ohm at 0ºC
     *
	 * Two resistance tolerances used in industry are:
	 *
	 * 1/3 DIN = ±1/3* (0.3 + 0.005*t)°C or 100.00 ±0.10 ohm at 0ºC
	 * 1/10 DIN = ±1/10* (0.3 + 0.005*t)°C or 100.00 ±0.03 ohm at 0ºC
     *
	 * The combination of resistance tolerance and temperature coefficient
	 * define the resistance vs. temperature characteristics for the RTD
	 * sensor. The larger the element tolerance, the more the sensor will
	 * deviate from a generalized curve, and the more variation there will
	 * be from sensor to sensor (interchangeability). This is important to
	 * users who need to change or replace sensors and want to minimize
	 * interchangeability errors.
	 */

	/* Read the ADC's for the RDT temperature sensors */
	//s_adc1 = AD7793_SingleConversion(Board_SLOT1_AD7793_CS1);
	//s_adc2 = AD7793_SingleConversion(Board_SLOT1_AD7793_CS2);

	s_adc1 = AD7793_ContinuousReadAvg(Board_SLOT2_AD7793_CS1, 12);
	s_adc2 = AD7793_ContinuousReadAvg(Board_SLOT2_AD7793_CS2, 12);

	/* Convert ADC value to Celcius */
	float temp1 = AD7793_temperature(s_adc1);
	float temp2 = AD7793_temperature(s_adc2);

	/* Convert Celcius to Fahrenheit */
	key = Hwi_disable();

	if (s_isFahrenheit)
	{
		tempScaleChar = 'F';
		temp1 = CELCIUS_TO_FAHRENHEIT(temp1);
		temp2 = CELCIUS_TO_FAHRENHEIT(temp2);
	}
	else
	{
		tempScaleChar = 'C';
	}

	Hwi_restore(key);

	//sprintf(buf, "%f", temp1);
	//System_printf("temp1 %s\n", buf);
	//System_flush();

	++s_uSampleCount;

    switch(uScreenNum)
    {
    	/* 2 Channel Temp Measurement Data */
		case 0:
			y = 0;
			x = 0;
			spacing = 2;

			tempScaleChar = (s_isFahrenheit) ? 'F' : 'C';

		    /* Top line fixed system font in inverse */
		    GrContextFontSet(&g_context, g_psFontFixed6x8);
		    height = GrStringHeightGet(&g_context);

		    len = sprintf(buf, "CNT");
			width = GrStringWidthGet(&g_context, buf, len);

			GrContextForegroundSetTranslated(&g_context, 1);
			GrContextBackgroundSetTranslated(&g_context, 0);

			rect.i16XMin = x;
			rect.i16YMin = y;
			rect.i16XMax = width + 1;
			rect.i16YMax = height + 1;
		    GrRectDraw(&g_context, &rect);

			GrContextForegroundSetTranslated(&g_context, 0);
			GrContextBackgroundSetTranslated(&g_context, 1);

			GrStringDraw(&g_context, buf, -1, x+1, y+1, 1);

			GrContextForegroundSetTranslated(&g_context, 1);
			GrContextBackgroundSetTranslated(&g_context, 0);

			sprintf(buf, "%u", s_uSampleCount);
			GrStringDraw(&g_context, buf, -1, x+width+4+1, y+1, 0);

			x = 2;
			y += spacing + height + 5;

			/* Setup the font and get it's height */
			GrContextFontSet(&g_context,  g_psFontCmss16b);
		    height = GrStringHeightGet(&g_context);

			//sprintf(buf, "CH-1: %-6.6X", s_adc1);
			sprintf(buf, "CH-1: %.1f %c", temp1, tempScaleChar);
			GrStringDraw(&g_context, buf, -1, x, y, 0);

			y += height + spacing;

			//sprintf(buf, "CH-2: %-6.6X", s_adc2);
			sprintf(buf, "CH-2: %.1f %c", temp2, tempScaleChar);
			GrStringDraw(&g_context, buf, -1, x, y, 0);

			break;

		/* 1 Big Number Centered */
		case 1:
		    /* Top line fixed system font in inverse */
		    GrContextFontSet(&g_context, g_psFontFixed6x8);
		    height = GrStringHeightGet(&g_context);

			GrContextForegroundSetTranslated(&g_context, 0);
			GrContextBackgroundSetTranslated(&g_context, 1);

		    len = sprintf(buf, "CH-1");
			width = GrStringWidthGet(&g_context, buf, len);
			GrStringDraw(&g_context, buf, -1, 0, y, 1);

			GrContextForegroundSetTranslated(&g_context, 1);
			GrContextBackgroundSetTranslated(&g_context, 0);

		    sprintf(buf, "%d.%02dV", rand() % 999 + 1, rand() % 10 + 1);
			GrStringDraw(&g_context, buf, -1, width+4, y, 0);

			y += height + spacing;

			/* Now draw the big digits centered */
			GrContextFontSet(&g_context, g_psFontWDseg7bold24pt);
			//GrContextFontSet(&g_context, g_psFontCm30b);
		    height = GrStringHeightGet(&g_context);
			len = sprintf(buf, "%d", rand() % 999 + 1);
			//GrStringDraw(&g_context, buf, -1, 10, y, 0);
			GrStringDrawCentered(&g_context, buf, len, SCREEN_WIDTH/2, SCREEN_HEIGHT/2, FALSE);

			y += height + spacing;
			break;

		default:
			break;
    }

    GrFlush(&g_context);
}


//*****************************************************************************
// OLED Display Drawing task
//
// It is pending for the message either from console task or from button ISR.
// Once the messages received, it draws to the screen based on information
//  contained in the message.
//
//*****************************************************************************

Void DisplayTaskFxn(UArg arg0, UArg arg1)
{
	//static char lineBuf[65];

    DisplayMessage msg;
    //unsigned int i = 0;
    //unsigned int fontHeight;

    //fontHeight = GrStringHeightGet(&g_context);
    bool screensave = 0;
    uint32_t secs = 0;

    ClearDisplay();

    DisplayWelcome();

    while (true)
    {
    	/* Wait for a message up to 1 second */
        if (!Mailbox_pend(g_mailboxDisplay, &msg, 1000))
        {
    		/* Check for display sleep timeout */
    		if (++secs >= 60 * 5)
    		{
    			/* power down and put the display in sleep mode */
    			FEMA128x64Sleep();
    			secs = 0;
    			screensave = 1;
    		}

    		if (!screensave)
    			DrawScreen(s_uScreenNum);

    		continue;
        }

        /* Reset the screensaver timeout */
		secs = 0;

        /* Check if screen saver is active and wakeup if so */
		if (screensave)
		{
			screensave = 0;
			/* Wakeup the screen and power it up */
			FEMA128x64Wake();
			/* Discard button press and force a screen refresh */
        	msg.dispCommand = SCREEN_REFRESH;
		}

		//System_printf("cmd=%d\n", msg.dispCommand);
		//System_flush();

		switch(msg.dispCommand)
		{
        case SCREEN_SET:
        	if (msg.dispArg1 < LAST_SCREEN)
        		s_uScreenNum = msg.dispArg1;
        	DrawScreen(s_uScreenNum);
            break;

        case SCREEN_NEXT:
        	++s_uScreenNum;
        	if (s_uScreenNum > LAST_SCREEN)
        		s_uScreenNum = 0;
        	DrawScreen(s_uScreenNum);
            break;

        case SCREEN_PREV:
        	if (s_uScreenNum)
        		--s_uScreenNum;
        	else if (!s_uScreenNum)
        		s_uScreenNum = LAST_SCREEN;
        	DrawScreen(s_uScreenNum);
            break;

        case SCREEN_REFRESH:
        default:
        	DrawScreen(s_uScreenNum);
            break;
        }
    }
}

// End-Of-File
