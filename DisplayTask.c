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
#include "Utils.h"
#include "DisplayTask.h"

#define X_TEMPERATURE   23
#define X_UVLEVEL       60
#define X_BARGRAPH      95

/* Global context for drawing */
extern tContext g_context;
extern tFont *g_psFontWDseg7bold24pt;
/* Handles created dynamically */
extern Mailbox_Handle g_mailboxDisplay;
extern SYSDATA g_sys;
extern SYSCONFIG g_cfg;

/* Static Module Globals */
static ScreenNum s_uScreenNum = SCREEN_UV;
static bool s_Screensave = false;

/* Static Function Prototypes */
static void ClearDisplay(void);
static void DrawAbout(void);
static void DrawUV(void);
static void DrawInfo(void);
static void PlotUVBarGraph(tRectangle rect, float percent);
void DrawInverseText(int x, int y, char* text, int len);

//*****************************************************************************
//
//*****************************************************************************

void ClearDisplay(void)
{
    tRectangle rect = {0, 0, SCREEN_WIDTH - 1, SCREEN_HEIGHT - 1};
    GrContextForegroundSetTranslated(&g_context, 0);
    GrContextBackgroundSetTranslated(&g_context, 0);
    GrRectFill(&g_context, &rect);
}

//*****************************************************************************
//
//*****************************************************************************

void DrawInverseText(int x, int y, char* text, int len)
{
    tRectangle rect;

    int width  = GrStringWidthGet(&g_context, text, len);
    int height = GrStringHeightGet(&g_context) - 1;

    GrContextForegroundSetTranslated(&g_context, 1);
    GrContextBackgroundSetTranslated(&g_context, 0);

    rect.i16XMin = x;
    rect.i16YMin = y;
    rect.i16XMax = x + width + 1;
    rect.i16YMax = y + height + 1;

    GrRectDraw(&g_context, &rect);

    GrContextForegroundSetTranslated(&g_context, 0);
    GrContextBackgroundSetTranslated(&g_context, 1);

    GrStringDraw(&g_context, text, -1, x+1, y+1, 1);

    GrContextForegroundSetTranslated(&g_context, 1);
    GrContextBackgroundSetTranslated(&g_context, 0);
}

//*****************************************************************************
//
//*****************************************************************************

void DrawAbout(void)
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

    GetHexStr(buf, &g_sys.ui8SerialNumber[0], 8);
    GrStringDraw(&g_context, buf, -1, 8, y, 0);
    y += height + spacing;

    GetHexStr(buf, &g_sys.ui8SerialNumber[8], 8);
    GrStringDraw(&g_context, buf, -1, 8, y, 0);
    y += height + spacing;

    GrFlush(&g_context);
}

//*****************************************************************************
//
//*****************************************************************************

void DrawInfo(void)
{
    int len;
    char buf[64];

    /* Set foreground pixel color on to 0x01 */
    GrContextForegroundSetTranslated(&g_context, 1);
    GrContextBackgroundSetTranslated(&g_context, 0);

    /* Setup font */
    uint32_t y;
    uint32_t height;
    uint32_t spacing = 4;

    /* Display the program version/revision */
    y = 4;
    GrContextFontSet(&g_context, g_psFontFixed6x8);
    height = GrStringHeightGet(&g_context);

    len = sprintf(buf, "PMX42 v%d.%02d", FIRMWARE_VER, FIRMWARE_REV);
    GrStringDrawCentered(&g_context, buf, len, 64, y, 0);
    y += height + (spacing * 2);

    /* Display the TCP/IP address if set */
    if (!strlen(g_sys.ipAddr))
        len = sprintf(buf, "(NO NETWORK OR DHCP)");
    else
        len = sprintf(buf, "IP %s", g_sys.ipAddr);
    GrStringDrawCentered(&g_context, buf, len, 64, y, 0);
    y += height + spacing;

    /* Format the current time/date and display it */

    len = sprintf(buf, "%d:%02d:%02d %d/%d/%d",
            g_sys.timeRTC.hour, g_sys.timeRTC.min, g_sys.timeRTC.sec,
            g_sys.timeRTC.month, g_sys.timeRTC.date, g_sys.timeRTC.year + 2000);
    GrStringDrawCentered(&g_context, buf, len, 64, y, 0);
    y += height + spacing;

    GrFlush(&g_context);
}

//*****************************************************************************
//
//*****************************************************************************

void DrawUV(void)
{
    int i;
    int len;
    uint32_t y;
    uint32_t x;
    uint32_t height;
    uint32_t width;
    uint32_t spacing;
    uint32_t adc;
    tRectangle rect;
    static char buf[128];
    //float v;
    float level;
    float power;
    //float step;
    float fullscale;
    float percentage;

    x = 0;
    y = 0;
    spacing = 2;

    /* Top line fixed system font in inverse */
    GrContextFontSet(&g_context, g_psFontFixed6x8);
    height = GrStringHeightGet(&g_context) - 1;

    /* Draw the channel heading in reverse */

    len = sprintf(buf, "CH");
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

    /* Draw the temperature heading */
    if (g_cfg.temp_format == TEMP_FAHRENHEIT)
        len = sprintf(buf, "degF");
    else if (g_cfg.temp_format == TEMP_KELVIN)
        len = sprintf(buf, "degK");
    else
        len = sprintf(buf, "degC");

    GrStringDraw(&g_context, buf, -1, X_TEMPERATURE, y+1, 0);

    /* Draw the UV level heading */
    len = sprintf(buf, "UV-C");
    width = GrStringWidthGet(&g_context, buf, len);
    GrStringDraw(&g_context, buf, -1, X_UVLEVEL, y+1, 0);

    /* Setup the font and get it's height */
    GrContextFontSet(&g_context,  g_psFontFixed6x8);
    height = GrStringHeightGet(&g_context);

    y += spacing + height + 5;

    for (i=0; i < g_sys.adcNumChannels; i++)
    {
        /* Draw the channel number */

        x = 3;
        sprintf(buf, "%d:", i+1);
        GrStringDraw(&g_context, buf, -1, x, y, 0);

        /* Draw the current temperature */

        x = X_TEMPERATURE;

        if (g_sys.rtdData[i] == RTD_ERROR)
            sprintf(buf, "ERR");
        else
        {
            if (g_cfg.temp_format == TEMP_FAHRENHEIT)
            {
                sprintf(buf, "%.1f", CELCIUS_TO_FAHRENHEIT(g_sys.rtdTempC[i]));
            }
            else if (g_cfg.temp_format == TEMP_KELVIN)
            {
                sprintf(buf, "%.1f", CELCIUS_TO_KELVIN(g_sys.rtdTempC[i]));
            }
            else
            {
                sprintf(buf, "%.1f", g_sys.rtdTempC[i]);
            }
        }

        GrStringDraw(&g_context, buf, -1, x, y, 0);

        /*
         * Display the UV power level
         */

        x = X_UVLEVEL;

        if (g_sys.adcData[i] == ADC_ERROR)
        {
            percentage = 0.0f;
            sprintf(buf, "ERR");
        }
        else
        {
            /* get the ADC sensor level */
            adc = g_sys.adcData[i];

            if (adc < 0xFF)
                adc = 0;

            level = (float)adc;

            /* The ADC vref is 4.096V */
            fullscale = (g_sys.adcID == AD7798_ID) ?  (float)AD7798_FULLSCALE : (float)AD7799_FULLSCALE;
#if 0
            /* Calculate the voltage per ADC step */
            step = ADC_VREF / fullscale;
            /* Calculate the actual voltage based on the ADC step value */
            v = step * level;
            /* sensor Vout = 0.71 x UV-C power in mW/cm2 */
            power = v / 0.71f;
#endif
            power = level / 6323.07f;

            if (power < 1.0f)
                sprintf(buf, "%.3f", power);
            else if (power > 10.0f)
                sprintf(buf, "%.1f", power);
            else
                sprintf(buf, "%.2f", power);

            //float percentage = (power / 7.0f) * 100.0f;
            percentage = (level / fullscale) * 100.0f;
        }

        GrStringDraw(&g_context, buf, -1, x, y, 0);

        /*
         * Display the power level bar graph
         */

        rect.i16XMin = X_BARGRAPH;
        rect.i16YMin = y;
        rect.i16XMax = SCREEN_WIDTH - 1;
        rect.i16YMax = (height - 2) + rect.i16YMin;

        PlotUVBarGraph(rect, percentage);

        y += height + spacing;
    }

    /* Draw the date and time at the bottom of the screen */

    y = 64 - (height + 1);

    len = sprintf(buf, "%d:%02d:%02d",
            g_sys.timeRTC.hour, g_sys.timeRTC.min, g_sys.timeRTC.sec);
    width = GrStringWidthGet(&g_context, buf, len);
    GrStringDraw(&g_context, buf, -1, 1, y, 0);
    //DrawInverseText(0, y, buf, len);

    len = sprintf(buf, "%02d/%02d/%d",
            g_sys.timeRTC.month, g_sys.timeRTC.date, g_sys.timeRTC.year + 2000);
    width = GrStringWidthGet(&g_context, buf, len);
    GrStringDraw(&g_context, buf, -1, SCREEN_WIDTH-(width+1), y, 0);
}

//*****************************************************************************
//
//*****************************************************************************

void PlotUVBarGraph(tRectangle rect, float percent)
{
    int32_t x;
    tRectangle rect2;

    GrContextForegroundSetTranslated(&g_context, 1);
    GrContextBackgroundSetTranslated(&g_context, 0);
    GrRectDraw(&g_context, &rect);

    if (percent <= 0.0f)
        return;

    rect2 = rect;

    rect2.i16XMin += 2;
    rect2.i16YMin += 2;
    rect2.i16XMax -= 2;
    rect2.i16YMax -= 2;

    int32_t x1 = rect2.i16XMin;
    int32_t x2 = rect2.i16XMax;

    float pscale = percent * 0.01f;

    x = (int16_t)((float)(x2 - x1) * pscale) + x1;

    if (x > x2)
        x = x2;

    if (x < x1)
        x = x1;

    rect2.i16XMax = (int16_t)x - 1;

    GrContextForegroundSetTranslated(&g_context, 1);
    GrContextBackgroundSetTranslated(&g_context, 0);

    GrRectFill(&g_context, &rect2);
}

//*****************************************************************************
// Display the current measurement screen data
//*****************************************************************************

void DrawScreen(uint32_t uScreenNum)
{
    ClearDisplay();

    switch(uScreenNum)
    {
    case SCREEN_ABOUT:
        DrawAbout();
        break;

    case SCREEN_INFO:
        DrawInfo();
        break;

    case SCREEN_UV:
        DrawUV();
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

    uint32_t ticks = 0;

    ClearDisplay();

    DrawAbout();

    Task_sleep(500);

    while (true)
    {
    	/* Wait for a message up to 1 second */
        if (!Mailbox_pend(g_mailboxDisplay, &msg, 250))
        {
            if (s_Screensave)
                continue;

    		/* Check for display sleep timeout */
    		if (++ticks >= ((g_cfg.screensave_time * 4) * 60))
    		{
    			ticks = 0;
    			s_Screensave = true;
                /* power down and put the display in sleep mode */
                FEMA128x64Sleep();
    		}
    		else
    		{
                DrawScreen(s_uScreenNum);
    		}
    		continue;
        }

        /* Reset the screen saver timeout */
		ticks = 0;

        /* Check if screen saver is active and wakeup if so */
		if (s_Screensave)
		{
		    /* Reset screen save state flag to off */
		    s_Screensave = false;

			/* Wakeup the screen and power it up */
			FEMA128x64Wake();
		}

		DrawScreen(s_uScreenNum);
    }
}

//*****************************************************************************
//
//*****************************************************************************

ScreenNum DisplaySetScreen(ScreenNum screen)
{
    DisplayMessage msg;

    if (screen > SCREEN_LAST)
        screen = SCREEN_LAST;

    /* Save the current screen number requested */
    s_uScreenNum = screen;

    /* Post a message to the display task to redraw and up the display */

    msg.dispCmd  = DISPLAY_DRAW;
    msg.dispArg1 = 0;
    msg.dispArg2 = 0;

    Mailbox_post(g_mailboxDisplay, &msg, BIOS_NO_WAIT);

    return screen;
}

//*****************************************************************************
//
//*****************************************************************************

Bool DisplayRefresh(void)
{
    Bool success;

    DisplayMessage msg;

    /* Post a message to the display task to redraw and up the display */
    msg.dispCmd  = DISPLAY_DRAW;
    msg.dispArg1 = 0;
    msg.dispArg2 = 0;

    success = Mailbox_post(g_mailboxDisplay, &msg, BIOS_NO_WAIT);

    return success;
}

//*****************************************************************************
//
//*****************************************************************************

ScreenNum DisplayGetScreen(void)
{
    return s_uScreenNum;
}

//*****************************************************************************
//
//*****************************************************************************

bool IsScreenSave(void)
{
    return s_Screensave;
}

// End-Of-File
