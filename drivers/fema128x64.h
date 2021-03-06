//*****************************************************************************
//
// cfal96x64x16.h - Prototypes for the Crystalfontz CFAL9664-F-B1 OLED display
//                                     with an SSD1332 controller.
//
// Copyright (c) 2011-2013 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 1.0 of the EK-LM4F232 Firmware Package.
//
//*****************************************************************************

#ifndef __FEMA128X66_H__
#define __FEMA128X64_H__

#define SCREEN_WIDTH	128
#define SCREEN_HEIGHT	64

//*****************************************************************************
//
// Prototypes for the globals exported by this driver.
//
//*****************************************************************************

void FEMA128x64Init(void);

void FEMA128x64Wake(void);
void FEMA128x64Sleep(void);

int SSD1309GetScreenBufferSize(void);
unsigned char* SSD1309GetScreenBuffer(void);

extern tDisplay g_FEMA128x64;

#endif // __FEMA128X64_H__
