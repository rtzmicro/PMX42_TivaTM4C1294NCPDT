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

#ifndef __BOARD_H
#define __BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "PMX42_TM4C1294NCPDT.h"

#define Board_initEMAC              PMX42_initEMAC
#define Board_initGeneral           PMX42_initGeneral
#define Board_initGPIO              PMX42_initGPIO
#define Board_initI2C               PMX42_initI2C
#define Board_initPWM               PMX42_initPWM
#define Board_initSDSPI             PMX42_initSDSPI
#define Board_initSPI               PMX42_initSPI
#define Board_initUART              PMX42_initUART
#define Board_initUSB               PMX42_initUSB
#define Board_initUSBMSCHFatFs      PMX42_initUSBMSCHFatFs
#define Board_initWatchdog          PMX42_initWatchdog
#define Board_initWiFi              PMX42_initWiFi

/* On board switches and LED's */
#define Board_LED_ON                PMX42_LED_ON
#define Board_LED_OFF               PMX42_LED_OFF
#define Board_LED_ACT               PMX42_LED_ACT
#define Board_LED_ALM               PMX42_LED_ALM
#define Board_BTN_SW1               PMX42_BTN_SW1
#define Board_BTN_SW2               PMX42_BTN_SW2
#define Board_BTN_SW3               PMX42_BTN_SW3
#define Board_BTN_SW4               PMX42_BTN_SW4
#define Board_BTN_SW5               PMX42_BTN_SW5
#define Board_BTN_SW6               PMX42_BTN_SW6

/* Each slot has a dedicated I2C port */
#define Board_I2C_SLOT1             PMX42_I2C0
#define Board_I2C_SLOT2             PMX42_I2C1
#define Board_I2C_SLOT3             PMX42_I2C2
#define Board_I2C_SLOT4             PMX42_I2C3

/* I2C3 is shared with the AT24MAC402 serial number/MAC
 * and the MCP79410T RTC parts.
 */
#define Board_I2C_AT24MAC402        PMX42_I2C3
#define Board_I2C_MCP79410T         PMX42_I2C3

#define Board_PWM0                  PMX42_PWM0
#define Board_PWM1                  PMX42_PWM0

/* SPI-1 dedicated to SD drive */
#define Board_SDSPI1                PMX42_SDSPI1

/* Slots 1 and 2 are driven by SPI-2
 * Slots 3 and 4 are driven by SPI-3
 */
#define Board_SPI2                  PMX42_SPI2
#define Board_SPI3                  PMX42_SPI3

#define Board_USBHOST               PMX42_USBHOST
#define Board_USBDEVICE             PMX42_USBDEVICE

#define Board_UART0                 PMX42_UART0
#define Board_UART3                 PMX42_UART3
#define Board_UART5                 PMX42_UART5
#define Board_UART7                 PMX42_UART7

/**** I/O Option Card Specific Port Defines ****/
#define Board_SLOT1_IRQ             PMX42_PL2_SLOT1_IRQ
#define Board_SLOT2_IRQ             PMX42_PL3_SLOT2_IRQ
#define Board_SLOT3_IRQ             PMX42_PL4_SLOT3_IRQ
#define Board_SLOT4_IRQ             PMX42_PL5_SLOT4_IRQ

#define Board_SLOT1_SS              PMX42_PD2_SLOT1_SS
#define Board_SLOT2_SS              PMX42_PM2_SLOT2_SS
#define Board_SLOT3_SS              PMX42_PQ1_SLOT3_SS
#define Board_SLOT4_SS              PMX42_PM3_SLOT4_SS

#define Board_SLOT1_AD7793_CS1		PMX42_PD2_SLOT1_SS
#define Board_SLOT1_AD7793_CS2		PMX42_PM4_T4CCP0
#define Board_SLOT2_AD7793_CS1      PMX42_PM2_SLOT2_SS
#define Board_SLOT2_AD7793_CS2      PMX42_PF1_M0PWM1

#define Board_SLOT2_RELAY1			PMX42_PF1_M0PWM1
#define Board_SLOT2_RELAY2			PMX42_PF2_M0PWM2

#define Board_SLOT3_UART_RS422		PMX42_UART5
#define Board_SLOT3_UART_RS232		PMX42_UART7

#define Board_SLOT3_GPIO_DE			Board_SLOT3_IRQ
#define Board_SLOT3_GPIO_RE			Board_SLOT3_SS

#define Board_WATCHDOG0             PMX42_WATCHDOG0

#define Board_WIFI                  PMX42_WIFI

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H */
