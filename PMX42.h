/*
 * PMX42.h : created 5/18/2015
 *
 * Copyright (C) 2015, Robert E. Starr. ALL RIGHTS RESERVED.
 *
 * THIS MATERIAL CONTAINS  CONFIDENTIAL, PROPRIETARY AND TRADE
 * SECRET INFORMATION. NO DISCLOSURE OR USE OF ANY
 * PORTIONS OF THIS MATERIAL MAY BE MADE WITHOUT THE EXPRESS
 * WRITTEN CONSENT OF THE AUTHOR.
 */

#ifndef __PMX42_H
#define __PMX42_H

#include "AD7799.h"

/* Helper Macros */
#define ADC_TO_CELCIUS(c)           ( 147.5f - ((75.0f * VREF * (float)c) / 4096.0f) )
#define CELCIUS_TO_FAHRENHEIT(c)    ( (float)c * 1.8f + 32.0f )

//*****************************************************************************
// CONSTANTS AND CONFIGURATION
//*****************************************************************************

/* VERSION INFO - The min build specifies the minimum build required
 * that does NOT force a default reset of all the config parameters
 * at run time. For instance, if the config loads build 5 and the minimum
 * is set to 3, then it will reset config for anything less than build 3.
 * Likewise, versions 3 or higher would load and use the config values from
 * eprom as normal. This provides a means to force run time config defaults
 * to be reset or not.
 */
#define FIRMWARE_VER        2           /* firmware version */
#define FIRMWARE_REV        34          /* firmware revision */
#define FIRMWARE_BUILD      1           /* firmware build number */

#define MAGIC               0xCEB0FACE  /* magic number for EEPROM data */
#define MAKEREV(v, r)       ((v << 16) | (r & 0xFFFF))

//*****************************************************************************
// 128-BIT GUID Structure
//*****************************************************************************

typedef struct _GUID128 {
    union {
        uint8_t     addr[16];
    } b8;
    union {
        uint16_t    addr[8];
    } w16;
    union {
        uint32_t    addr[4];
    } w32;
} GUID128;

//*****************************************************************************
//GLOBAL RUN-TIME DATA
//*****************************************************************************

typedef struct _SYSDATA
{
    uint8_t         ui8SerialNumber[16];    /* 128-bit serial number      */
    uint8_t         ui8MAC[6];              /* 48-bit MAC from EPROM      */
    char            ipAddr[32];             /* IP address from DHCP       */
    /* global runtime data */
    SPI_Handle      spi12;                  /* SPI handle for slots 1 & 2 */
    AD7799_Handle   AD7799HandleSlot1;
    AD7799_Handle   AD7799HandleSlot2;

    SPI_Handle      spi34;                  /* SPI handle for slots 3 & 4 */

} SYSDATA;

//*****************************************************************************
// SYSTEM CONFIG PARAMETERS STORED IN EPROM
//*****************************************************************************

typedef struct _SYSPARMS
{
    uint32_t magic;
    uint32_t version;
    uint32_t build;
    /*** GLOBAL PARAMETERS ***/
    long debug;                     	/* debug level */
} SYSCONFIG;

//*****************************************************************************
// Meter Command Message Structure
//*****************************************************************************

typedef enum CommandType{
    BUTTONPRESS,
} CommandType;

typedef struct CommandMessage{
    CommandType		command;
    uint32_t 		ui32Data;
} CommandMessage;

int main(void);
Void CommandTaskFxn(UArg arg0, UArg arg1);
int ReadGUIDS(uint8_t ui8SerialNumber[16], uint8_t ui8MAC[6]);

#endif /* __PMX42_H */
