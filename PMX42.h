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

#include "MCP79410.h"       /* RTC time/date */
#include "AD7799.h"         /* ADC converter */
#include "MAX31865.h"       /* RTD converter */

//*****************************************************************************
// CONSTANTS AND CONFIGURATION
//*****************************************************************************

/* There's only one SD drive in the system, so this is always zero */
#define SD_DRIVE_NUM        0

/* This enables the DIVSCLK output pin on PQ4 and generates a clock signal
 * from the main cpu clock divided by 'div' parameter. A value of 100 gives
 * a clock of 1.2 Mhz.
 */
#define DIV_CLOCK_ENABLED   0

/* VERSION INFO - The min build specifies the minimum build required
 * that does NOT force a default reset of all the config parameters
 * at run time. For instance, if the config loads build 5 and the minimum
 * is set to 3, then it will reset config for anything less than build 3.
 * Likewise, versions 3 or higher would load and use the config values from
 * eprom as normal. This provides a means to force run time config defaults
 * to be reset or not.
 */
#define FIRMWARE_VER        1           /* firmware version */
#define FIRMWARE_REV        6           /* firmware revision */
#define FIRMWARE_BUILD      1           /* firmware build number */
#define FIRMWARE_MIN_BUILD  1           /* min build req'd to force reset */

#if (FIRMWARE_MIN_BUILD > FIRMWARE_BUILD)
#error "PMX42 build option FIRMWARE_MIN_BUILD set incorrectly"
#endif

#define MAGIC               0xCEB0FACE  /* magic number for EEPROM data */
#define MAKEREV(v, r)       ((v << 16) | (r & 0xFFFF))

//*****************************************************************************
// 53105 ADC Card Constants and Data Structures
//*****************************************************************************

/* This defines the number of ADC cards and channels in the system */
#define ADC_NUM_CARDS               1
#define ADC_CHANNELS_PER_CARD       2
#define ADC_NUM_CHANNELS            (ADC_NUM_CARDS * ADC_CHANNELS_PER_CARD)
#define ADC_MAX_CHANNELS            (ADC_CHANNELS_PER_CARD * 2)

#define ADC_VREF                    4.096f
#define ADC_ERROR                   0xFFFFFFFF

/* Each ADC card has one converter with a chip select
 * and provides two ADC channels per converter.
 */
typedef struct _ADC_CONVERTER {
    AD7799_Handle   handle;
    uint32_t        chipsel;
} ADC_CONVERTER;

//*****************************************************************************
// 53107 RTD Card Constants and Data Structures
//*****************************************************************************

/* This defines the max number of 53107 RTD cards in the system */
#define RTD_NUM_CARDS               1
#define RTD_CHANNELS_PER_CARD       2
#define RTD_NUM_CHANNELS            (RTD_NUM_CARDS * RTD_CHANNELS_PER_CARD)
#define RTD_MAX_CHANNELS            (RTD_CHANNELS_PER_CARD * 2)

#define RTD_ERROR                   0xFFFFFFFF

/* Each 53107 RTD card has two RTD converters(channels). Each RTD
 * converter requires it's own chip select.
 */
typedef struct _RTD_CHANNEL {
    MAX31865_Handle handle;         /* Handle to RTD object for channel */
    uint32_t        chipsel;        /* channel 1 chip select */
} RTD_CHANNEL;

//*****************************************************************************
//GLOBAL RUN-TIME DATA
//*****************************************************************************

typedef struct _SYSDATA
{
    uint8_t         ui8SerialNumber[16];        /* 128-bit serial number      */
    uint8_t         ui8MAC[6];                  /* 48-bit MAC from EPROM      */
    char            ipAddr[32];                 /* IP address from DHCP       */
    /* global runtime data */
    I2C_Handle      i2c0;                       /* I2C0 MAC/Serial# EPROM     */
    I2C_Handle      i2c3;
    SPI_Handle      spi2;                       /* SPI handle for slots 1 & 2 */
    SPI_Handle      spi3;                       /* SPI handle for slots 3 & 4 */
    MCP79410_Handle handleRTC;
    /* AD7799 ADC data */
    uint8_t         adcID;                      /* chip ID, 16 or 24 bit type */
    uint32_t        adcNumChannels;             /* num of ADC channels active */
    uint32_t        adcData[ADC_MAX_CHANNELS];  /* the raw ADC value          */
    float           adcUV[ADC_MAX_CHANNELS];    /* UV power level mW/cm2      */
    /* MAX31865 RTD data */
    uint32_t        rtdNumChannels;             /* num of ADC channels active */
    uint32_t        rtdData[RTD_MAX_CHANNELS];  /* the raw ADC value          */
    float           rtdTempC[RTD_MAX_CHANNELS]; /* temperature Celcius value  */
    /* RTC time/date data */
    RTCC_Struct     timeRTC;
} SYSDATA;

//*****************************************************************************
// SYSTEM CONFIG PARAMETERS STORED IN EPROM
//*****************************************************************************

typedef struct _SYSPARMS
{
    uint32_t    magic;
    uint32_t    version;
    uint32_t    build;
    /*** GLOBAL PARAMETERS ***/
    long        debug;                     	/* debug level */
    uint32_t    screensave_time;
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
    uint32_t        ui32Mask;
} CommandMessage;

//*****************************************************************************
//
//*****************************************************************************

extern SYSDATA     g_sys;
extern SYSCONFIG   g_cfg;

//*****************************************************************************
//
//*****************************************************************************

int main(void);
Void CommandTaskFxn(UArg arg0, UArg arg1);
Void SampleTaskFxn(UArg arg0, UArg arg1);
int ReadGUIDS(I2C_Handle handle, uint8_t ui8SerialNumber[16], uint8_t ui8MAC[6]);

#endif /* __PMX42_H */
