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
#include <ti/drivers/UART.h>

/* USB Driver files */
#include <usblib/usblib.h>
#include <usblib/usb-ids.h>
#include <usblib/device/usbdevice.h>
#include <usblib/device/usbdbulk.h>

/* NDK BSD support */
#include <sys/socket.h>

#include <driverlib/sysctl.h>

#include <file.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>

/* Graphiclib Header file */
#include <grlib/grlib.h>
#include "drivers/fema128x64.h"

/* PMX42 Board Header file */
#include "Board.h"
#include "PMX42.h"
#include "Utils.h"
#include "DisplayTask.h"
#include "usb_device.h"

/* Debounce time for buttons */
#define DEBOUNCE_TIME       50

//*****************************************************************************
// Global System and Config Data
//*****************************************************************************

/* Global Data Items */
SYSDATA     g_sys;
SYSCONFIG   g_cfg;

/* Global context for drawing */
tContext g_context;

/* Handles created dynamically */
Mailbox_Handle g_mailboxDisplay = NULL;
Mailbox_Handle mailboxCommand = NULL;

//*****************************************************************************
// Global ADC Card Data
//*****************************************************************************

/* This table contains the handle to each ADC channel allocated in
 * the system along with the chip select for SPI2 to access the device.
 */
static ADC_CONVERTER g_adcConverter[ADC_NUM_CARDS] = {
    {
        .handle  = NULL,
        .chipsel = Board_SLOT1_SS,              /* Slot #1, channels 1 & 2 */
    },
#if 0
    {
        .handle  = NULL,
        .chipsel = Board_SLOT2_SS,              /* Slot #2, channels 3 & 4 */
    }
#endif
};

//*****************************************************************************
// Global RTD Card Data
//*****************************************************************************

/* This table contains the handle to each RTD channel allocated in
 * the system along with the chip select for SPI2 to access the device.
 */
static RTD_CHANNEL g_rtdConverter[RTD_NUM_CHANNELS] = {
    {
        .handle   = NULL,
        .chipsel  = Board_SLOT2_MAX31865_CS1,    /* Slot #2 channel 1 */
    },
    {
        .handle   = NULL,
        .chipsel  = Board_SLOT2_MAX31865_CS2,    /* Slot #2 channel 2 */
    },
};

//*****************************************************************************
// Static Function Prototypes
//*****************************************************************************

static bool Init_Peripherals(void);
static bool Init_Devices(void);

static bool RTC_Initialize(void);

static bool ADC_Initialize(void);
static AD7799_Handle ADC_AllocConverter(SPI_Handle spiHandle, uint32_t gpioCSIndex);
static uint32_t ADC_ReadChannel(uint32_t channel);
static float ADC_to_UVPower(uint32_t adc);;

static bool RTD_Initialize(void);
static MAX31865_Handle RTD_AllocConverter(SPI_Handle spiHandle, uint32_t gpioCSIndex);
static uint8_t RTD_ReadChannel(uint32_t channel, uint32_t* adc, float* tempC);

static void gpioButtonHwi(unsigned int index);

//*****************************************************************************
// Main Entry Point
//*****************************************************************************

int main(void)
{
	Task_Params taskParams;
    Mailbox_Params mboxParams;
    Error_Block eb;

    /* default GUID & MAC values */
    memset(g_sys.ui8SerialNumber, 0xFF, 16);
    memset(g_sys.ui8MAC, 0xFF, 6);

    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
    Board_initI2C();
    Board_initSDSPI();
    Board_initSPI();
    Board_initUART();
    Board_initUSB(Board_USBDEVICE);
    // Board_initWatchdog();
    // Board_initWiFi();

    /* Initialize the OLED display device driver */
    FEMA128x64Init();
    /* Initialize the graphics context */
    GrContextInit(&g_context, &g_FEMA128x64);

    /* Enable the LED's during startup up */
    GPIO_write(Board_LED_ACT, Board_LED_ON);
    GPIO_write(Board_LED_ALM, Board_LED_OFF);

    /* Create command task mailbox */
    Error_init(&eb);
    Mailbox_Params_init(&mboxParams);
    mailboxCommand = Mailbox_create(sizeof(CommandMessage), 10, &mboxParams, &eb);
    if (mailboxCommand == NULL) {
        System_abort("Mailbox create failed\nAborting...");
    }

    /* Create display task mailbox */
    Error_init(&eb);
    Mailbox_Params_init(&mboxParams);
    g_mailboxDisplay = Mailbox_create(sizeof(DisplayMessage), 10, &mboxParams, &eb);
    if (g_mailboxDisplay == NULL) {
        System_abort("Mailbox create failed\nAborting...");
    }

    /* Create the command handler task */
    Error_init(&eb);
    Task_Params_init(&taskParams);
    taskParams.stackSize = 2048;
    taskParams.priority  = 10;
    Task_create((Task_FuncPtr)CommandTaskFxn, &taskParams, &eb);

    System_printf("Starting PMX42 execution.\n");
    System_flush();

    /* Setup the callback Hwi handler for each button */
    GPIO_setCallback(Board_BTN_UP , gpioButtonHwi);
    GPIO_setCallback(Board_BTN_DN, gpioButtonHwi);
    GPIO_setCallback(Board_BTN_NXT, gpioButtonHwi);
    GPIO_setCallback(Board_BTN_PRV, gpioButtonHwi);
    GPIO_setCallback(Board_BTN_OK, gpioButtonHwi);
    GPIO_setCallback(Board_BTN_ESC, gpioButtonHwi);

    /* Enable keypad button interrupts */
    GPIO_enableInt(Board_BTN_UP );
    GPIO_enableInt(Board_BTN_DN);
    GPIO_enableInt(Board_BTN_NXT);
    GPIO_enableInt(Board_BTN_PRV);
    GPIO_enableInt(Board_BTN_OK);
    GPIO_enableInt(Board_BTN_ESC);

    /* Start BIOS */
    BIOS_start();

    return (0);
}

//*****************************************************************************
// This is a hook into the NDK stack to allow delaying execution of the NDK
// stack task until after we load the MAC address from the AT24MAC serial
// EPROM part. This hook blocks on a semaphore until after we're able to call
// Board_initEMAC() in the CommandTaskFxn() below. This mechanism allows us
// to delay execution until we load the MAC from EPROM.
//*****************************************************************************

Void NDKStackBeginHook(void)
{
    Semaphore_pend(g_semaNDKStartup, BIOS_WAIT_FOREVER);
}

//*****************************************************************************
//
//
//*****************************************************************************

bool Init_Peripherals(void)
{
    SPI_Params  spiParams;
    I2C_Params  params;

    /*
     * I2C0 Bus - Onboard AT24MAC MAC/Serial# part
     */

    I2C_Params_init(&params);

    params.transferCallbackFxn = NULL;
    params.transferMode = I2C_MODE_BLOCKING;
    params.bitRate = I2C_100kHz;

    if ((g_sys.i2c3 = I2C_open(PMX42_I2C3, &params)) == NULL)
    {
        System_printf("Error: Unable to openI2C3 port\n");
        System_flush();
        return false;
    }

    /*
     * SPI2 Bus - Generic use, spans all slots
     */

    SPI_Params_init(&spiParams);

    spiParams.mode            = SPI_MASTER;
    spiParams.transferMode    = SPI_MODE_BLOCKING;
    spiParams.transferTimeout = 1000;
    spiParams.frameFormat     = SPI_POL1_PHA1;
    spiParams.bitRate         = 100000;            /* 1 Mhz */
    spiParams.dataSize        = 8;

    if ((g_sys.spi2 = SPI_open(Board_SPI2, &spiParams)) == NULL)
    {
        System_printf("Error: Unable to open SPI2 port\n");
        System_flush();
        return false;
    }

    /*
     * SPI3 Bus - Generic use, spans all slots
     */

    SPI_Params_init(&spiParams);

    spiParams.mode            = SPI_MASTER;
    spiParams.transferMode    = SPI_MODE_BLOCKING;
    spiParams.transferTimeout = 1000;
    spiParams.frameFormat     = SPI_POL1_PHA1;
    spiParams.bitRate         = 100000;             /* 1 Mhz */
    spiParams.dataSize        = 8;

    if ((g_sys.spi3 = SPI_open(Board_SPI3, &spiParams)) == NULL)
    {
        System_printf("Error: Unable to open SPI3 port\n");
        System_flush();
        return false;
    }

    return true;
}

//*****************************************************************************
//
//
//*****************************************************************************

bool Init_Devices(void)
{
    /* This enables the DIVSCLK output pin on PQ4
     * and generates a 1.2 Mhz clock signal on the.
     * expansion header and pin 16 of the edge
     * connector if a clock signal is required.
     */
#if (DIV_CLOCK_ENABLED > 0)
    EnableClockDivOutput(100);
#endif

    /* Create and initialize the MCP79410 RTC object */
    RTC_Initialize();

    /* Create and initialize ADC Channels */
    ADC_Initialize();

    /* Create and initialize RTD Channels */
    RTD_Initialize();

    /* Load system configuration parameters from eprom */
    ConfigParamsRead(&g_cfg);

    /* STEP-1: Read the globally unique serial number from EPROM. We are also
     * reading the 6-byte MAC address from the AT24MAC serial EPROM.
     */
    if (!ReadGUIDS(g_sys.i2c3, g_sys.ui8SerialNumber, g_sys.ui8MAC))
    {
        System_printf("Read Serial Number Failed!\n");
        System_flush();
    }

    /* STEP-2 - Don't initialize EMAC layer until after reading MAC address above! */
    Board_initEMAC(g_sys.ui8MAC);

    /* STEP-3 - Now allow the NDK task, blocked by NDKStackBeginHook(), to run */
    Semaphore_post(g_semaNDKStartup);

    /* Initialize the USB module for device mode */
    USB_init();

    System_flush();

    return true;
}

//*****************************************************************************
// Create MCP79410 object and initialize the RTC time and date.
//*****************************************************************************

bool RTC_Initialize(void)
{
    bool success = FALSE;

    if ((g_sys.handleRTC = MCP79410_create(g_sys.i2c3, NULL)) == NULL)
    {
        System_abort("MCP79410_create failed\n");
    }

    /* Initialize the RTC clock if it's not running */
    if (!MCP79410_IsRunning(g_sys.handleRTC))
    {
        RTCC_Struct ts;

        ts.hour    = (uint8_t)0;
        ts.min     = (uint8_t)0;
        ts.sec     = (uint8_t)0;
        ts.month   = (uint8_t)0;
        ts.date    = (uint8_t)1;
        ts.weekday = (uint8_t)0;
        ts.year    = (uint8_t)(2021 - 2000);

        MCP79410_Initialize(g_sys.handleRTC, &ts, H24);

        success = TRUE;
    }

    return success;
}

//*****************************************************************************
// Create and Initialize ADC Channels. Each card contains one ADC with
// two channels.
//*****************************************************************************

bool ADC_Initialize(void)
{
    bool success = FALSE;
    size_t i;

    /* Zero out DAC level array */
    for (i=0; i < ADC_MAX_CHANNELS; i++)
    {
        g_sys.uvcADC[i] = ADC_ERROR;
        g_sys.uvcPower[i] = 0.0f;
    }

    /* Assume 16-bit ADC by default */
    g_sys.adcID = AD7798_ID;

    /* Clear ADC channels found count */
    g_sys.adcNumChannels = 0;

    /* Attempt to allocate and initialize each ADC converter */
    for (i=0; i < ADC_NUM_CARDS; i++)
    {
        /* Attempt to allocate and initialize the ADC */
        g_adcConverter[i].handle = ADC_AllocConverter(g_sys.spi2, g_adcConverter[i].chipsel);

        /* ADC found, increment the channel count */
        if (g_adcConverter[i].handle == NULL)
        {
            System_printf("ADC_Initialize() failed\n");
            //SetLastError(XSYSERR_ADC_INIT);
        }
        else
        {
            g_sys.adcNumChannels += ADC_CHANNELS_PER_CARD;
            success = TRUE;
        }
    }

    return success;
}

//*****************************************************************************
// This allocates an ADC context for communication and initializes the ADC
// converter for use. This is called for each ADC in the system (two per card).
//*****************************************************************************

AD7799_Handle ADC_AllocConverter(SPI_Handle spiHandle, uint32_t gpioCSIndex)
{
    AD7799_Handle handle;

    if ((handle = AD7799_create(spiHandle, gpioCSIndex, NULL)) != NULL)
    {
        AD7799_Reset(handle);

        if ((g_sys.adcID = AD7799_Init(handle)) == 0)
        {
            /* Failed to initialize, delete the AD7799 object */
            AD7799_delete(handle);

            handle = NULL;
        }
        else
        {
            /* Set gain to 1 */
            AD7799_SetGain(handle, AD7799_GAIN_1);

            /* Set the reference detect */
            AD7799_SetRefDetect(handle, AD7799_REFDET_ENA);

            /* Set for unipolar data reading */
            AD7799_SetUnipolar(handle, AD7799_UNIPOLAR_ENA);
        }
    }

    return handle;
}

//*****************************************************************************
//
//
//*****************************************************************************

uint32_t ADC_ReadChannel(uint32_t channel)
{
    uint32_t i;
    uint32_t data = ADC_ERROR;
    uint8_t status = 0;
    AD7799_Handle handle;

    handle = g_adcConverter[channel/2].handle;

    if (!handle)
        return ADC_ERROR;

    /* Select ADC Channel-1 */
    AD7799_SetChannel(handle, channel % 2);

    /* Set the channel mode to start the single conversion */
    AD7799_SetMode(handle, AD7799_MODE_SEL(AD7799_MODE_SINGLE) | AD7799_MODE_RATE(10));

    for (i=0; i < 20; i++)
    {
        /* Check for ADC conversion complete */
        if (AD7799_IsReady(handle))
        {
            /* Read ADC channel */
            data = AD7799_ReadData(handle);

            /* Read the current ADC status and check for error */
            status = AD7799_ReadStatus(handle);

            if (status & AD7799_STAT_ERR)
                data = ADC_ERROR;

            break;
        }

        Task_sleep(10);
    }

    return data;
}

//*****************************************************************************
// This function takes and ADC value and converts it to UV-C level in
// milliwatts per centimeter squared (mW/cm2)
//*****************************************************************************

float ADC_to_UVPower(uint32_t adc)
{
    float power;

    if (adc < 0xFF)
        adc = 0;

    power = (float)adc / 6323.07f;

    return power;
}

//*****************************************************************************
// Create and Initialize RTD Channels. Each card contains two RTD converters
// with separate chip selects.
//*****************************************************************************

bool RTD_Initialize(void)
{
    size_t i;
    bool success = TRUE;

    /* Clear ADC channels found count */
    g_sys.rtdNumChannels = 0;

    /* Zero out ADC data array */
    for (i=0; i < RTD_MAX_CHANNELS; i++)
    {
        g_sys.rtdADC[i]  = RTD_ERROR;
        g_sys.rtdTempC[i] = 0.0f;
    }

    /* Attempt to allocate and initialize each ADC converter */
    for (i=0; i < RTD_NUM_CHANNELS; i++)
    {
        /* Attempt to allocate and initialize the ADC */
        g_rtdConverter[i].handle = RTD_AllocConverter(g_sys.spi2, g_rtdConverter[i].chipsel);

        /* ADC found, increment the channel count */
        if (g_rtdConverter[i].handle == NULL)
        {
            System_printf("RTD_Initialize() failed\n");
            //SetLastError(XSYSERR_ADC_INIT);
            success = FALSE;
            break;
        }

        ++g_sys.rtdNumChannels;
    }

    return success;
}

//*****************************************************************************
// This allocates an RTD context for communication and initializes the RTD
// converter for use.
//*****************************************************************************

MAX31865_Handle RTD_AllocConverter(SPI_Handle spiHandle, uint32_t gpioCSIndex)
{
    MAX31865_Handle handle;

    /* Initialize the RTD device object parameters */
    MAX31865_Params params;
    MAX31865_Params_init(&params);

    /* Set conversion to 4-wire mode with 60Hz filtering */
    uint8_t configReg = MAX31865_CFG_MODE(1) | MAX31865_CFG_3WIRE_RTD(0) | MAX31865_CFG_50HZ(0);

    params.charge_time_delay     = MAX31865_CHARGE_TIME;
    params.conversion_time_delay = MAX31865_CONVERSION_TIME;
    params.rtd                   = 100;
    params.rref                  = 400;
    params.lowFaultThreshold     = 0;
    params.highFaultThreshold    = 0xFFFF;
    params.configReg             = configReg;
    params.chipselect            = gpioCSIndex;

    /* Create the I/O expander object on SPI-2 for this card */
    if ((handle = MAX31865_create(spiHandle, &params)) != NULL)
    {
        /* Attempt to initialize the card */
        if (!MAX31865_init(handle))
        {
            /* Failed to initialize, delete the object handle created */
            MAX31865_delete(handle);

            handle = NULL;
        }
    }

    return handle;
}

//*****************************************************************************
//
//
//*****************************************************************************

uint8_t RTD_ReadChannel(uint32_t channel, uint32_t* adc, float* tempC)
{
    uint16_t data;
    uint8_t status = MAX31865_ERR_UNDEFINED;

    if (channel >= RTD_NUM_CHANNELS)
        return status;

    MAX31865_Handle handle = g_rtdConverter[channel].handle;

    if (handle)
    {
        /* Read the raw ADC value from the RTD */
        status = MAX31865_readADC(handle, &data);

        if (status == MAX31865_ERR_SUCCESS)
        {
            /* Return the raw ADC value */
            *adc = data;

            /* Return ADC value converted to Celcius */
            *tempC = MAX31865_ADC_to_Celcius(handle, data);
        }
    }

    return status;
}

//*****************************************************************************
//
//
//*****************************************************************************

Void SampleTaskFxn(UArg arg0, UArg arg1)
{
    size_t channel;
    uint32_t adc;

    while(1)
    {
        /* No message, blink the LED */
        GPIO_toggle(Board_LED_ACT);

        /* Read the current date/time stamp from the RTC */
        MCP79410_GetTime(g_sys.handleRTC, &g_sys.timeRTC);

        /*
         *  Read ADC - If ADC cards were found, then read the data
         */

        if ((g_sys.adcNumChannels > 0) && (g_sys.adcID != 0))
        {
            for (channel=0; channel < ADC_NUM_CHANNELS; channel++)
            {
                adc = ADC_ReadChannel(channel);

                if (adc != ADC_ERROR)
                {
                    g_sys.uvcADC[channel] = adc;
                    g_sys.uvcPower[channel] = ADC_to_UVPower(adc);
                }
                else
                {
                    g_sys.uvcADC[channel] = ADC_ERROR;
                    g_sys.uvcPower[channel] = 0.0f;
                }
            }
        }

        /*
         *  Read RTD - If RTD cards were found, then read the data
         */

        if (g_sys.rtdNumChannels > 0)
        {
            for (channel=0; channel < RTD_NUM_CHANNELS; channel++)
            {
                uint8_t status;
                float tempC;

                status = RTD_ReadChannel(channel, &adc, &tempC);

                if (status == MAX31865_ERR_SUCCESS)
                {
                    g_sys.rtdADC[channel]  = adc;
                    g_sys.rtdTempC[channel] = tempC;
                }
                else
                {
                    g_sys.rtdADC[channel]  = RTD_ERROR;
                }
            }
        }

        /* Now tell the display task to refresh */
        //DisplayRefresh();

        /* Sleep a bit before next sample reads */
        Task_sleep(500);
    }
}

//*****************************************************************************
//
//
//*****************************************************************************

Void CommandTaskFxn(UArg arg0, UArg arg1)
{
    Error_Block eb;
	Task_Params taskParams;
    CommandMessage msgCmd;

    /* Open the peripherals we plan to use */
    Init_Peripherals();

    /* Initialize any I/O cards in the slots */
    Init_Devices();

    /* Startup the OLED Display Task */
    Error_init(&eb);
    Task_Params_init(&taskParams);
    taskParams.stackSize = 2048;
    taskParams.priority  = 9;
    Task_create((Task_FuncPtr)DisplayTaskFxn, &taskParams, &eb);

    /* Startup the ADC sample read task */
    Error_init(&eb);
    Task_Params_init(&taskParams);
    taskParams.stackSize = 2048;
    taskParams.priority  = 10;
    Task_create((Task_FuncPtr)SampleTaskFxn, &taskParams, &eb);

    /*
     * Now begin the main program loop processing button press events
     */

    while (true)
    {
    	/* Wait for a message up to 1 second */
        if (!Mailbox_pend(mailboxCommand, &msgCmd, BIOS_WAIT_FOREVER))
        {
            continue;
        }

        if (msgCmd.command == BUTTONPRESS)
        {
            uint32_t index = msgCmd.ui32Data;

            if (IsScreenSave())
            {
                DisplayRefresh();

                /* Debounce after button was pressed */
                Task_sleep(DEBOUNCE_TIME);
                /* Now wait for the button to release */
                while (!(GPIO_read(index)));
                /* Debounce after button released */
                Task_sleep(DEBOUNCE_TIME);
                /* Now re-enable button press interrupt again */
                GPIO_enableInt(index);
                continue;
            }

            ScreenNum screen = DisplayGetScreen();

            switch(msgCmd.ui32Data)
            {
            case Board_BTN_UP :
                if (screen == SCREEN_UV)
                {
                    TempFormat format;

                    format = (TempFormat)(g_cfg.temp_format + 1);

                    if (format >= TEMP_FORMAT_LAST)
                        format = 0;

                    g_cfg.temp_format = (uint8_t)format;
                }
                break;

            case Board_BTN_DN:
                break;

            case Board_BTN_NXT:
                if (++screen > SCREEN_LAST)
                    screen = SCREEN_FIRST;
                DisplaySetScreen(screen);
                break;

            case Board_BTN_PRV:
                if (screen == SCREEN_FIRST)
                    screen = SCREEN_LAST;
                else
                    --screen;
                DisplaySetScreen(screen);
                break;

            case Board_BTN_OK:
                //GPIO_toggle(Board_SLOT2_RELAY1);
                break;

            case Board_BTN_ESC:
               // GPIO_toggle(Board_SLOT2_RELAY2);
                break;

            default:
                break;
            }

            /* Debounce after button was pressed */
            Task_sleep(DEBOUNCE_TIME);
            /* Now wait for the button to release */
            while (!(GPIO_read(index)));
            /* Debounce after button released */
            Task_sleep(DEBOUNCE_TIME);
            /* Now re-enable button press interrupt again */
            GPIO_enableInt(index);
        }
    }
}

//*****************************************************************************
// HWI Callback function for front panel push button interrupts.
//*****************************************************************************

void gpioButtonHwi(unsigned int index)
{
    uint32_t mask;
    CommandMessage msg;

    /* GPIO pin interrupt occurred, read button state */
    mask = GPIO_read(index);

    GPIO_clearInt(index);

    /* Disable interrupt for now, the command task will
     * re-enable this after it processes the button press
     * message and debounces the button press.
     */
    GPIO_disableInt(index);

    msg.command  = BUTTONPRESS;
    msg.ui32Data = index;
    msg.ui32Mask = mask;

    Mailbox_post(mailboxCommand, &msg, BIOS_NO_WAIT);
}

// End-Of-File
