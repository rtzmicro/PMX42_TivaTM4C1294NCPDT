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
#include "AD7793.h"
#include "DisplayTask.h"
#include "usb_device.h"

/* Debounce time for buttons */
#define DEBOUNCE_TIME       40

/* Global context for drawing */
tContext g_context;

/* Handles created dynamically */
Mailbox_Handle g_mailboxDisplay = NULL;
Mailbox_Handle mailboxCommand = NULL;

SYSDATA     g_sys;
SYSCONFIG   g_cfg;

/* External Data Items */

/* Static Function Prototypes */

void gpioButtonHwi(unsigned int index);

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
    GPIO_write(Board_STAT_LED1, Board_LED_ON);
    GPIO_write(Board_STAT_LED2, Board_LED_OFF);

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
    GPIO_setCallback(Board_BTN_SW1, gpioButtonHwi);
    GPIO_setCallback(Board_BTN_SW2, gpioButtonHwi);
    GPIO_setCallback(Board_BTN_SW3, gpioButtonHwi);
    GPIO_setCallback(Board_BTN_SW4, gpioButtonHwi);
    GPIO_setCallback(Board_BTN_SW5, gpioButtonHwi);
    GPIO_setCallback(Board_BTN_SW6, gpioButtonHwi);

    /* Enable keypad button interrupts */
    GPIO_enableInt(Board_BTN_SW1);
    GPIO_enableInt(Board_BTN_SW2);
    GPIO_enableInt(Board_BTN_SW3);
    GPIO_enableInt(Board_BTN_SW4);
    GPIO_enableInt(Board_BTN_SW5);
    GPIO_enableInt(Board_BTN_SW6);

    /* Start BIOS */
    BIOS_start();

    return (0);
}

//*****************************************************************************
//
//
//*****************************************************************************

bool Init_Peripherals(void)
{
    SPI_Params  spiParams;

    /*
     * Slots 1 & 2 share quad-speed SPI-2 bus
     */

    SPI_Params_init(&spiParams);

    spiParams.mode            = SPI_MASTER;
    spiParams.transferMode    = SPI_MODE_BLOCKING;
    spiParams.transferTimeout = 1000;
    spiParams.frameFormat     = SPI_POL1_PHA1;
    spiParams.bitRate         = 100000;            /* 500 Khz */
    spiParams.dataSize        = 8;

    if ((g_sys.spi12 = SPI_open(Board_SLOT12_SPI, &spiParams)) == NULL)
    {
        System_printf("Error: Unable to open SPI2 port\n");
        System_flush();
        return false;
    }

    /*
     * Slots 3 & 4 share quad-speed SPI-3 bus
     */

    SPI_Params_init(&spiParams);

    spiParams.mode            = SPI_MASTER;
    spiParams.transferMode    = SPI_MODE_BLOCKING;
    spiParams.transferTimeout = 1000;
    spiParams.frameFormat     = SPI_POL1_PHA1;
    spiParams.bitRate         = 100000;             /* 500 Khz */
    spiParams.dataSize        = 8;

    if ((g_sys.spi34 = SPI_open(Board_SLOT34_SPI, &spiParams)) == NULL)
    {
        System_printf("Error: Unable to open SPI2 port\n");
        System_flush();
        return false;
    }

    return true;
}

//*****************************************************************************
//
//
//*****************************************************************************

bool Init_IO_Cards(void)
{
    /* This enables the DIVSCLK output pin on PQ4
     * and generates a 1.2 Mhz clock signal on the.
     * expansion header and pin 16 of the edge
     * connector if a clock signal is required.
     */
#if (DIV_CLOCK_ENABLED > 0)
    EnableClockDivOutput(100);
#endif

    /*
     * Create and initialize the AD7799 objects.
     */

    if ((g_sys.AD7799HandleSlot1 = AD7799_create(g_sys.spi12, Board_SLOT1_SS, Board_SLOT1_RDY, NULL)) == NULL)
    {
        System_printf("AD7799_create failed\n");
        return false;
    }

    if ((g_sys.AD7799HandleSlot2 = AD7799_create(g_sys.spi12, Board_SLOT2_SS, Board_SLOT2_RDY, NULL)) == NULL)
    {
        System_printf("AD7799_create failed\n");
        return false;
    }

    /*
     * Attempt to reset, initialize & detect presence of I/O cards
     */
#if 0
    AD7799_Reset(g_sys.AD7799HandleSlot1);

    if (AD7799_Init(g_sys.AD7799HandleSlot1) == 0)
    {
       System_printf("AD7799_Init() failed\n");
    }
    else
    {
        /* Set gain to 1 */
        AD7799_SetGain(g_sys.AD7799HandleSlot1, AD7799_GAIN_1);
        /* use AIN1(+) - AIN1(-) */
        AD7799_SetChannel(g_sys.AD7799HandleSlot1, AD7799_CH_AIN1P_AIN1M);
        /* use AIN2(+) - AIN2(-) */
        AD7799_SetChannel(g_sys.AD7799HandleSlot1, AD7799_CH_AIN2P_AIN2M);
        /* Set the reference detect */
        AD7799_SetReference(g_sys.AD7799HandleSlot1, AD7799_REFDET_ENA);
    }
#endif

    AD7799_Reset(g_sys.AD7799HandleSlot2);

    if (AD7799_Init(g_sys.AD7799HandleSlot2) == 0)
    {
        System_printf("AD7799_Init() failed\n");
    }
    else
    {
#if 1
        /* Set gain to 1 */
        AD7799_SetGain(g_sys.AD7799HandleSlot2, AD7799_GAIN_1);
        /* use AIN1(+) - AIN1(-) */
        AD7799_SetChannel(g_sys.AD7799HandleSlot2, AD7799_CH_AIN1P_AIN1M);
        /* use AIN2(+) - AIN2(-) */
        AD7799_SetChannel(g_sys.AD7799HandleSlot2, AD7799_CH_AIN2P_AIN2M);
        /* Set the reference detect */
        AD7799_SetReference(g_sys.AD7799HandleSlot2, AD7799_REFDET_ENA);
#endif
    }

    System_flush();

    return true;
}

//*****************************************************************************
// This is a hook into the NDK stack to allow delaying execution of the NDK
// stack task until after we load the MAC address from the AT24MAC serial
// EPROM part. This hook blocks on a semaphore until after we're able to call
// Board_initEMAC() in the CommandTaskFxn() below. This mechanism allows us
// to delay execution until we load the MAC from EPROM.
//*****************************************************************************

void NDKStackBeginHook(void)
{
    Semaphore_pend(g_semaNDKStartup, BIOS_WAIT_FOREVER);
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
    DisplayMessage msgDisp;

    /* STEP-1: Read the globally unique serial number from EPROM. We are also
     * reading the 6-byte MAC address from the AT24MAC serial EPROM.
     */
    if (!ReadGUIDS(g_sys.ui8SerialNumber, g_sys.ui8MAC))
    {
        System_printf("Read Serial Number Failed!\n");
        System_flush();
    }

    /* STEP-2 - Don't initialize EMAC layer until after reading MAC address above! */
    Board_initEMAC();

    /* STEP-3 - Now allow the NDK task, blocked by NDKStackBeginHook(), to run */
    Semaphore_post(g_semaNDKStartup);

    /* Open the peripherals we plan to use */
    Init_Peripherals();

    /* Initialize any I/O cards in the slots */
    Init_IO_Cards();

    /* Initialize the USB module for device mode */
    USB_init();

    /*
     * Create the display task
     */

    Error_init(&eb);
    Task_Params_init(&taskParams);
    taskParams.stackSize = 2048;
    taskParams.priority  = 5;
    Task_create((Task_FuncPtr)DisplayTaskFxn, &taskParams, &eb);

    /* Now begin the main program command task processing loop */

    while (true)
    {
    	/* Wait for a message up to 1 second */
        if (!Mailbox_pend(mailboxCommand, &msgCmd, 500))
        {
        	/* No message, blink the LED */
    		GPIO_toggle(Board_STAT_LED1);
    		continue;
        }

        if (msgCmd.command == BUTTONPRESS)
        {
            bool buttonValid = true;
            unsigned int index = msgCmd.ui32Data;

            switch(msgCmd.ui32Data)
            {
            case Board_BTN_SW1:
                msgDisp.dispCommand = SCREEN_NEXT;
                Mailbox_post(g_mailboxDisplay, &msgDisp, BIOS_NO_WAIT);
                break;

            case Board_BTN_SW2:
                msgDisp.dispCommand = SCREEN_PREV;
                Mailbox_post(g_mailboxDisplay, &msgDisp, BIOS_NO_WAIT);
                break;

            case Board_BTN_SW3:
                /* Toggle temp display mode */
                /* Update the screen */
                msgDisp.dispCommand = SCREEN_REFRESH;
                Mailbox_post(g_mailboxDisplay, &msgDisp, BIOS_NO_WAIT);
                break;

            case Board_BTN_SW4:
                break;

            case Board_BTN_SW5:
                //GPIO_toggle(Board_SLOT2_RELAY1);
                break;

            case Board_BTN_SW6:
               // GPIO_toggle(Board_SLOT2_RELAY2);
                break;

            default:
                buttonValid = false;
                break;
            }

            if (buttonValid)
            {
                uint32_t i;

                /* Some debounce time for the button */
                Task_sleep(DEBOUNCE_TIME);

                /* Wait for button to release, then re-enable interrupt */
                for (i=0; i < 100; i++)
                {
                    if (GPIO_read(index))
                        break;

                    Task_sleep(10);
                }

                Task_sleep(DEBOUNCE_TIME);

                /* Re-enable the interrupt for the button pressed */
                GPIO_enableInt(index);
            }
        }
    }
}

//*****************************************************************************
// HWI Callback function for front panel push button interrupts.
//*****************************************************************************

void gpioButtonHwi(unsigned int index)
{
	uint32_t btn;
    CommandMessage msg;

	/* GPIO pin interrupt occurred, read current button state mask */
    btn = GPIO_read(index);

    GPIO_clearInt(index);

    if (btn)
    {
        /* Disable interrupt for now, the command task will
         * re-enable this after it processes the button press
         * message and debounces the button press.
         */
        GPIO_disableInt(index);

        msg.command  = BUTTONPRESS;
        msg.ui32Data = index;
        Mailbox_post(mailboxCommand, &msg, BIOS_NO_WAIT);
    }
}

// End-Of-File
