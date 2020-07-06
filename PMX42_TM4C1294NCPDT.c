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
 *  ======== PMX42.c ========
 *  This file is responsible for setting up the board specific items for the
 *  PMX42 board.
 */

#include <stdint.h>
#include <stdbool.h>

#include <xdc/std.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/family/arm/m3/Hwi.h>

#include <inc/hw_ints.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_gpio.h>

#include <driverlib/flash.h>
#include <driverlib/gpio.h>
#include <driverlib/i2c.h>
#include <driverlib/pin_map.h>
#include <driverlib/pwm.h>
#include <driverlib/ssi.h>
#include <driverlib/sysctl.h>
#include <driverlib/uart.h>
#include <driverlib/udma.h>
#include <driverlib/eeprom.h>

#include "PMX42_TM4C1294NCPDT.h"

#include "PMX42.h"

/* Global STC-1200 System data */
extern SYSDATA g_sys;

#ifndef TI_DRIVERS_UART_DMA
#define TI_DRIVERS_UART_DMA 0
#endif

/*
 *  =============================== DMA ===============================
 */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN(dmaControlTable, 1024)
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=1024
#elif defined(__GNUC__)
__attribute__ ((aligned (1024)))
#endif
static tDMAControlTable dmaControlTable[32];
static bool dmaInitialized = false;

/* Hwi_Struct used in the initDMA Hwi_construct call */
static Hwi_Struct dmaHwiStruct;

/* Hwi_Struct used in the usbBusFault Hwi_construct call */
static Hwi_Struct usbBusFaultHwiStruct;

/*
 *  ======== dmaErrorHwi ========
 */
static Void dmaErrorHwi(UArg arg)
{
    System_printf("DMA error code: %d\n", uDMAErrorStatusGet());
    uDMAErrorStatusClear();
    System_abort("DMA error!!");
}

/*
 *  ======== PMX42_usbBusFaultHwi ========
 */
static Void PMX42_usbBusFaultHwi(UArg arg)
{
    /*
     *  This function should be modified to appropriately manage handle
     *  a USB bus fault.
    */
    System_printf("USB bus fault detected.");
    Hwi_clearInterrupt(INT_GPIOQ4);
    System_abort("USB error!!");
}

/*
 *  ======== PMX42_initDMA ========
 */
void PMX42_initDMA(void)
{
    Error_Block eb;
    Hwi_Params  hwiParams;

    if (!dmaInitialized) {
        Error_init(&eb);
        Hwi_Params_init(&hwiParams);
        Hwi_construct(&(dmaHwiStruct), INT_UDMAERR, dmaErrorHwi,
                      &hwiParams, &eb);
        if (Error_check(&eb)) {
            System_abort("Couldn't construct DMA error hwi");
        }

        SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
        uDMAEnable();
        uDMAControlBaseSet(dmaControlTable);

        dmaInitialized = true;
    }
}

/*
 *  =============================== General ===============================
 */
/*
 *  ======== PMX42_initGeneral ========
 */
void PMX42_initGeneral(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOJ);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOR);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOS);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOT);

    // Initialize the EEPROM so we can access it later

    SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);

    if (EEPROMInit() != EEPROM_INIT_OK)
        System_abort("EEPROMInit() failed!\n");

    uint32_t size = EEPROMSizeGet();
}

/*
 *  =============================== EMAC ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(EMAC_config, ".const:EMAC_config")
#pragma DATA_SECTION(emacHWAttrs, ".const:emacHWAttrs")
#pragma DATA_SECTION(NIMUDeviceTable, ".data:NIMUDeviceTable")
#endif

#include <ti/drivers/EMAC.h>
#include <ti/drivers/emac/EMACSnow.h>

/*
 *  Required by the Networking Stack (NDK). This array must be NULL terminated.
 *  This can be removed if NDK is not used.
 *  Double curly braces are needed to avoid GCC bug #944572
 *  https://bugs.launchpad.net/gcc-linaro/+bug/944572
 */
NIMU_DEVICE_TABLE_ENTRY NIMUDeviceTable[2] = {
    {
#if TI_EXAMPLES_PPP
        /* Use PPP driver for PPP example only */
        .init = USBSerialPPP_NIMUInit
#else
        /* Default: use Ethernet driver */
        .init = EMACSnow_NIMUInit
#endif
    },
    {NULL}
};

EMACSnow_Object emacObjects[PMX42_EMACCOUNT];

/*
 *  EMAC configuration structure
 *  Set user/company specific MAC octates. The following sets the address
 *  to ff-ff-ff-ff-ff-ff. Users need to change this to make the label on
 *  their boards.
 */
unsigned char macAddress[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

const EMACSnow_HWAttrs emacHWAttrs[PMX42_EMACCOUNT] = {
    {
        .baseAddr    = EMAC0_BASE,
        .intNum      = INT_EMAC0,
        .intPriority = (~0),
        .macAddress  = macAddress
    }
};

const EMAC_Config EMAC_config[] = {
    {
        .fxnTablePtr = &EMACSnow_fxnTable,
        .object      = &emacObjects[0],
        .hwAttrs     = &emacHWAttrs[0]
    },
    {NULL, NULL, NULL}
};

/*
 *  ======== PMX42_initEMAC ========
 */
void PMX42_initEMAC(void)
{
    memcpy(macAddress, g_sys.ui8MAC, 6);

#if 0
    uint32_t ulUser0, ulUser1;

    /* Get the MAC address */
    FlashUserGet(&ulUser0, &ulUser1);
    if ((ulUser0 != 0xffffffff) && (ulUser1 != 0xffffffff)) {
        System_printf("Using MAC address in flash\n");
        /*
         *  Convert the 24/24 split MAC address from NV ram into a 32/16 split MAC
         *  address needed to program the hardware registers, then program the MAC
         *  address into the Ethernet Controller registers.
         */
        macAddress[0] = ((ulUser0 >>  0) & 0xff);
        macAddress[1] = ((ulUser0 >>  8) & 0xff);
        macAddress[2] = ((ulUser0 >> 16) & 0xff);
        macAddress[3] = ((ulUser1 >>  0) & 0xff);
        macAddress[4] = ((ulUser1 >>  8) & 0xff);
        macAddress[5] = ((ulUser1 >> 16) & 0xff);
    }
    else if (macAddress[0] == 0xff && macAddress[1] == 0xff &&
             macAddress[2] == 0xff && macAddress[3] == 0xff &&
             macAddress[4] == 0xff && macAddress[5] == 0xff) {
        System_abort("Change the macAddress variable to match your boards MAC sticker");
    }
#endif
    // Enable peripheral EPHY0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_EPHY0);

    GPIOPinConfigure(GPIO_PF0_EN0LED0);  /* PMX42_J2_GRN */
    GPIOPinTypeEthernetLED(GPIO_PORTF_BASE, GPIO_PIN_0);

    GPIOPinConfigure(GPIO_PK6_EN0LED1);  /* PMX42_J2_YEL */
    GPIOPinTypeEthernetLED(GPIO_PORTK_BASE, GPIO_PIN_6);

    /* Once EMAC_init is called, EMAC_config cannot be changed */
    EMAC_init();
}

/*
 *  =============================== GPIO ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(GPIOTiva_config, ".const:GPIOTiva_config")
#endif

#include <ti/drivers/GPIO.h>
#include <ti/drivers/gpio/GPIOTiva.h>

/*
 * Array of Pin configurations
 * NOTE: The order of the pin configurations must coincide with what was
 *       defined in PMX42_TM4C1294NCPDT.h
 * NOTE: Pins not used for interrupts should be placed at the end of the
 *       array.  Callback entries can be omitted from callbacks array to
 *       reduce memory usage.
 */
GPIO_PinConfig gpioPinConfigs[] = {
    /* Input Interrupt pins */
    /* PMX42_BTN_SW1 */
    GPIOTiva_PN_0 | GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_RISING,
    /* PMX42_BTN_SW2 */
    GPIOTiva_PN_1 | GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_RISING,
    /* PMX42_BTN_SW3 */
    GPIOTiva_PN_2 | GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_RISING,
    /* PMX42_BTN_SW4 */
    GPIOTiva_PN_3 | GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_RISING,
    /* PMX42_BTN_SW5 */
    GPIOTiva_PN_4 | GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_RISING,
    /* PMX42_BTN_SW6 */
    GPIOTiva_PN_5 | GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_RISING,

    /* Output pins */
    /* PMX42_STAT_LED1 */
    GPIOTiva_PP_2 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW,
    /* PMX42_STAT_LED2 */
    GPIOTiva_PP_3 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW,

    /* PMX42_SLOT1 */
    GPIOTiva_PD_2 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH,
    GPIOTiva_PM_4 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW,
    GPIOTiva_PM_5 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW,
    GPIOTiva_PM_6 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW,

    /* PMX42_SLOT2 */
    GPIOTiva_PM_2 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_HIGH,
    GPIOTiva_PF_1 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW,
    GPIOTiva_PF_2 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW,
    GPIOTiva_PF_3 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW,

    /* PMX42_SLOT3_GPIO_DE : PL4=RS422 DE */
    GPIOTiva_PL_4 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW,
    /* PMX42_SLOT3_GPIO_RE : PQ1=RS422 RE */
    GPIOTiva_PQ_1 | GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW,
};

/*
 * Array of callback function pointers
 * NOTE: The order of the pin configurations must coincide with what was
 *       defined in PMX42_TM4C1294NCPDT.h
 * NOTE: Pins not used for interrupts can be omitted from callbacks array to
 *       reduce memory usage (if placed at end of gpioPinConfigs array).
 */
GPIO_CallbackFxn gpioCallbackFunctions[] = {
	NULL,	/* PMX42_BTN_SW1 */
    NULL, 	/* PMX42_BTN_SW2 */
    NULL,	/* PMX42_BTN_SW3 */
    NULL,	/* PMX42_BTN_SW4 */
    NULL,	/* PMX42_BTN_SW5 */
    NULL	/* PMX42_BTN_SW6 */
};

/* The device-specific GPIO_config structure */
const GPIOTiva_Config GPIOTiva_config = {
    .pinConfigs         = (GPIO_PinConfig *)gpioPinConfigs,
    .callbacks          = (GPIO_CallbackFxn *)gpioCallbackFunctions,
    .numberOfPinConfigs = sizeof(gpioPinConfigs)/sizeof(GPIO_PinConfig),
    .numberOfCallbacks  = sizeof(gpioCallbackFunctions)/sizeof(GPIO_CallbackFxn),
    .intPriority        = (~0)
};

/*
 *  ======== PMX42_initGPIO ========
 */
void PMX42_initGPIO(void)
{
    /* Setup the button GPIO input pins used */
    GPIOPinTypeGPIOInput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1 |
                         GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);

    /* Setup LED GPIO output pins used (PP2 & PP3) */
    GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_2 | GPIO_PIN_3);

    /* Setup SLOT-1 : SSI2 slave select pins */
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_2);		/* SLOT1: PD2/SLOT1_SS */
    GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_4);		/* SLOT1: PM4/T4CCP0   */
    GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_5);     /* SLOT1: PM5/T4CCP1   */
    GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_6);     /* SLOT1: PM6/T5CCP0   */

    /* Setup SLOT-2 : Enable for GPIOOutputs */
    GPIOPinTypeGPIOOutput(GPIO_PORTM_BASE, GPIO_PIN_2);     /* SLOT2: PM2/SLOT2_SS */
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);		/* SLOT2: PF1/M0PWM1   */
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);		/* SLOT2: PF2/M0PWM2   */
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);     /* SLOT2: PF3/M0PWM3   */

    /* Setup SLOT-3 : RS-422 DE and RE control pins */
    GPIOPinTypeGPIOOutput(GPIO_PORTL_BASE, GPIO_PIN_4);		/* SLOT3: PL4 (DE) */
    GPIOPinTypeGPIOOutput(GPIO_PORTQ_BASE, GPIO_PIN_1);		/* SLOT3: PQ1 (RE) */

    /* Once GPIO_init is called, GPIO_config cannot be changed */
    GPIO_init();
}

/*
 *  =============================== I2C ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(I2C_config, ".const:I2C_config")
#pragma DATA_SECTION(i2cTivaHWAttrs, ".const:i2cTivaHWAttrs")
#endif

#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CTiva.h>

I2CTiva_Object i2cTivaObjects[PMX42_I2CCOUNT];

const I2CTiva_HWAttrs i2cTivaHWAttrs[PMX42_I2CCOUNT] = {
    {
        .baseAddr    = I2C0_BASE,
        .intNum      = INT_I2C0,
        .intPriority = (~0)
    },
    {
        .baseAddr    = I2C1_BASE,
        .intNum      = INT_I2C1,
        .intPriority = (~0)
    },
    {
        .baseAddr    = I2C2_BASE,
        .intNum      = INT_I2C2,
        .intPriority = (~0)
    },
    {
        .baseAddr    = I2C3_BASE,
        .intNum      = INT_I2C3,
        .intPriority = (~0)
    }
};

const I2C_Config I2C_config[] = {
    {
        .fxnTablePtr = &I2CTiva_fxnTable,
        .object      = &i2cTivaObjects[0],
        .hwAttrs     = &i2cTivaHWAttrs[0]
    },
    {
        .fxnTablePtr = &I2CTiva_fxnTable,
        .object      = &i2cTivaObjects[1],
        .hwAttrs     = &i2cTivaHWAttrs[1]
    },
    {
        .fxnTablePtr = &I2CTiva_fxnTable,
        .object      = &i2cTivaObjects[2],
        .hwAttrs     = &i2cTivaHWAttrs[2]
    },
    {
        .fxnTablePtr = &I2CTiva_fxnTable,
        .object      = &i2cTivaObjects[3],
        .hwAttrs     = &i2cTivaHWAttrs[3]
    },
    {NULL, NULL, NULL}
};

/*
 *  ======== PMX42_initI2C ========
 */
void PMX42_initI2C(void)
{
    /* I2C0 Init */
    /* Enable the peripheral */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);
    /* Configure the appropriate pins to be I2C instead of GPIO. */
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    /* I2C1 Init */
    /* Enable the peripheral */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
    /* Configure the appropriate pins to be I2C instead of GPIO. */
    GPIOPinConfigure(GPIO_PG0_I2C1SCL);
    GPIOPinConfigure(GPIO_PG1_I2C1SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTG_BASE, GPIO_PIN_0);
    GPIOPinTypeI2C(GPIO_PORTG_BASE, GPIO_PIN_1);

    /* I2C2 Init */
    /* Enable the peripheral */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);
    /* Configure the appropriate pins to be I2C instead of GPIO. */
    GPIOPinConfigure(GPIO_PL1_I2C2SCL);
    GPIOPinConfigure(GPIO_PL0_I2C2SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTL_BASE, GPIO_PIN_1);
    GPIOPinTypeI2C(GPIO_PORTL_BASE, GPIO_PIN_0);

    /* I2C3 Init */
    /* Enable the peripheral */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);
    /* Configure the appropriate pins to be I2C instead of GPIO. */
    GPIOPinConfigure(GPIO_PK4_I2C3SCL);
    GPIOPinConfigure(GPIO_PK5_I2C3SDA);
    GPIOPinTypeI2CSCL(GPIO_PORTK_BASE, GPIO_PIN_4);
    GPIOPinTypeI2C(GPIO_PORTK_BASE, GPIO_PIN_5);

    I2C_init();
}

/*
 *  =============================== PWM ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(PWM_config, ".const:PWM_config")
#pragma DATA_SECTION(pwmTivaHWAttrs, ".const:pwmTivaHWAttrs")
#endif

#include <ti/drivers/PWM.h>
#include <ti/drivers/pwm/PWMTiva.h>
#include <driverlib/pwm.h>

PWMTiva_Object pwmTivaObjects[PMX42_PWMCOUNT];

const PWMTiva_HWAttrs pwmTivaHWAttrs[PMX42_PWMCOUNT] = {
    {
        .baseAddr   = PWM0_BASE,
        .pwmOutput  = PWM_OUT_0,
        .pwmGenOpts = PWM_GEN_MODE_DOWN | PWM_GEN_MODE_DBG_RUN
    }
};

const PWM_Config PWM_config[] = {
    {
        .fxnTablePtr = &PWMTiva_fxnTable,
        .object      = &pwmTivaObjects[0],
        .hwAttrs     = &pwmTivaHWAttrs[0]
    },
    {NULL, NULL, NULL}
};

/*
 *  ======== PMX42_initPWM ========
 */
void PMX42_initPWM(void)
{
    /* Enable PWM peripherals */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);

    /*
     * Enable PWM output on GPIO pins.  PWM output is connected to an Ethernet
     * LED on the development board (D4).  The PWM configuration
     * below will disable Ethernet functionality.
     */
    GPIOPinConfigure(GPIO_PF0_M0PWM0);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_0);

    PWM_init();
}

/*
 *  =============================== SDSPI ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(SDSPI_config, ".const:SDSPI_config")
#pragma DATA_SECTION(sdspiTivaHWattrs, ".const:sdspiTivaHWattrs")
#endif

#include <ti/drivers/SDSPI.h>
#include <ti/drivers/sdspi/SDSPITiva.h>

/* SDSPI objects */
SDSPITiva_Object sdspiTivaObjects[PMX42_SDSPICOUNT];

const SDSPITiva_HWAttrs sdspiTivaHWattrs[PMX42_SDSPICOUNT] = {
    {
   		/* SD Card Socket (J1) */
        .baseAddr = SSI1_BASE,

        .portSCK  = GPIO_PORTB_BASE,
        .pinSCK   = GPIO_PIN_5,
        .portMISO = GPIO_PORTE_BASE,
        .pinMISO  = GPIO_PIN_5,
        .portMOSI = GPIO_PORTE_BASE,
        .pinMOSI  = GPIO_PIN_4,
        .portCS   = GPIO_PORTK_BASE,
        .pinCS    = GPIO_PIN_7,
    },
    {
   		/* OnBoard Flash (U206) */
        .baseAddr = SSI1_BASE,

        .portSCK  = GPIO_PORTQ_BASE,
        .pinSCK   = GPIO_PIN_0,
        .portMISO = GPIO_PORTE_BASE,
        .pinMISO  = GPIO_PIN_5,
        .portMOSI = GPIO_PORTE_BASE,
        .pinMOSI  = GPIO_PIN_4,
        .portCS   = GPIO_PORTK_BASE,
        .pinCS    = GPIO_PIN_4,
    }
};

const SDSPI_Config SDSPI_config[] = {
    {
        .fxnTablePtr = &SDSPITiva_fxnTable,
        .object      = &sdspiTivaObjects[0],
        .hwAttrs     = &sdspiTivaHWattrs[0]
    },
    {
        .fxnTablePtr = &SDSPITiva_fxnTable,
        .object      = &sdspiTivaObjects[1],
        .hwAttrs     = &sdspiTivaHWattrs[1]
    },
    {NULL, NULL, NULL}
};

/*
 *  ======== PMX42_initSDSPI ========
 */
void PMX42_initSDSPI(void)
{
    /* Enable SD SSI peripherals */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);

    /* SSI-1 Configure Pins (must be done first!) */

    // Enable pin PE5 for SSI1 SSI1XDAT1
    GPIOPinConfigure(GPIO_PE5_SSI1XDAT1);
    GPIOPinTypeSSI(GPIO_PORTE_BASE, GPIO_PIN_5);

    // Enable pin PE4 for SSI1 SSI1XDAT0
    GPIOPinConfigure(GPIO_PE4_SSI1XDAT0);
    GPIOPinTypeSSI(GPIO_PORTE_BASE, GPIO_PIN_4);

    // Enable pin PB5 for SSI1 SSI1CLK
    GPIOPinConfigure(GPIO_PB5_SSI1CLK);
    GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_5);

    // Enable pin PB4 for SSI1 SSI1FSS
    GPIOPinConfigure(GPIO_PB4_SSI1FSS);
    GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_4);

    /* Now, configure the pad settings */

    /* SCK (PB5) */
    GPIOPadConfigSet(GPIO_PORTB_BASE,
                     GPIO_PIN_5,
                     GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
    /* MOSI (PE4) */
    GPIOPadConfigSet(GPIO_PORTE_BASE,
                     GPIO_PIN_4,
                     GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
    /* MISO (PE5) */
    GPIOPadConfigSet(GPIO_PORTE_BASE,
                     GPIO_PIN_5,
                     GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD_WPU);
    /* CS (PB4) */
    GPIOPadConfigSet(GPIO_PORTB_BASE,
                     GPIO_PIN_4,
                     GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);

    SDSPI_init();
}


/*
 *  =============================== SPI ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(SPI_config, ".const:SPI_config")
#pragma DATA_SECTION(spiTivaDMAHWAttrs, ".const:spiTivaDMAHWAttrs")
#endif

#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPITivaDMA.h>

/* SPI objects */
SPITivaDMA_Object spiTivaDMAObjects[PMX42_SPICOUNT];
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN(spiTivaDMAscratchBuf, 32)
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment=32
#elif defined(__GNUC__)
__attribute__ ((aligned (32)))
#endif
uint32_t spiTivaDMAscratchBuf[PMX42_SPICOUNT];

const SPITivaDMA_HWAttrs spiTivaDMAHWAttrs[PMX42_SPICOUNT] = {
    {
        .baseAddr               = SSI2_BASE,
        .intNum                 = INT_SSI2,
        .intPriority            = (~0),
        .scratchBufPtr          = &spiTivaDMAscratchBuf[0],
        .defaultTxBufValue      = 0,
        .rxChannelIndex         = UDMA_SEC_CHANNEL_UART2RX_12,
        .txChannelIndex         = UDMA_SEC_CHANNEL_UART2TX_13,
        .channelMappingFxn      = uDMAChannelAssign,
        .rxChannelMappingFxnArg = UDMA_CH12_SSI2RX,
        .txChannelMappingFxnArg = UDMA_CH13_SSI2TX
    },
    {
        .baseAddr               = SSI3_BASE,
        .intNum                 = INT_SSI3,
        .intPriority            = (~0),
        .scratchBufPtr          = &spiTivaDMAscratchBuf[1],
        .defaultTxBufValue      = 0,
        .rxChannelIndex         = UDMA_SEC_CHANNEL_TMR2A_14,
        .txChannelIndex         = UDMA_SEC_CHANNEL_TMR2B_15,
        .channelMappingFxn      = uDMAChannelAssign,
        .rxChannelMappingFxnArg = UDMA_CH14_SSI3RX,
        .txChannelMappingFxnArg = UDMA_CH15_SSI3TX
    }
};

const SPI_Config SPI_config[] = {
    {
        .fxnTablePtr = &SPITivaDMA_fxnTable,
        .object      = &spiTivaDMAObjects[0],
        .hwAttrs     = &spiTivaDMAHWAttrs[0]
    },
    {
        .fxnTablePtr = &SPITivaDMA_fxnTable,
        .object      = &spiTivaDMAObjects[1],
        .hwAttrs     = &spiTivaDMAHWAttrs[1]
    },
    {NULL, NULL, NULL}
};

/*
 *  ======== PMX42_initSPI ========
 */
void PMX42_initSPI(void)
{
	/* Enable SSI2 and SSI3 peripheral devices */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);

	/*
     * SSI2 (I/O Slots 1 & 2)
     */

	// Enable pin PD3 for SSI2 SSI2CLK
	GPIOPinConfigure(GPIO_PD3_SSI2CLK);
	GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_3);

#if 0
	/* THESE ARE FOR QUAD SPEED SPI */
	// Enable pin PD6 for SSI2 SSI2XDAT3
	GPIOPinConfigure(GPIO_PD6_SSI2XDAT3);
	GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_6);

	// Enable pin PD7 for SSI2 SSI2XDAT2
	// First open the lock and select the bits we want to modify in the GPIO commit register.
	HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTD_BASE + GPIO_O_CR) = 0x80;

	// Now modify the configuration of the pins that we unlocked.
	GPIOPinConfigure(GPIO_PD7_SSI2XDAT2);
	GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_7);
#endif
	// Enable pin PD1 for SSI2 SSI2XDAT0
	GPIOPinConfigure(GPIO_PD1_SSI2XDAT0);
	GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_1);

	// Enable pin PD0 for SSI2 SSI2XDAT1
	GPIOPinConfigure(GPIO_PD0_SSI2XDAT1);
	GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_0);

    /*
     * SSI3 (I/O Slots 3 & 4)
     */

    // Enable pin PQ3 for SSI3 SSI3XDAT1
    GPIOPinConfigure(GPIO_PQ3_SSI3XDAT1);
    GPIOPinTypeSSI(GPIO_PORTQ_BASE, GPIO_PIN_3);

    // Enable pin PQ2 for SSI3 SSI3XDAT0
    GPIOPinConfigure(GPIO_PQ2_SSI3XDAT0);
    GPIOPinTypeSSI(GPIO_PORTQ_BASE, GPIO_PIN_2);

#if 0
	/* THESE ARE FOR QUAD SPEED SPI */
    // Enable pin PP0 for SSI3 SSI3XDAT2
    GPIOPinConfigure(GPIO_PP0_SSI3XDAT2);
    GPIOPinTypeSSI(GPIO_PORTP_BASE, GPIO_PIN_0);

    // Enable pin PP1 for SSI3 SSI3XDAT3
    GPIOPinConfigure(GPIO_PP1_SSI3XDAT3);
    GPIOPinTypeSSI(GPIO_PORTP_BASE, GPIO_PIN_1);
#endif

    // Enable pin PQ0 for SSI3 SSI3CLK
    GPIOPinConfigure(GPIO_PQ0_SSI3CLK);
    GPIOPinTypeSSI(GPIO_PORTQ_BASE, GPIO_PIN_0);

    PMX42_initDMA();
    SPI_init();
}


/*
 *  =============================== UART ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(UART_config, ".const:UART_config")
#pragma DATA_SECTION(uartTivaHWAttrs, ".const:uartTivaHWAttrs")
#endif

#include <ti/drivers/UART.h>
#if TI_DRIVERS_UART_DMA
#include <ti/drivers/uart/UARTTivaDMA.h>

UARTTivaDMA_Object uartTivaObjects[PMX42_UARTCOUNT];

const UARTTivaDMA_HWAttrs uartTivaHWAttrs[PMX42_UARTCOUNT] = {
    {
        .baseAddr       = UART0_BASE,
        .intNum         = INT_UART0,
        .intPriority    = (~0),
        .rxChannelIndex = UDMA_CH8_UART0RX,
        .txChannelIndex = UDMA_CH9_UART0TX,
    },
    {
        .baseAddr       = UART3_BASE,
        .intNum         = INT_UART3,
        .intPriority    = (~0),
        .rxChannelIndex = UDMA_CH16_UART3RX,
        .txChannelIndex = UDMA_CH16_UART3TX,
    },
    {
        .baseAddr       = UART5_BASE,
        .intNum         = INT_UART5,
        .intPriority    = (~0),
        .rxChannelIndex = UDMA_CH6_UART5RX,
        .txChannelIndex = UDMA_CH7_UART5TX,
    },
    {
        .baseAddr       = UART7_BASE,
        .intNum         = INT_UART7,
        .intPriority    = (~0),
        .rxChannelIndex = UDMA_CH20_UART7RX,
        .txChannelIndex = UDMA_CH21_UART7TX,
    }
};

const UART_Config UART_config[] = {
    {
        .fxnTablePtr = &UARTTivaDMA_fxnTable,
        .object      = &uartTivaObjects[0],
        .hwAttrs     = &uartTivaHWAttrs[0]
    },
    {
        .fxnTablePtr = &UARTTivaDMA_fxnTable,
        .object      = &uartTivaObjects[1],
        .hwAttrs     = &uartTivaHWAttrs[1]
    },
    {
        .fxnTablePtr = &UARTTivaDMA_fxnTable,
        .object      = &uartTivaObjects[2],
        .hwAttrs     = &uartTivaHWAttrs[2]
    },
    {
        .fxnTablePtr = &UARTTivaDMA_fxnTable,
        .object      = &uartTivaObjects[3],
        .hwAttrs     = &uartTivaHWAttrs[3]
    },
    {NULL, NULL, NULL}
};
#else
#include <ti/drivers/uart/UARTTiva.h>

UARTTiva_Object uartTivaObjects[PMX42_UARTCOUNT];
unsigned char uartTivaRingBuffer[PMX42_UARTCOUNT][32];

/* UART configuration structure */
const UARTTiva_HWAttrs uartTivaHWAttrs[PMX42_UARTCOUNT] = {
    {
        .baseAddr    = UART0_BASE,
        .intNum      = INT_UART0,
        .intPriority = (~0),
        .flowControl = UART_FLOWCONTROL_NONE,
        .ringBufPtr  = uartTivaRingBuffer[0],
        .ringBufSize = sizeof(uartTivaRingBuffer[0])
    },
    {
        .baseAddr    = UART3_BASE,
        .intNum      = INT_UART3,
        .intPriority = (~0),
        .flowControl = UART_FLOWCONTROL_NONE,
        .ringBufPtr  = uartTivaRingBuffer[1],
        .ringBufSize = sizeof(uartTivaRingBuffer[1])
    },
    {
        .baseAddr    = UART5_BASE,
        .intNum      = INT_UART5,
        .intPriority = (~0),
        .flowControl = UART_FLOWCONTROL_NONE,
        .ringBufPtr  = uartTivaRingBuffer[2],
        .ringBufSize = sizeof(uartTivaRingBuffer[2])
    },
    {
        .baseAddr    = UART7_BASE,
        .intNum      = INT_UART7,
        .intPriority = (~0),
        .flowControl = UART_FLOWCONTROL_NONE,
        .ringBufPtr  = uartTivaRingBuffer[3],
        .ringBufSize = sizeof(uartTivaRingBuffer[3])
    }
};

const UART_Config UART_config[] = {
    {
        .fxnTablePtr = &UARTTiva_fxnTable,
        .object      = &uartTivaObjects[0],
        .hwAttrs     = &uartTivaHWAttrs[0]
    },
    {
        .fxnTablePtr = &UARTTiva_fxnTable,
        .object      = &uartTivaObjects[1],
        .hwAttrs     = &uartTivaHWAttrs[1]
    },
    {
        .fxnTablePtr = &UARTTiva_fxnTable,
        .object      = &uartTivaObjects[2],
        .hwAttrs     = &uartTivaHWAttrs[2]
    },
    {
        .fxnTablePtr = &UARTTiva_fxnTable,
        .object      = &uartTivaObjects[3],
        .hwAttrs     = &uartTivaHWAttrs[3]
    },
    {NULL, NULL, NULL}
};
#endif /* TI_DRIVERS_UART_DMA */

/*
 *  ======== PMX42_initUART ========
 */
void PMX42_initUART(void)
{
	/* Enable UART Peripherals */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART3);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART5);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);

	// Enable pin PH0 for UART0
	GPIOPinConfigure(GPIO_PH0_U0RTS);
	GPIOPinConfigure(GPIO_PH1_U0CTS);
	GPIOPinTypeUART(GPIO_PORTH_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Enable pin PA0 for UART0
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Enable pin PJ0 for UART3
	GPIOPinConfigure(GPIO_PJ0_U3RX);
	GPIOPinConfigure(GPIO_PJ1_U3TX);
	GPIOPinTypeUART(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	// Enable pin PP4 for UART3
	GPIOPinConfigure(GPIO_PP4_U3RTS);
	GPIOPinConfigure(GPIO_PP5_U3CTS);
	GPIOPinTypeUART(GPIO_PORTP_BASE, GPIO_PIN_4 | GPIO_PIN_5);

	// Enable pin PC7 for UART5
	GPIOPinConfigure(GPIO_PC7_U5TX);
	GPIOPinConfigure(GPIO_PC4_U7RX);
	GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_7 | GPIO_PIN_4);

	// Enable pin PC5 for UART7
	GPIOPinConfigure(GPIO_PC5_U7TX);
	GPIOPinConfigure(GPIO_PC6_U5RX);
	GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_6);

    /* Initialize the UART driver */
#if TI_DRIVERS_UART_DMA
    PMX42_initDMA();
#endif
    UART_init();
}

/*
 *  =============================== USB ===============================
 */
/*
 *  ======== PMX42_initUSB ========
 *  This function just turns on the USB
 */
void PMX42_initUSB(PMX42_USBMode usbMode)
{
    Error_Block eb;
    Hwi_Params  hwiParams;

    /* Enable the USB peripheral and PLL */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_USB0);
    SysCtlUSBPLLEnable();

    /* Setup pins for USB operation */
    GPIOPinTypeUSBAnalog(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinTypeUSBAnalog(GPIO_PORTL_BASE, GPIO_PIN_6 | GPIO_PIN_7);

    /* Additional configurations for Host mode */
    if (usbMode == PMX42_USBHOST) {
        /* Configure the pins needed */
        HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
        HWREG(GPIO_PORTD_BASE + GPIO_O_CR) = 0xff;
        GPIOPinConfigure(GPIO_PD6_USB0EPEN);
        GPIOPinTypeUSBDigital(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);

        /*
         *  USB bus fault is routed to pin PQ4.  We create a Hwi to allow us
         *  to detect power faults and recover gracefully or terminate the
         *  program.  PQ4 is active low; set the pin as input with a weak
         *  pull-up.
         */
        GPIOPadConfigSet(GPIO_PORTQ_BASE, GPIO_PIN_4,
                         GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
        GPIOIntTypeSet(GPIO_PORTQ_BASE, GPIO_PIN_4, GPIO_FALLING_EDGE);
        GPIOIntClear(GPIO_PORTQ_BASE, GPIO_PIN_4);

        /* Create a Hwi for PQ4 pin. */
        Error_init(&eb);
        Hwi_Params_init(&hwiParams);
        Hwi_construct(&(usbBusFaultHwiStruct), INT_GPIOQ4,
                      PMX42_usbBusFaultHwi, &hwiParams, &eb);
        if (Error_check(&eb)) {
            System_abort("Couldn't construct USB bus fault hwi");
        }
    }
}

/*
 *  =============================== USBMSCHFatFs ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(USBMSCHFatFs_config, ".const:USBMSCHFatFs_config")
#pragma DATA_SECTION(usbmschfatfstivaHWAttrs, ".const:usbmschfatfstivaHWAttrs")
#endif

#include <ti/drivers/USBMSCHFatFs.h>
#include <ti/drivers/usbmschfatfs/USBMSCHFatFsTiva.h>

USBMSCHFatFsTiva_Object usbmschfatfstivaObjects[PMX42_USBMSCHFatFsCOUNT];

const USBMSCHFatFsTiva_HWAttrs usbmschfatfstivaHWAttrs[PMX42_USBMSCHFatFsCOUNT] = {
    {
        .intNum      = INT_USB0,
        .intPriority = (~0)
    }
};

const USBMSCHFatFs_Config USBMSCHFatFs_config[] = {
    {
        .fxnTablePtr = &USBMSCHFatFsTiva_fxnTable,
        .object      = &usbmschfatfstivaObjects[0],
        .hwAttrs     = &usbmschfatfstivaHWAttrs[0]
    },
    {NULL, NULL, NULL}
};

/*
 *  ======== PMX42_initUSBMSCHFatFs ========
 */
void PMX42_initUSBMSCHFatFs(void)
{
    /* Initialize the DMA control table */
    PMX42_initDMA();

    /* Call the USB initialization function for the USB Reference modules */
    PMX42_initUSB(PMX42_USBHOST);
    USBMSCHFatFs_init();
}

/*
 *  =============================== Watchdog ===============================
 */
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(Watchdog_config, ".const:Watchdog_config")
#pragma DATA_SECTION(watchdogTivaHWAttrs, ".const:watchdogTivaHWAttrs")
#endif

#include <ti/drivers/Watchdog.h>
#include <ti/drivers/watchdog/WatchdogTiva.h>

WatchdogTiva_Object watchdogTivaObjects[PMX42_WATCHDOGCOUNT];

const WatchdogTiva_HWAttrs watchdogTivaHWAttrs[PMX42_WATCHDOGCOUNT] = {
    {
        .baseAddr    = WATCHDOG0_BASE,
        .intNum      = INT_WATCHDOG,
        .intPriority = (~0),
        .reloadValue = 80000000 // 1 second period at default CPU clock freq
    },
};

const Watchdog_Config Watchdog_config[] = {
    {
        .fxnTablePtr = &WatchdogTiva_fxnTable,
        .object      = &watchdogTivaObjects[0],
        .hwAttrs     = &watchdogTivaHWAttrs[0]
    },
    {NULL, NULL, NULL},
};

/*
 *  ======== PMX42_initWatchdog ========
 *
 * NOTE: To use the other watchdog timer with base address WATCHDOG1_BASE,
 *       an additional function call may need be made to enable PIOSC. Enabling
 *       WDOG1 does not do this. Enabling another peripheral that uses PIOSC
 *       such as ADC0 or SSI0, however, will do so. Example:
 *
 *       SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
 *       SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG1);
 *
 *       See the following forum post for more information:
 *       http://e2e.ti.com/support/microcontrollers/stellaris_arm_cortex-m3_microcontroller/f/471/p/176487/654390.aspx#654390
 */
void PMX42_initWatchdog(void)
{
    /* Enable peripherals used by Watchdog */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_WDOG0);

    /* Initialize the Watchdog driver */
    Watchdog_init();
}

/*
 *  =============================== WiFi ===============================
 */
#if TI_DRIVERS_WIFI_INCLUDED
/* Place into subsections to allow the TI linker to remove items properly */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_SECTION(WiFi_config, ".const:WiFi_config")
#pragma DATA_SECTION(wiFiCC3100HWAttrs, ".const:wiFiCC3100HWAttrs")
#endif

#include <ti/drivers/WiFi.h>
#include <ti/drivers/wifi/WiFiCC3100.h>

WiFiCC3100_Object wiFiCC3100Objects[EK_TM4C1294XL_WIFICOUNT];

const WiFiCC3100_HWAttrs wiFiCC3100HWAttrs[EK_TM4C1294XL_WIFICOUNT] = {
    {
        .irqPort   = GPIO_PORTM_BASE,
        .irqPin    = GPIO_PIN_3,
        .irqIntNum = INT_GPIOM,
        .csPort    = GPIO_PORTH_BASE,
        .csPin     = GPIO_PIN_2,
        .enPort    = GPIO_PORTC_BASE,
        .enPin     = GPIO_PIN_6
    }
};

const WiFi_Config WiFi_config[] = {
    {
        .fxnTablePtr = &WiFiCC3100_fxnTable,
        .object      = &wiFiCC3100Objects[0],
        .hwAttrs     = &wiFiCC3100HWAttrs[0]
    },
    {NULL,NULL, NULL},
};

/*
 *  ======== PMX42_initWiFi ========
 */
void PMX42_initWiFi(void)
{
    /* Configure EN & CS pins to disable CC3100 */
    GPIOPinTypeGPIOOutput(GPIO_PORTH_BASE, GPIO_PIN_2);
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_6);
    GPIOPinWrite(GPIO_PORTH_BASE, GPIO_PIN_2, GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0);

    /* Configure SSI2 for CC3100 */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
    GPIOPinConfigure(GPIO_PD3_SSI2CLK);
    GPIOPinConfigure(GPIO_PD1_SSI2XDAT0);
    GPIOPinConfigure(GPIO_PD0_SSI2XDAT1);
    GPIOPinTypeSSI(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_3);

    /* Configure IRQ pin */
    GPIOPinTypeGPIOInput(GPIO_PORTM_BASE, GPIO_PIN_3);
    GPIOPadConfigSet(GPIO_PORTM_BASE, GPIO_PIN_3, GPIO_STRENGTH_2MA,
                     GPIO_PIN_TYPE_STD_WPD);
    GPIOIntTypeSet(GPIO_PORTM_BASE, GPIO_PIN_3, GPIO_RISING_EDGE);

    SPI_init();
    PMX42_initDMA();

    WiFi_init();
}
#endif /* TI_DRIVERS_WIFI_INCLUDED */
