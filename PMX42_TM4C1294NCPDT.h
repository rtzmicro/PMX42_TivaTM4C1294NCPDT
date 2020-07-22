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
/** ============================================================================
 *  @file       PMX42.h
 *
 *  @brief      PMX42 Board Specific APIs
 *
 *  The PMX42 header file should be included in an application as follows:
 *  @code
 *  #include <PMX42.h>
 *  @endcode
 *
 *  ============================================================================
 */

#ifndef __PMX42_TM4C1294NCPDT_H
#define __PMX42_TM4C1294NCPDT_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ti/drivers/GPIO.h>

/* LEDs on PMX42 are active high. */
#define PMX42_LED_OFF	( 0)
#define PMX42_LED_ON	(~0)

#define PIN_LOW			( 0)
#define PIN_HIGH		(~0)

/* Board specific I2C addresses */
#define AT24MAC_EPROM_ADDR      (0xA0 >> 1)
#define AT24MAC_EPROM_EXT_ADDR  (0xB0 >> 1)

/* Push button switch bits on Port-N */
#define B_BTN_SW1		0x01
#define B_BTN_SW2		0x02
#define B_BTN_SW3		0x04
#define B_BTN_SW4		0x08
#define B_BTN_SW5		0x10
#define B_BTN_SW6		0x20

/*!
 *  @def    PMX42_EMACName
 *  @brief  Enum of EMAC names on the PMX42 dev board
 */
typedef enum PMX42_EMACName {
    PMX42_EMAC0 = 0,

    PMX42_EMACCOUNT
} PMX42_EMACName;

/*!
 *  @def    PMX42_GPIOName
 *  @brief  Enum of LED names on the PMX42 dev board
 */
typedef enum PMX42_GPIOName {
    /* Input IRQ's */
    PMX42_BTN_SW1 = 0,
    PMX42_BTN_SW2,
    PMX42_BTN_SW3,
    PMX42_BTN_SW4,
    PMX42_BTN_SW5,
    PMX42_BTN_SW6,
    PMX42_PL2_SLOT1_IRQ,
    PMX42_PL3_SLOT2_IRQ,
    PMX42_PL4_SLOT3_IRQ,
    PMX42_PL5_SLOT4_IRQ,
    /* LED outputs */
    PMX42_STAT_LED1,
    PMX42_STAT_LED2,
    /* SLOT-1 */
	PMX42_PD2_SLOT1_SS,
	PMX42_PM4_T4CCP0,
    PMX42_PM5_T4CCP1,
    PMX42_PM6_T5CCP0,
    /* SLOT-2 */
    PMX42_PM2_SLOT2_SS,
	PMX42_PF1_M0PWM1,
	PMX42_PF2_M0PWM2,
    PMX42_PF3_M0PWM3,
    /* SLOT-3 */
    PMX42_PQ1_SLOT3_SS,
    /* SLOT-4 */
    PMX42_PM3_SLOT4_SS,

    PMX42_GPIOCOUNT
} PMX42_GPIOName;





/*!
 *  @def    PMX42_I2CName
 *  @brief  Enum of I2C names on the PMX42 dev board
 */
typedef enum PMX42_I2CName {
    PMX42_I2C0 = 0,
    PMX42_I2C1,
    PMX42_I2C2,
    PMX42_I2C3,

    PMX42_I2CCOUNT
} PMX42_I2CName;

/*!
 *  @def    PMX42_PWMName
 *  @brief  Enum of PWM names on the PMX42 dev board
 */
typedef enum PMX42_PWMName {
    PMX42_PWM0 = 0,

    PMX42_PWMCOUNT
} PMX42_PWMName;

/*!
 *  @def    PMX42_SDSPIName
 *  @brief  Enum of SDSPI names on the PMX42 dev board
 */
typedef enum PMX42_SDSPIName {
    PMX42_SDSPI0 = 0,		/* SD Card Socket */
    PMX42_SDSPI1,			/* U206 OnBoard Flash */

    PMX42_SDSPICOUNT
} PMX42_SDSPIName;

/*!
 *  @def    PMX42_SPIName
 *  @brief  Enum of SPI names on the PMX42 dev board
 */
typedef enum PMX42_SPIName {
    PMX42_SPI2 = 0,
    PMX42_SPI3,

    PMX42_SPICOUNT
} PMX42_SPIName;

/*!
 *  @def    PMX42_UARTName
 *  @brief  Enum of UARTs on the PMX42 dev board
 */
typedef enum PMX42_UARTName {
    PMX42_UART0 = 0,
    PMX42_UART3,
    PMX42_UART5,
    PMX42_UART7,

    PMX42_UARTCOUNT
} PMX42_UARTName;

/*!
 *  @def    PMX42_USBMode
 *  @brief  Enum of USB setup function on the PMX42 dev board
 */
typedef enum PMX42_USBMode {
    PMX42_USBDEVICE,
    PMX42_USBHOST
} PMX42_USBMode;

/*!
 *  @def    PMX42_USBMSCHFatFsName
 *  @brief  Enum of USBMSCHFatFs names on the PMX42 dev board
 */
typedef enum PMX42_USBMSCHFatFsName {
    PMX42_USBMSCHFatFs0 = 0,

    PMX42_USBMSCHFatFsCOUNT
} PMX42_USBMSCHFatFsName;

/*
 *  @def    PMX42_WatchdogName
 *  @brief  Enum of Watchdogs on the PMX42 dev board
 */
typedef enum PMX42_WatchdogName {
    PMX42_WATCHDOG0 = 0,

    PMX42_WATCHDOGCOUNT
} PMX42_WatchdogName;

/*!
 *  @def    PMX42_WiFiName
 *  @brief  Enum of WiFi names on the PMX42 dev board
 */
typedef enum PMX42_WiFiName {
    PMX42_WIFI = 0,

    PMX42_WIFICOUNT
} PMX42_WiFiName;

/*!
 *  @brief  Initialize the general board specific settings
 *
 *  This function initializes the general board specific settings. This include
 *     - Enable clock sources for peripherals
 */
extern void PMX42_initGeneral(void);

/*!
 *  @brief Initialize board specific EMAC settings
 *
 *  This function initializes the board specific EMAC settings and
 *  then calls the EMAC_init API to initialize the EMAC module.
 *
 *  The EMAC address is programmed as part of this call.
 *
 */
extern void PMX42_initEMAC(void);

/*!
 *  @brief  Initialize board specific GPIO settings
 *
 *  This function initializes the board specific GPIO settings and
 *  then calls the GPIO_init API to initialize the GPIO module.
 *
 *  The GPIOs controlled by the GPIO module are determined by the GPIO_config
 *  variable.
 */
extern void PMX42_initGPIO(void);

/*!
 *  @brief  Initialize board specific I2C settings
 *
 *  This function initializes the board specific I2C settings and then calls
 *  the I2C_init API to initialize the I2C module.
 *
 *  The I2C peripherals controlled by the I2C module are determined by the
 *  I2C_config variable.
 */
extern void PMX42_initI2C(void);

/*!
 *  @brief  Initialize board specific PWM settings
 *
 *  This function initializes the board specific PWM settings and then calls
 *  the PWM_init API to initialize the PWM module.
 *
 *  The PWM peripherals controlled by the PWM module are determined by the
 *  PWM_config variable.
 */
extern void PMX42_initPWM(void);

/*!
 *  @brief  Initialize board specific SDSPI settings
 *
 *  This function initializes the board specific SDSPI settings and then calls
 *  the SDSPI_init API to initialize the SDSPI module.
 *
 *  The SDSPI peripherals controlled by the SDSPI module are determined by the
 *  SDSPI_config variable.
 */
extern void PMX42_initSDSPI(void);

/*!
 *  @brief  Initialize board specific SPI settings
 *
 *  This function initializes the board specific SPI settings and then calls
 *  the SPI_init API to initialize the SPI module.
 *
 *  The SPI peripherals controlled by the SPI module are determined by the
 *  SPI_config variable.
 */
extern void PMX42_initSPI(void);

/*!
 *  @brief  Initialize board specific UART settings
 *
 *  This function initializes the board specific UART settings and then calls
 *  the UART_init API to initialize the UART module.
 *
 *  The UART peripherals controlled by the UART module are determined by the
 *  UART_config variable.
 */
extern void PMX42_initUART(void);

/*!
 *  @brief  Initialize board specific USB settings
 *
 *  This function initializes the board specific USB settings and pins based on
 *  the USB mode of operation.
 *
 *  @param      usbMode    USB mode of operation
 */
extern void PMX42_initUSB(PMX42_USBMode usbMode);

/*!
 *  @brief  Initialize board specific USBMSCHFatFs settings
 *
 *  This function initializes the board specific USBMSCHFatFs settings and then
 *  calls the USBMSCHFatFs_init API to initialize the USBMSCHFatFs module.
 *
 *  The USBMSCHFatFs peripherals controlled by the USBMSCHFatFs module are
 *  determined by the USBMSCHFatFs_config variable.
 */
extern void PMX42_initUSBMSCHFatFs(void);

/*!
 *  @brief  Initialize board specific Watchdog settings
 *
 *  This function initializes the board specific Watchdog settings and then
 *  calls the Watchdog_init API to initialize the Watchdog module.
 *
 *  The Watchdog peripherals controlled by the Watchdog module are determined
 *  by the Watchdog_config variable.
 */
extern void PMX42_initWatchdog(void);

/*!
 *  @brief  Initialize board specific WiFi settings
 *
 *  This function initializes the board specific WiFi settings and then calls
 *  the WiFi_init API to initialize the WiFi module.
 *
 *  The hardware resources controlled by the WiFi module are determined by the
 *  WiFi_config variable.
 */
extern void PMX42_initWiFi(void);

#ifdef __cplusplus
}
#endif

#endif /* __PMX42_TM4C1294NCPDT_H */
