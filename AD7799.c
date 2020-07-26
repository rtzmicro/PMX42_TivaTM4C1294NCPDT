/***************************************************************************//**
 *   @file   AD7799.c
 *   @brief  Implementation of AD7799 Driver.
 *   @author Bancisor MIhai
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: 577
*******************************************************************************/

/******************************************************************************/
/* Include Files                                                              */
/******************************************************************************/

/* TI-RTOS Kernel Header files */
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* TI-RTOS Kernel Header files */
#include <xdc/std.h>
#include <xdc/runtime/Assert.h>
#include <xdc/runtime/Diags.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/Log.h>
#include <xdc/runtime/Memory.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Driver Header files */
#include <ti/drivers/SPI.h>
#include <ti/drivers/GPIO.h>


#include "Board.h"
#include "PMX42.h"
#include "AD7799.h"             // AD7799 definitions.

/* Default AD7799DB parameters structure */
const AD7799_Params AD7799_defaultParams = {
    0,   /* dummy */
};

/*** Static Function Prototypes ***/

static Void AD7799_destruct(AD7799_Handle handle);
static void AD7799_SetRegisterValue(AD7799_Handle handle, uint8_t regAddress, uint32_t regValue, uint8_t size);
static uint32_t AD7799_GetRegisterValue(AD7799_Handle handle, uint8_t regAddress, uint8_t size);

/*
 *  ======== AD7799_construct ========
 */
AD7799_Handle AD7799_construct(
        AD7799_Object *obj,
        SPI_Handle spiHandle,
        uint32_t gpioCSIndex,
        uint32_t gpioRDYIndex,
        AD7799_Params *params)
{
    /* Initialize the object's fields */
    obj->spiHandle = spiHandle;
    obj->gpioCS    = gpioCSIndex;
    obj->gpioRDY   = gpioRDYIndex;

    GateMutex_construct(&(obj->gate), NULL);

    return (AD7799_Handle)obj;
}

/*
 *  ======== AD7799_create ========
 */
AD7799_Handle AD7799_create(
        SPI_Handle spiHandle,
        uint32_t gpioCSIndex,
        uint32_t gpioRDYIndex,
        AD7799_Params *params)
{
    AD7799_Handle handle;
    Error_Block eb;

    Error_init(&eb);

    handle = Memory_alloc(NULL, sizeof(AD7799_Object), NULL, &eb);

    if (handle == NULL) {
        return (NULL);
    }

    handle = AD7799_construct(handle, spiHandle, gpioCSIndex, gpioRDYIndex, params);

    return handle;
}

/*
 *  ======== AD7799_delete ========
 */
Void AD7799_delete(AD7799_Handle handle)
{
    AD7799_destruct(handle);

    Memory_free(NULL, handle, sizeof(AD7799_Object));
}

/*
 *  ======== AD7799_destruct ========
 */
Void AD7799_destruct(AD7799_Handle handle)
{
    Assert_isTrue((handle != NULL), NULL);

    GateMutex_destruct(&(handle->gate));
}

Void AD7799_Params_init(AD7799_Params *params)
{
    Assert_isTrue(params != NULL, NULL);

    *params = AD7799_defaultParams;
}

/***************************************************************************//**
 * @brief Reads the value of the selected register
 *
 * @param regAddress - The address of the register to read.
 * @param size - The size of the register to read.
 *
 * @return data - The value of the selected register register.
*******************************************************************************/

uint32_t AD7799_GetRegisterValue(
        AD7799_Handle handle,
        uint8_t regAddress,
        uint8_t size
        )
{
    uint8_t txBuf[4];
    uint8_t rxBuf[4];
    uint32_t i;
    uint32_t regval = 0;
    SPI_Transaction transaction;

    memset(txBuf, 0, sizeof(txBuf));

    /* Assert the chip select low */
    GPIO_write(handle->gpioCS, PIN_LOW);

    /*
     * Write the command byte and register address
     */

    txBuf[0] = AD7799_COMM_READ | AD7799_COMM_ADDR(regAddress);

    transaction.count = 1;
    transaction.txBuf = (Ptr)txBuf;
    transaction.rxBuf = (Ptr)rxBuf;

    /* Initiate SPI transfer */
    SPI_transfer(handle->spiHandle, &transaction);

    /* Wait for data ready RDY */
    while (GPIO_read(handle->gpioRDY) == 0);

    /*
     * Now read back any response data
     */

    memset(rxBuf, 0, sizeof(rxBuf));

    for (i=0; i < size; i++)
    {
        transaction.count = 1;
        transaction.txBuf = (Ptr)&txBuf[i];
        transaction.rxBuf = (Ptr)&rxBuf[i];

        /* Initiate SPI transfer */
        SPI_transfer(handle->spiHandle, &transaction);

        /* Wait for data ready RDY */
        while (GPIO_read(handle->gpioRDY) == 0);
    }

    /* Release chip select to high */
    GPIO_write(handle->gpioCS, PIN_HIGH);

    /* Extract the data value returned */

    if (size == 1)
    {
        regval += (rxBuf[0] << 0);
    }
    else if(size == 2)
    {
        regval += (rxBuf[0] << 8);
        regval += (rxBuf[1] << 0);
    }
    else if(size == 3)
    {
        regval += ((unsigned long)rxBuf[0] << 16);
        regval += (rxBuf[1] << 8);
        regval += (rxBuf[2] << 0);
    }

    return regval;
}

/***************************************************************************//**
 * @brief Writes the value to the register
 *
 * @param -  regAddress - The address of the register to write to.
 * @param -  regValue - The value to write to the register.
 * @param -  size - The size of the register to write.
 *
 * @return  None.
*******************************************************************************/

void AD7799_SetRegisterValue(
        AD7799_Handle handle,
        uint8_t regAddress,
        uint32_t regValue,
        uint8_t size
        )
{
    uint8_t txBuf[5];
    uint8_t rxBuf[5];
    uint32_t i;
    SPI_Transaction transaction;

    memset(txBuf, 0, sizeof(txBuf));

    /* Assert the chip select low */
    GPIO_write(handle->gpioCS, PIN_LOW);

    transaction.count = 1;
    transaction.txBuf = (Ptr)&txBuf;
    transaction.rxBuf = (Ptr)&rxBuf;

    txBuf[0] = AD7799_COMM_WRITE | AD7799_COMM_ADDR(regAddress);

    /* Initiate SPI transfer */
    SPI_transfer(handle->spiHandle, &transaction);

    /* Wait for data ready RDY */
    while (GPIO_read(handle->gpioRDY) == 0);

    /*
     * Now write any register data
     */

    memset(rxBuf, 0, sizeof(rxBuf));

    /* Format the register data value to write */

    if (size == 1)
    {
        txBuf[1] = (uint8_t)regValue;
    }
    else if (size == 2)
    {
        txBuf[1] = (uint8_t)(regValue >> 8);
        txBuf[2] = (uint8_t)regValue;
    }
    else if (size == 3)
    {
        txBuf[1] = (uint8_t)(regValue >> 16);
        txBuf[2] = (uint8_t)(regValue >> 8);
        txBuf[3] = (uint8_t)regValue;
    }

    for(i=0; i < size; i++)
    {
        transaction.count = size;
        transaction.txBuf = (Ptr)&txBuf[i];
        transaction.rxBuf = (Ptr)&rxBuf[i];

        /* Initiate SPI transfer */
        SPI_transfer(handle->spiHandle, &transaction);

        /* Wait for data ready RDY */
        while (GPIO_read(handle->gpioRDY) == 0);
    }

    /* Release chip select to high */
    GPIO_write(handle->gpioCS, PIN_HIGH);
}

/***************************************************************************//**
 * @brief Reads /RDY bit of status reg.
 *
 * @param None.
 *
 * @return rdy  - 0 if RDY is 1.
 *              - 1 if RDY is 0.
*******************************************************************************/

uint8_t AD7799_IsReady(AD7799_Handle handle)
{
    uint8_t rdy = 0;
    IArg key;

    /* Enter the critical section */
    key = GateMutex_enter(GateMutex_handle(&(handle->gate)));

    rdy = AD7799_GetRegisterValue(handle, AD7799_REG_STAT, 1) & AD7799_STAT_RDY;

    /* Exit the critical section */
    GateMutex_leave(GateMutex_handle(&(handle->gate)), key);

    return(!rdy);
}

/***************************************************************************//**
 * @brief Sends 32 consecutive 1's on SPI in order to reset the part.
 *
 * @param None.
 *
 * @return  None.
*******************************************************************************/

void AD7799_Reset(AD7799_Handle handle)
{
    IArg key;
    SPI_Transaction transaction;
    uint32_t txBuf = 0xFFFFFFFF;
    uint32_t rxBuf;

    /* Initialize opcode transaction structure */
    transaction.count = 4;
    transaction.txBuf = (Ptr)&txBuf;
    transaction.rxBuf = (Ptr)&rxBuf;

    /* Enter the critical section */
    key = GateMutex_enter(GateMutex_handle(&(handle->gate)));

    /* Assert the chip select low */
    GPIO_write(handle->gpioCS, PIN_LOW);

    /* Initiate SPI transfer */
    SPI_transfer(handle->spiHandle, &transaction);

    /* Release chip select to high */
    GPIO_write(handle->gpioCS, PIN_HIGH);

    /* Exit the critical section */
    GateMutex_leave(GateMutex_handle(&(handle->gate)), key);

    /* Settling time after chip reset */
    Task_sleep(100);
}

/***************************************************************************//**
 * @brief Initializes the AD7799 and checks if the device is present.
 *
 * @param None.
 *
 * @return status - Result of the initialization procedure.
 *                  Example: 1 - if initialization was successful (ID is 0x0B).
 *                           0 - if initialization was unsuccessful.
*******************************************************************************/

uint8_t AD7799_Init(AD7799_Handle handle)
{
    uint32_t reg;
    uint8_t status = 0x1;
    IArg key;

    /* Enter the critical section */
    key = GateMutex_enter(GateMutex_handle(&(handle->gate)));

    reg = AD7799_GetRegisterValue(handle, AD7799_REG_ID, 1);

    if ((reg & 0x0F) != AD7799_ID)
    {
        status = 0x0;
    }

    /* Exit the critical section */
    GateMutex_leave(GateMutex_handle(&(handle->gate)), key);

    return status;
}



/***************************************************************************//**
 * @brief Sets the operating mode of AD7799.
 *
 * @param mode - Mode of operation.
 *
 * @return  None.    
*******************************************************************************/

void AD7799_SetMode(AD7799_Handle handle, uint32_t mode)
{
    uint32_t command;
    IArg key;

    /* Enter the critical section */
    key = GateMutex_enter(GateMutex_handle(&(handle->gate)));

    command = AD7799_GetRegisterValue(handle, AD7799_REG_MODE, 2);

    command &= ~AD7799_MODE_SEL(0xFF);
    command |= AD7799_MODE_SEL(mode);

    AD7799_SetRegisterValue(handle, AD7799_REG_MODE, command, 2);

    /* Exit the critical section */
    GateMutex_leave(GateMutex_handle(&(handle->gate)), key);
}

/***************************************************************************//**
 * @brief Selects the channel of AD7799.
 *
 * @param  channel - ADC channel selection.
 *
 * @return  None.    
*******************************************************************************/

void AD7799_SetChannel(AD7799_Handle handle, uint32_t channel)
{
    uint32_t command;
    IArg key;

    /* Enter the critical section */
    key = GateMutex_enter(GateMutex_handle(&(handle->gate)));

    command = AD7799_GetRegisterValue(handle, AD7799_REG_CONF, 2);

    command &= ~AD7799_CONF_CHAN(0xFF);
    command |= AD7799_CONF_CHAN(channel);

    AD7799_SetRegisterValue(handle, AD7799_REG_CONF, command, 2);

    /* Exit the critical section */
    GateMutex_leave(GateMutex_handle(&(handle->gate)), key);
}

/***************************************************************************//**
 * @brief  Sets the gain of the In-Amp.
 *
 * @param  gain - Gain.
 *
 * @return  None.    
*******************************************************************************/

void AD7799_SetGain(AD7799_Handle handle, uint32_t gain)
{
    uint32_t command;
    IArg key;

    /* Enter the critical section */
    key = GateMutex_enter(GateMutex_handle(&(handle->gate)));

    command = AD7799_GetRegisterValue(handle, AD7799_REG_CONF, 2);

    command &= ~AD7799_CONF_GAIN(0xFF);
    command |= AD7799_CONF_GAIN(gain);

    AD7799_SetRegisterValue(handle, AD7799_REG_CONF, command, 2);

    /* Exit the critical section */
    GateMutex_leave(GateMutex_handle(&(handle->gate)), key);
}

/***************************************************************************//**
 * @brief Enables or disables the reference detect function.
 *
 * @param state - State of the reference detect function.
 *               Example: 0	- Reference detect disabled.
 *                        1	- Reference detect enabled.
 *
 * @return None.    
*******************************************************************************/

void AD7799_SetReference(AD7799_Handle handle, uint8_t state)
{
    uint32_t command = 0;
    IArg key;

    /* Enter the critical section */
    key = GateMutex_enter(GateMutex_handle(&(handle->gate)));

    command = AD7799_GetRegisterValue(handle, AD7799_REG_CONF, 2);

    command &= ~AD7799_CONF_REFDET(1);
    command |= AD7799_CONF_REFDET(state);

    AD7799_SetRegisterValue(handle, AD7799_REG_CONF, command, 2);

    /* Exit the critical section */
    GateMutex_leave(GateMutex_handle(&(handle->gate)), key);
}

/***************************************************************************//**
 * Reads status register.
 *******************************************************************************/

uint8_t AD7799_ReadStatus(AD7799_Handle handle)
{
    uint8_t status;
    IArg key;

    /* Enter the critical section */
    key = GateMutex_enter(GateMutex_handle(&(handle->gate)));

    status = AD7799_GetRegisterValue(handle, AD7799_REG_STAT, 1);

    /* Exit the critical section */
    GateMutex_leave(GateMutex_handle(&(handle->gate)), key);

    return status;
}

/******************************************************************************
 * Read the 24-bit data word register
 ******************************************************************************/

uint32_t AD7799_ReadData(
        AD7799_Handle handle,
        uint8_t channel
        )
{
    IArg key;
    uint32_t data;

    /* Select channel zero and read the data */
    //AD7799_SetChannel(handle, channel);

    /* Enter the critical section */
    key = GateMutex_enter(GateMutex_handle(&(handle->gate)));

    /* Read the 24-bit ADC data register */
    data = AD7799_GetRegisterValue(handle, AD7799_REG_DATA, 3);

    /* Exit the critical section */
    GateMutex_leave(GateMutex_handle(&(handle->gate)), key);

    return data;
}

/* End-Of-File */
