//*****************************************************************************
//
//
//
//
//*****************************************************************************

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

/* TI-RTOS Driver files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>

#include <inc/hw_ints.h>
#include <inc/hw_memmap.h>
#include <inc/hw_types.h>
#include <inc/hw_sysctl.h>
#include <inc/hw_gpio.h>

#include <driverlib/gpio.h>
#include <driverlib/pin_map.h>

#include <file.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>
#include <stdlib.h>
#include <math.h>

/* PMX42 Board Header file */
#include "Board.h"
#include "PMX42.h"
#include "AD7793.h"				// AD7793 definitions.

//#define BIPOLAR_24BIT	1

/* Static Data */
static SPI_Handle s_spiHandle = NULL;

/* Static Functions */
static void AD7793_ConfigureDevice(uint32_t chipSelect);
static int AD7793_Wait4Ready(uint32_t chipSelect);

//*****************************************************************************
// Description:	This initializes the SPI port to the AD7793 and checks if
// 				the device is is present. If so, the chips are reset and
//				configured for RTD measurement.
//
// Returns:		TRUE	- Success
//				FALSE	- Error
//*****************************************************************************

bool AD7793_Init(void)
{ 
	uint32_t data;
	bool success = true;
	SPI_Params	spiParams;

	/* De-assert both AD7793 chip selects */
	GPIO_write(Board_SLOT2_AD7793_CS1, PIN_HIGH);
	GPIO_write(Board_SLOT2_AD7793_CS2, PIN_HIGH);

	SPI_Params_init(&spiParams);

	spiParams.mode 			  = SPI_MASTER;
	spiParams.transferMode	  = SPI_MODE_BLOCKING;
	spiParams.transferTimeout = 1000;
	spiParams.frameFormat 	  = SPI_POL1_PHA1;
	spiParams.bitRate 		  = 1000000;			/* 1 Mhz for now */
	spiParams.dataSize 		  = 8;

	/* Slots 1 & 2 share quad-speed SPI2 bus */
	if ((s_spiHandle = SPI_open(Board_SPI2, &spiParams)) == NULL)
	{
		System_printf("Error: Unable to open SPI2 port\n");
		return 0;
	}

	/* The 53107 RTD board has two ADCs, reset them both */
	AD7793_Reset(Board_SLOT2_AD7793_CS1);
	AD7793_Reset(Board_SLOT2_AD7793_CS2);

	/* Config registers should be 0x0710 after reset */
    //data = AD7793_GetRegisterValue(Board_SLOT1_AD7793_SS1, AD7793_REG_CONF, 2);
    //data = AD7793_GetRegisterValue(Board_SLOT1_AD7793_SS2, AD7793_REG_CONF, 2);

	/* Read the ID register to make sure AD7793 #1 device is there */
	data = AD7793_GetRegisterValue(Board_SLOT2_AD7793_CS1, AD7793_REG_ID, 1);

	if ((data & AD7793_ID_MASK) != AD7793_ID)
	{
		System_printf("Error: AD7793 #1 missing %x\n", data);
		success = false;
	}

	/* Read the ID register to make sure AD7793 #2 device is there */
	data = AD7793_GetRegisterValue(Board_SLOT2_AD7793_CS2, AD7793_REG_ID, 1);

	if ((data & AD7793_ID_MASK) != AD7793_ID)
	{
		System_printf("Error: AD7793 #2 missing %x\n", data);
		success = false;
	}

	if (success != false)
	{
		/* Configure both AD7793's on the I/O card */
		AD7793_ConfigureDevice(Board_SLOT2_AD7793_CS1);
		AD7793_ConfigureDevice(Board_SLOT2_AD7793_CS2);

		System_printf("AD7793's Initialized!\n", data);
	}

	System_flush();

	return success;
}

//*****************************************************************************
// Configure the AD7793 chip for RTD measurement.
//*****************************************************************************

void AD7793_ConfigureDevice(uint32_t chipSelect)
{
	int i;
	uint32_t regval;

	/* Setup the mode register to use the internal
	 * clock source at 470 Hz sample rate and set
	 * the ADC in IDLE mode state.
	 */

	regval = AD7793_MODE_SEL(AD7793_MODE_IDLE) |
			 AD7793_MODE_CLKSRC(AD7793_CLK_INT) |
			 AD7793_MODE_RATE(1);

	AD7793_SetMode(chipSelect, regval);

	/* Set the ADC channel number to use
	 * channel AIN1(+) - AIN1(-).
	 */

	AD7793_SetChannel(chipSelect, AD7793_CH_AIN1P_AIN1M);

	/* Setup the Excitation Current Sources
	 * IEXC1 connects to IOUT1 and IEXC2 connect to IOUT2.
	 */

	regval = AD7793_IEXCDIR(AD7793_DIR_IEXC1_IOUT1_IEXC2_IOUT2) |
			 AD7793_IEXCEN(AD7793_EN_IXCEN_210uA);

	AD7793_SetRegisterValue(chipSelect, AD7793_REG_IO, regval, 1);

	/* Now setup the ADC configuration parameters to
	 * use the external reference with gain of 128.
	 */
#ifdef BIPOLAR_24BIT
	regval = AD7793_CONF_REFSEL(AD7793_REFSEL_EXT) |
			 AD7793_CONF_BUF |
			 AD7793_CONF_GAIN(AD7793_GAIN_8);
#else
	regval = AD7793_CONF_REFSEL(AD7793_REFSEL_EXT) |
			 AD7793_CONF_BUF |
			 AD7793_CONF_UNIPOLAR |
			 AD7793_CONF_GAIN(AD7793_GAIN_8);
#endif

	AD7793_SetRegisterValue(chipSelect, AD7793_REG_CONF, regval, 2);

	/* Perform Internal Zero-Scale Calibration */

	i = AD7793_Calibrate(chipSelect, AD7793_MODE_CAL_INT_ZERO, AD7793_CH_AIN1P_AIN1M);

#if 1
    if (!i)
    	System_printf("AD7793 %d CALIBRATE FAILED\n", chipSelect);
    else
    	System_printf("AD7793 calibrated %d\n", i);
#endif

    System_flush();

}

//*****************************************************************************
// The principle of operation is to measure the resistance of a platinum
// element. The most common type (PT100) has a resistance of 100 ohms at 0 °C
// and 138.4 ohms at 100 °C. There are also PT1000 sensors that have a
// resistance of 1000 ohms at 0 °C. The relationship between temperature and
// resistance is approximately linear over a small temperature range:
// for example, if you assume that it is linear over the 0 to 100 °C range,
// the error at 50 °C is 0.4 °C. For precision measurement, it is necessary
// to linearise the resistance to give an accurate temperature.
//
// Errors in the excitation current sources cancel out and the reading
// is simply the ratio of the unknown impedance (RTD) (multiplied by the
// INAMP Gain) to the known impedance (RREF). This is a 3-wire ratiometric
// measurement.
//*****************************************************************************

#define RTD_A	( 3.9083e-3 )			/* A = 0.00398 RTD coefficient */
#define RTD_B	( -5.775e-7 )
#define RTD_R0	( 100.0f )				/* RTD is 100 ohms at 0C       */
#define RREF	( 4990.0f )				/* Reference Resistor Value is 4.99k */
#define GAIN	( 8.0f )				/* AD7793 gain setting */


float AD7793_temperature(uint32_t adcValue)
{
	float Z1, Z2, Z3, Z4, Rt, temp;

	/* Calculate the RTD's resistance value via.
	 * The formula as current excitation mode follows:
	 *
	 * Rrtd = Rref * ((CODE/2^N) / GAIN)
	 *
	 * where N = number of full scale ADC bits (8388608.0f)
	 */

	float adc = (float)adcValue;

	/* 210uA = 2.0958V */
	//float Vref = RREF * 2.0f * 0.000210f;
	//Rt = (adc * (2.0f * RREF)) / Vref;

	Rt = RREF * ((adc / 8388608.0f) / GAIN);

	Z1 = -(RTD_A);
	Z2 = (RTD_A * RTD_A) - 4.0f * RTD_B;
	Z3 = (4.0f * RTD_B) / RTD_R0;
	Z4 = 2.0f * RTD_B;

	temp = Z2 + (Z3 * Rt);
	temp = (sqrtf(temp) + Z1) / Z4;

	if (temp >= 0.0f)
		return temp;

	float rpoly = Rt;

	temp = -242.02f;
	temp += 2.2228f * rpoly;
	rpoly *= Rt;  // square
	temp += 2.5859e-3f * rpoly;
	rpoly *= Rt;  // ^3
	temp -= 4.8260e-6f * rpoly;
	rpoly *= Rt;  // ^4
	temp -= 2.8183e-8f * rpoly;
	rpoly *= Rt;  // ^5
	temp += 1.5243e-10f * rpoly;

	return temp;
}

//*****************************************************************************
// Send 32 consecutive 1's to the SPI port for internal device reset.
//*****************************************************************************

bool AD7793_Reset(uint32_t chipSelect)
{
	bool success = true;
	SPI_Transaction transaction;
	uint32_t txBuf = 0xFFFFFFFF;
	uint32_t rxBuf;

	/* Initialize opcode transaction structure */
	transaction.count = 4;
	transaction.txBuf = (Ptr)&txBuf;
	transaction.rxBuf = (Ptr)&rxBuf;

	GPIO_write(chipSelect, PIN_LOW);			/* Assert the chip select low  */
	SPI_transfer(s_spiHandle, &transaction);	/* Initiate SPI transfer       */
	GPIO_write(chipSelect, PIN_HIGH);			/* Release chip select to high */

	/* Settling time after chip reset */
	Task_sleep(10);
	
	return success;
}

//*****************************************************************************
// Test the RDY bit from the status register
//*****************************************************************************

bool AD7793_IsReady(uint32_t chipSelect)
{
	SPI_Transaction transaction;
	uint8_t txBuf[2];
	uint8_t rxBuf[2];
	
	txBuf[0] = AD7793_COMM_READ | AD7793_COMM_ADDR(AD7793_REG_STAT);
	txBuf[1] = 0;

	/* Initialize opcode transaction structure */
	transaction.count = 2;
	transaction.txBuf = (Ptr)&txBuf;
	transaction.rxBuf = (Ptr)&rxBuf;

	GPIO_write(chipSelect, PIN_LOW);			/* Assert the chip select low  */
	SPI_transfer(s_spiHandle, &transaction);	/* Initiate SPI transfer       */
	GPIO_write(chipSelect, PIN_HIGH);			/* Release chip select to high */
	
	return (rxBuf[1] & AD7793_STAT_RDY) ? true : false;
}

//*****************************************************************************
//
//*****************************************************************************

void AD7793_SetRegisterValue(
		uint32_t chipSelect,
		uint8_t regAddress,
        uint32_t regValue,
        uint8_t size
		)
{
	uint8_t txBuf[5];
	uint8_t rxBuf[5];
	SPI_Transaction transaction;

	memset(txBuf, 0, sizeof(txBuf));

	transaction.count = size + 1;
	transaction.txBuf = (Ptr)&txBuf;
	transaction.rxBuf = (Ptr)&rxBuf;

	txBuf[0] = AD7793_COMM_WRITE | AD7793_COMM_ADDR(regAddress);

	/* Format the register data value to send */

    switch (size)
    {
		case 3:
			txBuf[1] = (uint8_t)(regValue >> 16);
			txBuf[2] = (uint8_t)(regValue >> 8);
			txBuf[3] = (uint8_t)regValue;
			break;
		case 2:
			txBuf[1] = (uint8_t)(regValue >> 8);
			txBuf[2] = (uint8_t)regValue;
			break;
		case 1:
			txBuf[1] = (uint8_t)regValue;
			break;
		default:
			break;
	}

    if (chipSelect)
    	GPIO_write(chipSelect, PIN_LOW);		/* Assert the chip select low  */

	SPI_transfer(s_spiHandle, &transaction);	/* Initiate SPI transfer       */

	if (chipSelect)
		GPIO_write(chipSelect, PIN_HIGH);		/* Release chip select to high */
}

//*****************************************************************************
//
//*****************************************************************************

uint32_t AD7793_GetRegisterValue(
		uint32_t chipSelect,
		uint8_t regAddress,
		uint8_t size
		)
{
	uint8_t txBuf[5];
	uint8_t rxBuf[5];
	uint32_t rxData = 0;
	SPI_Transaction transaction;

	memset(txBuf, 0, sizeof(txBuf));
	memset(rxBuf, 0, sizeof(rxBuf));

	txBuf[0] = AD7793_COMM_READ | AD7793_COMM_ADDR(regAddress);

	transaction.count = size + 1;
	transaction.txBuf = (Ptr)&txBuf;
	transaction.rxBuf = (Ptr)&rxBuf;

	if (chipSelect)
		GPIO_write(chipSelect, PIN_LOW);		/* Assert the chip select low  */

	SPI_transfer(s_spiHandle, &transaction);	/* Initiate SPI transfer       */

	if (chipSelect)
		GPIO_write(chipSelect, PIN_HIGH);		/* Release chip select to high */

	/* Extract the data value returned */

	switch (size)
	{
		case 3:
			// In most cases, the ADC code is read by a microcontroller in 8-bit
			// segments and concatenated into a 32-bit data type. If the ADC’s 
			// resolution is less than 32 bits and the output code is signed, the
			// data will need to be sign-extended into the 32-bit integer data 
			// type to preserve the sign.

#ifdef BIPOLAR_24BIT
			rxData = (uint32_t)(((((rxBuf[1] & 0x80) ? 0xFF : 0x00)) << 24) |
                                                  ((rxBuf[1] & 0xFF) << 16) |
                                                  ((rxBuf[2] & 0xFF) << 8 ) |
                                                  ((rxBuf[3] & 0xFF) << 0 ));
#else
			rxData = (uint32_t)((rxBuf[1] << 16) | (rxBuf[2] << 8) | rxBuf[3]);
#endif
			break;
		case 2:
			rxData = (uint32_t)((rxBuf[1] << 8) | rxBuf[2]);
			break;
		case 1:
			rxData = (uint32_t)(rxBuf[1]);
			break;
		default:
			break;
	}

    return rxData;
}

//*****************************************************************************
//
//*****************************************************************************

void AD7793_SetMode(uint32_t chipSelect, uint32_t mode)
{
    uint32_t data;
    
    data = AD7793_GetRegisterValue(chipSelect, AD7793_REG_MODE, 2);

    data &= ~(AD7793_MODE_SEL(0xFF));
    data |= AD7793_MODE_SEL(mode);

    AD7793_SetRegisterValue(chipSelect, AD7793_REG_MODE, data, 2);
}

//*****************************************************************************
//
//*****************************************************************************

void AD7793_SetChannel(uint32_t chipSelect, uint32_t channel)
{
    uint32_t data;
    
    data = AD7793_GetRegisterValue(chipSelect, AD7793_REG_CONF, 2);

    data &= ~(AD7793_CONF_CHAN(0xFF));
    data |= AD7793_CONF_CHAN(channel);

    AD7793_SetRegisterValue(chipSelect, AD7793_REG_CONF, data, 2);
}

//*****************************************************************************
//
//*****************************************************************************

void AD7793_SetGain(uint32_t chipSelect, uint32_t gain)
{
    uint32_t data;
    
    data = AD7793_GetRegisterValue(chipSelect, AD7793_REG_CONF, 2);

    data &= ~(AD7793_CONF_GAIN(0xFF));
    data |= AD7793_CONF_GAIN(gain);

    AD7793_SetRegisterValue(chipSelect, AD7793_REG_CONF, data, 2);
}

//*****************************************************************************
//
//*****************************************************************************

void AD7793_SetIntReference(uint32_t chipSelect, uint8_t type)
{
    uint32_t data = 0;
    
    data = AD7793_GetRegisterValue(chipSelect, AD7793_REG_CONF, 2);

    data &= ~(AD7793_CONF_REFSEL(AD7793_REFSEL_INT));
    data |= AD7793_CONF_REFSEL(type);

    AD7793_SetRegisterValue(chipSelect, AD7793_REG_CONF, data, 2);
}

//*****************************************************************************
//
//*****************************************************************************

int AD7793_Wait4Ready(uint32_t chipSelect)
{
	int i;
	int status = 0;
	uint8_t  pins;
	uint32_t cfg;
	uint32_t port;

	/* Here we temporarily swap the GPIO input on PD0 to input mode
	 * and look waiting for it to complete to signal RDY status,
	 * then we restore the GPIO back to SSI mode.
	 */

	switch (chipSelect)
	{
	case Board_SLOT2_AD7793_CS1:
	case Board_SLOT2_AD7793_CS2:
		port = GPIO_PORTD_BASE;
		pins = GPIO_PIN_0;
		cfg  = GPIO_PD0_SSI2XDAT1;
		break;
	}

	/* PD0->SSI2XDAT1->RDY pin */
	GPIOPinTypeGPIOInput(port, pins);

	/* Now loop polling for the RDY pin to go low */

	for (i=0; i < 100; i++)
	{
		int32_t mask = GPIOPinRead(port, pins);

		if (!mask)
		{
			status = i+1;
			break;
		}

		Task_sleep(1);
	}

	/* Set the SSInXDAT1 back to SPI mode */
	GPIOPinConfigure(cfg);
	GPIOPinTypeSSI(port, pins);

	return status;
}

//*****************************************************************************
//
//*****************************************************************************

int AD7793_Calibrate(uint32_t chipSelect, uint8_t mode, uint8_t channel)
{
	int i;
    uint32_t oldRegValue = 0x00;
    uint32_t newRegValue = 0x00;
    
    AD7793_SetChannel(chipSelect, channel);

    oldRegValue &= AD7793_GetRegisterValue(chipSelect, AD7793_REG_MODE, 2);

    oldRegValue &= ~(AD7793_MODE_SEL(0x7));
    newRegValue = oldRegValue | AD7793_MODE_SEL(mode);

	/* Assert the chip select low  */
    GPIO_write(chipSelect, PIN_LOW);
    
    /* Issue the calibrate command */
    AD7793_SetRegisterValue(0, AD7793_REG_MODE, newRegValue, 2);

    /* Wait for RDY status */
    i = AD7793_Wait4Ready(chipSelect);

    /* Release chip select to high */
    GPIO_write(chipSelect, PIN_HIGH);

    return i;
}

//*****************************************************************************
//
//*****************************************************************************

uint32_t AD7793_SingleConversion(uint32_t chipSelect)
{
    uint32_t mode;
    uint32_t regData = 0;

    mode = AD7793_MODE_SEL(AD7793_MODE_SINGLE);

    /* Assert the chip select low  */
	GPIO_write(chipSelect, PIN_LOW);

    /* CS is not modified by SPI read/write functions */
    AD7793_SetRegisterValue(0, AD7793_REG_MODE, mode, 2);

    /* Wait for RDY status */
    AD7793_Wait4Ready(chipSelect);

    /* CS is not modified by SPI read/write functions */
    regData = AD7793_GetRegisterValue(0, AD7793_REG_DATA, 3);

    /* Release chip select to high */
	GPIO_write(chipSelect, PIN_HIGH);

	/* Set the ADC back to idle mode */
	AD7793_SetMode(chipSelect, AD7793_MODE_IDLE);

    return regData;
}

//*****************************************************************************
//
//*****************************************************************************

uint32_t AD7793_ContinuousReadAvg(uint32_t chipSelect, uint32_t numSamples)
{
    uint32_t samplesAverage = 0;
    uint32_t mode;
    uint32_t n;

    mode = AD7793_MODE_SEL(AD7793_MODE_CONT);

    /* Assert the chip select low  */
	GPIO_write(chipSelect, PIN_LOW);

	/* CS is not modified by the SPI  write */
    AD7793_SetRegisterValue(0, AD7793_REG_MODE, mode, 2);

    for(n=0; n < numSamples; n++)
    {
        /* Wait for RDY status */
        AD7793_Wait4Ready(chipSelect);

        /* CS is not modified by the SPI read */
        samplesAverage += AD7793_GetRegisterValue(0, AD7793_REG_DATA, 3);
    }

    /* Release chip select to high */
	GPIO_write(chipSelect, PIN_HIGH);

    samplesAverage = samplesAverage / numSamples;

    /* Set the ADC back to idle mode */
	AD7793_SetMode(chipSelect, AD7793_MODE_IDLE);

    return samplesAverage;
}

/* End-Of-File */
