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

/* TI-RTOS Driver files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SDSPI.h>
#include <ti/drivers/I2C.h>
#include <ti/drivers/UART.h>

#include <file.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdbool.h>

/* PMX42 Board Header file */
#include "Board.h"
#include "PMX42.h"
#include "RAMP.h"

/* Static Function Prototypes */

static int RxFlushStream(UART_Handle handle);
static uint16_t CRC16Update(uint16_t crc, uint8_t data);

//*****************************************************************************
// Initalize FCB structure to default values
//*****************************************************************************

void RAMP_InitFcb(FCB* fcb)
{
	fcb->type    = MAKETYPE(0, TYPE_MSG_ONLY);
	fcb->seqnum  = MIN_SEQ_NUM;
	fcb->acknak  = 0;
	fcb->address = 0;
	fcb->textlen = 0;
	fcb->textbuf = NULL;
}

//*****************************************************************************
// Transmit a RAMP frame of data out the RS-422 port
//*****************************************************************************

int RAMP_TxFrame(UART_Handle handle, FCB *fcb, uint8_t *textbuf, int textlen)
{
    int i;
	uint8_t b;
    uint8_t *p;
    uint16_t crc = 0;

    /* Are we sending an ACK/NAK only frame? */
    if ((fcb->type == 0x011) || (fcb->type == 0x012))
    {
		fcb->textlen   = 0;
		fcb->framelen  = ACK_FRAME_LEN;
    }
    else
    {
		fcb->textlen   = textlen;
		fcb->framelen  = textlen + (FRAME_OVERHEAD - PREAMBLE_OVERHEAD);
		fcb->textbuf   = textbuf;
    }

    /* Send the Preamble MSB for the frame start */
    b = PREAMBLE_MSB;
    UART_write(handle, &b, 1);

    /* Send the Preamble LSB for the frame start */
    b = PREAMBLE_LSB;
    UART_write(handle, &b, 1);

    /* CRC starts here, sum in the phantom byte first to prime things */
    crc = CRC16Update(crc, CRC_PHANTOM_BYTE);

    /* Send the Frame length (MSB) */
    b = (uint8_t)((fcb->framelen >> 8) & 0xFF);
    crc = CRC16Update(crc, b);
    UART_write(handle, &b, 1);

    /* Send the Frame length (LSB) */
    b = (uint8_t)(fcb->framelen & 0xFF);
    crc = CRC16Update(crc, b);
    UART_write(handle, &b, 1);

    /* Send the Frame Type Byte */
    b = (uint8_t)(fcb->type & 0xFF);
    crc = CRC16Update(crc, b);
    UART_write(handle, &b, 1);

    /* Send the Frame Address Byte */
    b = (uint8_t)(fcb->address & 0xFF);
    crc = CRC16Update(crc, b);
    UART_write(handle, &b, 1);

    /* Check for ACK or NAK only frame (always 11H or 12H) */

    if ((fcb->type == 0x011) || (fcb->type == 0x012))
    {
		/* Send the ACK/NAK Sequence Number */
		b = (uint8_t)(fcb->acknak & 0xFF);
		crc = CRC16Update(crc, b);
		UART_write(handle, &b, 1);
    }
    else
    {
    	/* We're sending a full RAMP frame, continue endcoding the rest of the frame */

		/* Send the Frame Sequence Number */
		b = (uint8_t)(fcb->seqnum & 0xFF);
		crc = CRC16Update(crc, b);
		UART_write(handle, &b, 1);

		/* Send the ACK/NAK Sequence Number */
		b = (uint8_t)(fcb->acknak & 0xFF);
		crc = CRC16Update(crc, b);
		UART_write(handle, &b, 1);

		/* Send the Text length (MSB) */
		b = (uint8_t)((fcb->textlen >> 8) & 0xFF);
		crc = CRC16Update(crc, b);
		UART_write(handle, &b, 1);

		/* Send the Text length (LSB) */
		b = (uint8_t)(fcb->textlen & 0xFF);
		crc = CRC16Update(crc, b);
		UART_write(handle, &b, 1);

		/* Send any Text data associated with the frame */

		if (fcb->textbuf && fcb->textlen)
		{
			p = fcb->textbuf;

			for (i=0; i < fcb->textlen; i++)
			{
				b = *p++;
				crc = CRC16Update(crc, b);
				UART_write(handle, &b, 1);
			}
		}
    }

    /* Send the CRC MSB */
    b = (uint8_t)(crc >> 8);
    UART_write(handle, &b, 1);

    /* Send the CRC LSB */
    b = (uint8_t)(crc & 0xFF);
    UART_write(handle, &b, 1);

    return 0;
}

//*****************************************************************************
// Receive a RAMP data frame from the RS-422 port
//*****************************************************************************

int RAMP_RxFrame(UART_Handle handle, FCB *fcb, uint8_t *textbuf, int maxlen)
{
	int rc = 0;
    int i;
    uint16_t lsb;
    uint16_t msb;
	uint8_t b;
    uint8_t *p;
    uint16_t crc = 0;

    /* Save the receive text buffer pointer */
    fcb->textbuf = textbuf;

    /* Read the Preamble MSB for the frame start */
    if (UART_read(handle, &b, 1) != 1)
    	return ERR_TIMEOUT;

    if (b != PREAMBLE_MSB)
    {
    	RxFlushStream(handle);
    	return ERR_BAD_PREAMBLE;
    }

    /* Read the Preamble LSB for the frame start */
    if (UART_read(handle, &b, 1) != 1)
    	return ERR_TIMEOUT;

    if (b != PREAMBLE_LSB)
    {
    	RxFlushStream(handle);
    	return ERR_BAD_PREAMBLE;
    }

    /* CRC starts here, sum in the phantom byte first to prime things */
    crc = CRC16Update(crc, CRC_PHANTOM_BYTE);

    /* Read the Frame length (MSB) */
    if (UART_read(handle, &b, 1) != 1)
    	return ERR_SHORT_FRAME;

    crc = CRC16Update(crc, b);
    msb = (uint16_t)b;

    /* Read the Frame length (LSB) */
    if (UART_read(handle, &b, 1) != 1)
    	return ERR_SHORT_FRAME;

    crc = CRC16Update(crc, b);
    lsb = (uint16_t)b;

    /* Build and validate maximum frame length */
    fcb->framelen = (size_t)((msb << 8) | lsb) & 0xFFFF;

    if (fcb->framelen > MAX_FRAME_LEN)
    {
    	RxFlushStream(handle);
    	return ERR_FRAME_LEN;
    }

    /* Read the Frame Type Byte */
    if (UART_read(handle, &b, 1) != 1)
    	return ERR_SHORT_FRAME;

    crc = CRC16Update(crc, b);
    fcb->type = b;

	/* Read the Frame Address Byte */
	if (UART_read(handle, &b, 1) != 1)
		return ERR_SHORT_FRAME;

	crc = CRC16Update(crc, b);
	fcb->address = b;

    /* Check for ACK/NAK only frame (type always 11H or 12H) */

    if ((fcb->framelen == ACK_FRAME_LEN) && ((fcb->type == 0x011) || (fcb->type == 0x012)))
    {
		/* Read the ACK/NAK Sequence Number */
		if (UART_read(handle, &b, 1) != 1)
			return ERR_SHORT_FRAME;

		crc = CRC16Update(crc, b);
		fcb->acknak = b;
    }
    else
    {
    	/* It's a full RAMP frame, continue decoding the rest of the frame */

		/* Read the Frame Sequence Number */
		if (UART_read(handle, &b, 1) != 1)
			return ERR_SHORT_FRAME;

		crc = CRC16Update(crc, b);
		fcb->seqnum = b;

		/* Read the ACK/NAK Sequence Number */
		if (UART_read(handle, &b, 1) != 1)
			return ERR_SHORT_FRAME;

		crc = CRC16Update(crc, b);
		fcb->acknak = b;

		/* Read the Text length (MSB) */
		if (UART_read(handle, &b, 1) != 1)
			return ERR_SHORT_FRAME;

		crc = CRC16Update(crc, b);
		msb = (uint16_t)b;

		/* Read the Text length (LSB) */
		if (UART_read(handle, &b, 1) != 1)
			return ERR_SHORT_FRAME;

		crc = CRC16Update(crc, b);
		lsb = (uint16_t)b;

		/* Get the frame length received and validate it */
		fcb->textlen = (size_t)((msb << 8) | lsb) & 0xFFFF;

		/* Check the text length is not above our maximum payload size */
		if (fcb->textlen > MAX_TEXT_LEN)
		{
			RxFlushStream(handle);
			return ERR_TEXT_LEN;
		}

		/* The text length should be the frame overhead minus the preamble overhead
		 * plus the text length specified in the received frame. If these don't match
		 * then we have either a packet data error or a malformed packet.
		 */
		if (fcb->textlen + (FRAME_OVERHEAD - PREAMBLE_OVERHEAD) != fcb->framelen)
		{
			RxFlushStream(handle);
			return ERR_TEXT_LEN;
		}

		/* Read text data associated with the frame */

		p = fcb->textbuf;

		for (i=0; i < fcb->textlen; i++)
		{
			if (UART_read(handle, &b, 1) != 1)
				return ERR_SHORT_FRAME;

			/* update the CRC */
			crc = CRC16Update(crc, b);

			if (i >= maxlen)
			{
				rc = ERR_RX_OVERFLOW;
				continue;
			}

			if (p)
				*p++ = b;
		}
    }

    /* Read the packet CRC MSB */
    if (UART_read(handle, &b, 1) != 1)
    	return ERR_SHORT_FRAME;

    msb = (uint16_t)b & 0xFF;

    /* Read the packet CRC LSB */
    if (UART_read(handle, &b, 1) != 1)
    	return ERR_SHORT_FRAME;

    lsb = (uint16_t)b & 0xFF;

    /* Build and validate the CRC */
    fcb->crc = (uint16_t)((msb << 8) | lsb) & 0xFFFF;

    /* Validate the CRC values match */
    if (fcb->crc != crc)
    	rc = ERR_CRC;

    return rc;
}

//*****************************************************************************
// Flush the rx stream until no more data is read for at least 1 second.
//*****************************************************************************

int RxFlushStream(UART_Handle handle)
{
	int n = 0;
	uint8_t b;

	for (;;)
	{
		/* Attempt to read a character for 1 sec */
		if (UART_read(handle, &b, 1) != 1)
			break;
		/* Got something, keep reading */
		++n;
	}

	return n;
}

//*****************************************************************************
// Update the CRC-16 sum value for a byte
//*****************************************************************************

uint16_t CRC16Update(uint16_t crc, uint8_t data)
{
    int i;

    crc = crc ^ ((uint16_t)data << 8);

    for (i=0; i < 8; i++)
    {
        if (crc & 0x8000)
            crc = (crc << 1) ^ 0x1021;
        else
            crc <<= 1;
    }

    return crc;
}

// End-Of-File
