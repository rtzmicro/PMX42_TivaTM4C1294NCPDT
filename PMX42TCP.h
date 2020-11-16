// =========================================================================
// PMX421200TCP.h v1.02 01/04/2020
//
// PMX42 Client/Server Network Packet Definitions
//
// Developed by Robert E. Starr, Jr.
//
// Copyright (C) 2019, RTZ Professional Audio, LLC
//
// RTZ is a registered trademark of RTZ Professional Audio, LLC
// All Rights Reserved
// =========================================================================

#ifndef _PMX42TCP_H_
#define _PMX42TCP_H_

#pragma once
#pragma pack(push, 8)

#include <stdint.h>

// =========================================================================
// TCP/IP Port Numbers for PMX42 remote server
// =========================================================================

#define PMX42_PORT_STATE        4200    /* streaming transport state   */
#define PMX42_PORT_COMMAND      4201    /* transport cmd/response port */

/* Defines the maximum number of tracks supported by any machine.
 * Some machines may have less, like 16 or 8 track machines.
 */
#define PMX42_MAX_CHANNELS      8       /* max ADC channels */

// =========================================================================
// PMX42 state update message structure. This message streams from the PMX42
// to the TCP client to indicate the current transport time, the led/lamp
// states and other real time feedback information. State packets stream
// anytime there is tape motion, or any other transport state change
// event occurs. This is a one way stream to the client and the PMX42-1200
// server does not attempt to receive any data back on this port stream.
// =========================================================================

typedef struct _PMX42_STATE_MSG {
    uint32_t    length;                 /* size of this msg structure */
    uint32_t    adc_id;                 /* ADC ID 16 or 24 bit        */
    uint32_t    adc_channels;           /* max num of ADC channels    */
    uint32_t    adc_data[PMX42_MAX_CHANNELS];
} PMX42_STATE_MSG;

// =========================================================================
// PMX42 COMMAND/RESPONSE Messages
// =========================================================================

typedef struct _PMX42_COMMAND_HDR {
    uint16_t    hdrlen;                 /* size of this msg structure */
    uint16_t    command;                /* the command ID to execute  */
    uint16_t    index;                  /* ADC channel index          */
    uint16_t    status;                 /* return status/error code   */
    union {
        uint32_t    U;
        int32_t     I;
        float       F;
    } param1;                           /* int, unsigned or float     */
    union {
        uint32_t    U;
        int32_t     I;
        float       F;
    }  param2;                          /* int, unsigned or float     */
    uint16_t    datalen;                /* trailing payload data len  */
} PMX42_COMMAND_HDR;

/*
 * Locator Command Message Types for 'PMX42_COMMAND_HDR.command'
 */

#define PMX42_CMD_SETLAMP           1   /* params.U 0=off, 1=on       */

#pragma pack(pop)

#endif /* _PMX42TCP_H_ */
