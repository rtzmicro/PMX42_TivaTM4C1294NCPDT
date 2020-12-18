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
 *    ======== tcpEchoHooks.c ========
 *    Contains non-BSD sockets code (NDK Network Open Hook)
 */

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/cfg/global.h>
#include <xdc/runtime/Error.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Driver files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/SDSPI.h>

/* NDK BSD support */
#include <sys/socket.h>

#include <file.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* PMX42 Board Header file */
#include "Board.h"
#include "PMX42.h"
#include "PMX42TCP.h"

#define NUMTCPWORKERS       4

#ifdef CYASSL_TIRTOS
#define TCPHANDLERSTACK     8704
#else
#define TCPHANDLERSTACK     1024
#endif

/* Global PMX42 System data */
extern SYSDATA g_sys;

/* Prototypes */
//void netOpenHook(void);
void netIPUpdate(unsigned int IPAddr, unsigned int IfIdx, unsigned int fAdd);
Void tcpStateHandler(UArg arg0, UArg arg1);
Void tcpStateWorker(UArg arg0, UArg arg1);
Void tcpCommandHandler(UArg arg0, UArg arg1);
Void tcpCommandWorker(UArg arg0, UArg arg1);

static int ReadData(int fd, void *pbuf, int size, int flags);
static int WriteData(int fd, void *pbuf, int size, int flags);

/* External Function Prototypes */
extern void NtIPN2Str(uint32_t IPAddr, char *str);

//*****************************************************************************
// NDK network IP update hook used to get IP address from DHCP
//*****************************************************************************

// This handler is called when the DHCP client is assigned an
// address from a DHCP server. We store this in our runtime data
// structure for use later.

void netIPUpdate(unsigned int IPAddr, unsigned int IfIdx, unsigned int fAdd)
{
    if (fAdd)
        NtIPN2Str(IPAddr, g_sys.ipAddr);
    else
        NtIPN2Str(0, g_sys.ipAddr);

    System_printf("netIPUpdate() dhcp->%s\n", g_sys.ipAddr);
    System_flush();
}

//*****************************************************************************
// NDK network open hook used to initialize IPv6
//*****************************************************************************

void netOpenHook(void)
{
    Task_Handle taskHandle;
    Task_Params taskParams;
    Error_Block eb;

    /* Make sure Error_Block is initialized */
    Error_init(&eb);

    /* Create the task that listens for incoming TCP connections
     * to handle streaming transport state info. The parameter arg0
     * will be the port that this task listens on.
     */

    Task_Params_init(&taskParams);

    taskParams.stackSize = TCPHANDLERSTACK;
    taskParams.priority  = 1;
    taskParams.arg0      = PMX42_PORT_STATE;

    taskHandle = Task_create((Task_FuncPtr)tcpStateHandler, &taskParams, &eb);

    if (taskHandle == NULL) {
        System_printf("netOpenHook: Failed to create tcpStateHandler Task\n");
    }

    /* Create the Task that listens for incoming TCP connections
     * to handle command/response requests. The parameter arg0 will
     * be the port that this task listens on.
     */

    Task_Params_init(&taskParams);

    taskParams.stackSize = TCPHANDLERSTACK;
    taskParams.priority  = 1;
    taskParams.arg0      = PMX42_PORT_COMMAND;

    taskHandle = Task_create((Task_FuncPtr)tcpCommandHandler, &taskParams, &eb);

    if (taskHandle == NULL) {
        System_printf("netOpenHook: Failed to create tcpCommandHandler Task\n");
    }

    System_flush();
}

//*****************************************************************************
// LISTENER CREATES TRANSPORT STATE STREAMING WORKER TASK FOR NEW CONNECTIONS.
//*****************************************************************************

Void tcpStateHandler(UArg arg0, UArg arg1)
{
    int                status;
    int                clientfd;
    int                server = 0;
    struct sockaddr_in localAddr;
    struct sockaddr_in clientAddr;
    int                optval;
    int                optlen = sizeof(optval);
    socklen_t          addrlen = sizeof(clientAddr);
    Task_Handle        taskHandle;
    Task_Params        taskParams;
    Error_Block        eb;

    //Task_Handle hSelf = Task_self();
    //fdOpenSession(hSelf);

    server = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

    if (server == -1) {
        System_printf("Error: socket not created.\n");
        goto shutdown;
    }

    memset(&localAddr, 0, sizeof(localAddr));

    localAddr.sin_family      = AF_INET;
    localAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    localAddr.sin_port        = htons(arg0);

    status = bind(server, (struct sockaddr *)&localAddr, sizeof(localAddr));

    if (status == -1) {
        System_printf("Error: bind failed.\n");
        goto shutdown;
    }

    status = listen(server, NUMTCPWORKERS);

    if (status == -1) {
        System_printf("Error: listen failed.\n");
        goto shutdown;
    }

    if (setsockopt(server, SOL_SOCKET, SO_KEEPALIVE, &optval, optlen) < 0) {
        System_printf("Error: setsockopt failed\n");
        goto shutdown;
    }

    while ((clientfd = accept(server, (struct sockaddr *)&clientAddr, &addrlen)) != -1)
    {
        System_printf("tcpStateHandler: Creating thread clientfd = %d\n", clientfd);
        System_flush();

        /* Init the Error_Block */
        Error_init(&eb);

        /* Initialize the defaults and set the parameters. */
        Task_Params_init(&taskParams);

        taskParams.arg0      = (UArg)clientfd;
        taskParams.stackSize = 1280;

        taskHandle = Task_create((Task_FuncPtr)tcpStateWorker, &taskParams, &eb);

        if (taskHandle == NULL) {
            System_printf("Error: Failed to create new Task\n");
            System_flush();
            close(clientfd);
        }

        /* addrlen is a value-result param, must reset for next accept call */
        addrlen = sizeof(clientAddr);
    }

    System_printf("Error: accept failed.\n");

shutdown:

    System_flush();

    if (server > 0) {
        close(server);
    }

    //fdClose(hSelf);
}

//*****************************************************************************
// STREAMS TRANSPORT STATE CHANGE INFO TO CLIENT. THERE CAN BE MULTIPLE
// STREAMING STATE WORKER THREADS RUNNING.
//*****************************************************************************

Void tcpStateWorker(UArg arg0, UArg arg1)
{
    int         clientfd = (int)arg0;
    int         bytesSent;
    int         bytesToSend;
    uint8_t*    buf;
    size_t      i;
    bool        connected = true;

    PMX42_STATE_MSG stateMsg;

    System_printf("tcpStateWorker: CONNECT clientfd = 0x%x\n", clientfd);
    System_flush();

    /* Set initially to send tape time update packet
     * since the client just connected.
     */
    Event_post(g_eventState, Event_Id_00);

    const UInt EVENT_MASK = Event_Id_00|Event_Id_01|Event_Id_02|Event_Id_03|Event_Id_04;

    while (connected)
    {
        /* Wait for a position change event from the tape roller position task */
        UInt events = Event_pend(g_eventState, Event_Id_NONE, EVENT_MASK, 2500);

        memset(&stateMsg, 0, sizeof(PMX42_STATE_MSG));

        int textlen = sizeof(PMX42_STATE_MSG);

        stateMsg.length       = textlen;
        stateMsg.adc_channels = g_sys.adcChannels;
        stateMsg.adc_id       = g_sys.adcID;

        /* Copy the track state info */
        for (i=0; i < stateMsg.adc_channels; i++)
            stateMsg.adc_data[i] = g_sys.adcData[i];

        /* Prepare to start sending state message buffer */

        bytesToSend = textlen;

        buf = (uint8_t*)&stateMsg;

        do {

            if ((bytesSent = send(clientfd, buf, bytesToSend, 0)) <= 0)
            {
                System_printf("tcpStateWorker send failed = %d\n", bytesSent);
                connected = false;
                break;
            }

            bytesToSend -= bytesSent;

            buf += bytesSent;

        } while (bytesToSend > 0);
    }

    System_printf("tcpStateWorker DISCONNECT clientfd = 0x%x\n", clientfd);
    System_flush();

    close(clientfd);
}

//*****************************************************************************
// LISTENER CREATES COMMAND/RESPONSE WORKER TASK FOR NEW CONNECTIONS.
//*****************************************************************************

Void tcpCommandHandler(UArg arg0, UArg arg1)
{
    int                status;
    int                clientfd;
    int                server;
    struct sockaddr_in localAddr;
    struct sockaddr_in clientAddr;
    int                optval;
    int                optlen = sizeof(optval);
    socklen_t          addrlen = sizeof(clientAddr);
    Task_Handle        taskHandle;
    Task_Params        taskParams;
    Error_Block        eb;

    //Task_Handle hSelf = Task_self();
    //fdOpenSession(hSelf);

    server = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);

    if (server == -1) {
        System_printf("Error: socket not created.\n");
        goto shutdown;
    }

    memset(&localAddr, 0, sizeof(localAddr));

    localAddr.sin_family      = AF_INET;
    localAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    localAddr.sin_port        = htons(arg0);

    status = bind(server, (struct sockaddr *)&localAddr, sizeof(localAddr));

    if (status == -1) {
        System_printf("Error: bind failed.\n");
            goto shutdown;
    }

    status = listen(server, NUMTCPWORKERS);

    if (status == -1) {
        System_printf("Error: listen failed.\n");
            goto shutdown;
    }

    optval = 100;

    if (setsockopt(server, SOL_SOCKET, SO_KEEPALIVE, &optval, optlen) < 0) {
        System_printf("Error: setsockopt failed\n");
        goto shutdown;
    }

    while ((clientfd = accept(server, (struct sockaddr *)&clientAddr, &addrlen)) != -1)
    {
        //System_printf("tcpStateHandler: Creating thread clientfd = %d\n", clientfd);
        //System_flush();

        /* Init the Error_Block */
        Error_init(&eb);

        /* Initialize the defaults and set the parameters. */
        Task_Params_init(&taskParams);
        taskParams.arg0      = (UArg)clientfd;
        taskParams.stackSize = 1280;

        taskHandle = Task_create((Task_FuncPtr)tcpCommandWorker, &taskParams, &eb);

        if (taskHandle == NULL) {
            //System_printf("Error: Failed to create new Task\n");
            //System_flush();
            close(clientfd);
        }

        /* addrlen is a value-result param, must reset for next accept call */
        addrlen = sizeof(clientAddr);
    }

    System_printf("Error: accept failed.\n");
    System_flush();

shutdown:

    System_flush();

    if (server > 0) {
        close(server);
    }

    //fdClose(hSelf);
}

//*****************************************************************************
// HANDLES COMMAND/RESPONSE REQUESTS FROM CLIENT. THERE CAN BE MULTIPLE
// COMMAND HANDLER WORKER THREADS RUNNING.
//*****************************************************************************

Void tcpCommandWorker(UArg arg0, UArg arg1)
{
    bool        connected = true;
    int         clientfd = (int)arg0;
    int         bytesSent;
    int         bytesRcvd;
    size_t      index;
    uint16_t    status;

    PMX42_COMMAND_HDR msg;

    System_printf("tcpCommandWorker: CONNECT clientfd = 0x%x\n", clientfd);
    System_flush();

    while (connected)
    {
        /* Attempt to read a message header */
        bytesRcvd = ReadData(clientfd, &msg, sizeof(PMX42_COMMAND_HDR), 0);

        if (bytesRcvd <= 0)
        {
            System_printf("Error: TCP read error %d.\n", bytesRcvd);
            connected = FALSE;
            break;
        }

        msg.status = status = 0;

        /* Index into cue table or track table */
        index = (size_t)msg.index;

        /*
         * Determine which command to process from the client
         */

        switch(msg.command)
        {
        case PMX42_CMD_SETLAMP:
            /* param1: 0=off, 1=on
             * param2: not used, zero
             */
            break;
        }

        /* Now send the response packet */

        msg.status  = status;
        msg.datalen = 0;
        msg.index   = index;

        bytesSent =  WriteData(clientfd, &msg, sizeof(PMX42_COMMAND_HDR), 0);

        if (bytesSent <= 0)
        {
            System_printf("Error: TCP write error %d.\n", bytesSent);
            connected = false;
            break;
        }
    }

    System_printf("tcpCommandWorker DISCONNECT clientfd = 0x%x\n", clientfd);
    System_flush();

    close(clientfd);
}

/* This function performs a blocked read for 'size' number of bytes. It will
 * continue to read until all bytes are read, or return if an error occurs.
 */

int ReadData(int fd, void *pbuf, int size, int flags)
{
    int bytesRcvd = 0;
    int bytesToRecv = size;

    uint8_t* buf = (uint8_t*)pbuf;

    do {

        if ((bytesRcvd = recv(fd, buf, bytesToRecv, 0)) <= 0)
        {
            System_printf("Error: TCP recv failed %d.\n", bytesRcvd);
            break;
        }

        bytesToRecv -= bytesRcvd;

        buf += bytesRcvd;

    } while(bytesToRecv > 0);

    return bytesRcvd;
}

/* This function performs a blocked write for 'size' number of bytes. It will
 * continue to write until all bytes are sent, or return if an error occurs.
 */

int WriteData(int fd, void *pbuf, int size, int flags)
{
    int bytesSent = 0;
    int bytesToSend = size;

    uint8_t* buf = (uint8_t*)pbuf;

    do {

        if ((bytesSent = send(fd, buf, bytesToSend, 0)) <= 0)
        {
            System_printf("Error: TCP send failed %d.\n", bytesSent);
            break;
        }

        bytesToSend -= bytesSent;

        buf += bytesSent;

    } while (bytesToSend > 0);

    return bytesSent;
}

// End-Of-File
