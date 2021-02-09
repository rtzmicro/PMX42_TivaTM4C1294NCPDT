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

#ifndef __DISPLAYTASK_H
#define __DISPLAYTASK_H

/*** CONSTANTS AND CONFIGURATION *******************************************/

typedef enum ScreenNum {
    SCREEN_ABOUT=0,
    SCREEN_INFO,
    SCREEN_UV,

} ScreenNum;

#define SCREEN_FIRST        SCREEN_ABOUT
#define SCREEN_LAST         SCREEN_UV

/* Screen saver triggers in 30 minutes by default */
#define SCREEN_SAVER_TIME   30

/*** SYSTEM CONFIG PARAMETERS STORED IN EPROM ******************************/

typedef enum DisplayCommand{
	DISPLAY_DRAW,
} DisplayCommand;

typedef struct DisplayMessage{
    DisplayCommand	dispCmd;
    uint32_t		dispArg1;
    uint32_t		dispArg2;
} DisplayMessage;

/*** FUNCTION PROTOTYPES ***************************************************/

Void DisplayTaskFxn(UArg arg0, UArg arg1);

Bool DisplayRefresh(void);

ScreenNum DisplaySetScreen(ScreenNum screen);
ScreenNum DisplayGetScreen(void);

bool IsScreenSave(void);

#endif /* __DISPLAYTASK_H */
