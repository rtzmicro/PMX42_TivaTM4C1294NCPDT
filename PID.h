/* ============================================================================
 *
 * DTC-1200 Digital Transport Controller for Ampex MM-1200 Tape Machines
 *
 * Copyright (C) 2016, RTZ Professional Audio, LLC
 * All Rights Reserved
 *
 * RTZ is registered trademark of RTZ Professional Audio, LLC
 *
 * ============================================================================ */

#ifndef __PID_H__
#define __PID_H__

#define PID_TOLERANCE	3  	/* error tolerance */

/* Floating Point PID */

typedef struct _FPID {
    float   Kp;         	/* proportional gain */
    float   Ki;         	/* integral gain */
    float   Kd;         	/* derivative gain */
    float   error;          /* last setpoint error */
    float   esum;    		/* accumulated error */
    float	pvprev;         /* saved previous pv value */
    float	cvi;			/* integral component of CV */
    float	cvd;            /* derivative component of CV */
    float	tolerance;		/* error tolerance */
} FPID;

// PID Function Prototypes

void fpid_init(FPID* p, float Kp, float Ki, float Kd, float tolerance);
float fpid_calc(FPID* p, float setpoint, float actual);

#endif /* __PID_H__ */

/* end-of-file */
