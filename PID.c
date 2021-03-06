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

#include <limits.h>
#include <math.h>

#include "pid.h"
#include "PMX42_TM4C1294NCPDT.h"

/*
 * Function:    fpid_init()
 *
 * Synopsis:    void fpid_init(p, Kp, Ki, Kd, tolerance)
 *
 *              PID* p;     	-	 Pointer to PID data structure.
 *              float Kp;			- Proportional gain
 *              float Ki;    		- Integral gain
 *              float Kd;			- Derivative gain
 *              float tolerance;	- Error tolerance
 *
 * Description: This function initializes the static members of the PID
 *              data structure to the initial starting state.
 *
 * Returns:     void
 */

void fpid_init(
    FPID*   p,
    float   Kp,
	float   Ki,
	float   Kd,
	float   tolerance
    )
{
    p->Kp = Kp;		/* proportional gain */
    p->Ki = Ki;		/* integral gain */
    p->Kd = Kd;		/* derivative gain */

    p->tolerance = tolerance;

    p->error  = 0.0f;
    p->esum   = 0.0f;
    p->pvprev = 0.0f;
    p->cvi    = 0.0f;
    p->cvd    = 0.0f;
}


/*
 * Function:    fpid_calc()
 *
 * Synopsis:    float fpid_velocity_calc(p, setpoint, actual)
 *
 *              PID* p;         - Pointer to PID data structure.
 *              float setpoint;	- Desired setpoint value.
 *              float actual;   - Actual measured value from sensor.
 *
 * Description: This routine is designed to be called at fixed (or near fixed)
 *              time intervals either under a timer interrupt or from a foreground
 *              polling loop to perform PID calculations.
 *
 *              A digital controller measures the controlled variable at specific
 *              times, which are separated by a time interval called the sampling
 *              time, Delta T. The controller subtracts each sample of the measured
 *              variable from the setpoint to determine a set of error samples.
 *
 * Returns:     The PID output control variable (CV) value.
 */


float fpid_calc(FPID* p, float setpoint, float actual)
{
	float cv;	// return value

	/* Calculate the setpoint error */
	p->error = setpoint - actual;

	/* Error is within tolerance. */
	if (fabs(p->error) < p->tolerance)
		p->error = 0.0f;

	/* Compute proportional term
	 * Kp * E
	 */
	cv = p->Kp * p->error;

	/* Compute integral term by summing errors. */
	p->esum = p->esum + p->error;

	/* Limit integral term to CV range. */
	//long Ki = (PID_CVmax / p->Ki);

	/*if (p->esum > (float)DAC_MAX)
		p->esum = (float)DAC_MAX;
	else if (p->esum < 0.0f)
		p->esum = 0.0f;*/

	/* calculate the integral term
	 * Ki * Esum;
	 */
	p->cvi = p->Ki * p->esum;

	/* calculate the derivative term
	 * Kd * PVdelta
	 */
	p->cvd = p->Kd * (p->pvprev - actual);
	p->pvprev = actual;

	/* Add terms: P+I+D */
	cv = cv + p->cvi + p->cvd;

	/* Limit CV to allowed values */
	/*if (cv < 0.0f)
		cv = 0.0f;
	else if (cv > (float)DAC_MAX)
		cv = (float)DAC_MAX; */

	return cv;
}

/* end-of-file */
