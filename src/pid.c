/*
    This file is part of AutoQuad.

    AutoQuad is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad.  If not, see <http://www.gnu.org/licenses/>.

    Copyright © 2011-2014  Bill Nesbitt
    Copyright 2016 Maxim Paperno
*/

#include "pid.h"
#include "aq.h"
#include "util.h"
#include "control.h"
#include "config.h"
#include "nav.h"
#include <stdlib.h>
#include <math.h>

pidStruct_t *pidInit(int pParam, int iParam, int dParam, int fParam, int pMaxParam, int iMaxParam, int dMaxParam, int oMaxParam) {
    pidStruct_t *pid;

    pid = (pidStruct_t *)aqDataCalloc(1, sizeof(pidStruct_t));

    pid->pParam = pParam;
    pid->iParam = iParam;
    pid->dParam = dParam;
    pid->fParam = fParam;
    pid->pMaxParam = pMaxParam;
    pid->iMaxParam = iMaxParam;
    pid->dMaxParam = dMaxParam;
    pid->oMaxParam = oMaxParam;

    return pid;
}

//float pidUpdate(pidStruct_t *pid, float setpoint, float position) {
//	float error;
//	float p = *pid->pGain;
//	float i = *pid->iGain;
//	float d = *pid->dGain;
//
//	if (pid->pTrim)
//		p += (*pid->pTrim * p / 500.0f);
//	if (pid->iTrim)
//		i += (*pid->iTrim * i / 500.0f);
//	if (pid->dTrim)
//		d += (*pid->dTrim * d / 500.0f);
//
//	error = setpoint - position;
//
//	// calculate the proportional term
//	pid->pTerm_1 = p * error;
//	if (pid->pTerm_1 > *pid->pMax) {
//		pid->pTerm_1 = *pid->pMax;
//	}
//	else if (pid->pTerm_1 < -*pid->pMax) {
//		pid->pTerm_1 = -*pid->pMax;
//	}
//
//	// calculate the integral state with appropriate limiting
//	pid->iState += error;
//	pid->iTerm_1 = i * pid->iState;
//	if (pid->iTerm_1 > *pid->iMax) {
//		pid->iTerm_1 = *pid->iMax;
//		pid->iState -= error;
//	}
//	else if (pid->iTerm_1 < -*pid->iMax) {
//		pid->iTerm_1 = -*pid->iMax;
//		pid->iState -= error;
//	}
//
//	// derivative
//	pid->dTerm_1 = (d * pid->fGain) * (error - pid->dState);
//	pid->dState += pid->fGain * (error - pid->dState);
//	if (pid->dTerm_1 > *pid->dMax) {
//		pid->dTerm_1 = *pid->dMax;
//	}
//	else if (pid->dTerm_1 < -*pid->dMax) {
//		pid->dTerm_1 = -*pid->dMax;
//	}
//
//	pid->pv_1 = position;
//	pid->sp_1 = setpoint;
//	pid->co_1 = pid->pTerm_1 + pid->iTerm_1 + pid->dTerm_1;
//
//	if (pid->co_1 > *pid->oMax) {
//		pid->co_1 = *pid->oMax;
//	}
//	else if (pid->co_1 < -*pid->oMax) {
//		pid->co_1 = -*pid->oMax;
//	}
//
//	return pid->co_1;
//}

float pidUpdate(pidStruct_t *pid, float setpoint, float position) {
    float error;
    float v, f, vMax, oMax;

    error = setpoint - position;

    // calculate the proportional term
    vMax = configGetParamValue(pid->pMaxParam);
    pid->pTerm_1 = constrainFloat(configGetParamValue(pid->pParam) * error, -vMax, vMax);

    // set output maximum to param value or pMax.
    oMax = pid->oMaxParam ? configGetParamValue(pid->oMaxParam) : vMax;

    // calculate the integral state with appropriate limiting
    if (pid->iParam) {
	v = configGetParamValue(pid->iParam);
	vMax = configGetParamValue(pid->iMaxParam);
	pid->iState += error;
	pid->iTerm_1 = v * pid->iState;
	if (fabsf(pid->iTerm_1) > vMax) {
	    pid->iTerm_1 = constrainFloat(pid->iTerm_1, -vMax, vMax);
	    pid->iState = pid->iTerm_1 / v;
	}
    } else
	pid->iTerm_1 = 0.0f;

    // derivative
    if (pid->dParam) {
	v = configGetParamValue(pid->dParam);
	vMax = configGetParamValue(pid->dMaxParam);
	f = pid->fParam ? configGetParamValue(pid->fParam) : 1.0f;
	// uncomment this line if you want the D term to ignore set point changes
	error = -position;

	pid->dTerm_1 = constrainFloat((v * f) * (error - pid->dState), -vMax, vMax);
	pid->dState += f * (error - pid->dState);
    }
    else
	pid->dTerm_1 = 0.0f;

    pid->pv_1 = position;
    pid->sp_1 = setpoint;
    pid->co_1 = constrainFloat(pid->pTerm_1 + pid->iTerm_1 + pid->dTerm_1, -oMax, oMax);

    return pid->co_1;
}

/*
float pidUpdate2(pidStruct_t *pid, float setpoint, float position) {
    float error;
    float p = *pid->pGain;
    float i = *pid->iGain;
    float d = (pid->dGain) ? *pid->dGain : 0.0f;
    float f = (pid->fGain) ? *pid->fGain : 1.0f;

    if (pid->pTrim)
	p += (*pid->pTrim * p * 0.002f);
    if (pid->iTrim)
	i += (*pid->iTrim * i * 0.002f);
    if (pid->dTrim)
	d += (*pid->dTrim * d * 0.002f);
    if (pid->fTrim)
	f += (*pid->fTrim * f * 0.002f);

    error = setpoint - position;

    // calculate the proportional term
    pid->pTerm_1 = p * error;
//    if (pid->pTerm_1 > *pid->pMax) {
//	    pid->pTerm_1 = *pid->pMax;
//    }
//    else if (pid->pTerm_1 < -*pid->pMax) {
//	    pid->pTerm_1 = -*pid->pMax;
//    }

    // calculate the integral state with appropriate limiting
    pid->iTerm_1 += i * error;
//    if (pid->iTerm_1 > *pid->iMax) {
//	    pid->iTerm_1 = *pid->iMax;
//	    pid->iState -= error;
//    }
//    else if (pid->iTerm_1 < -*pid->iMax) {
//	    pid->iTerm_1 = -*pid->iMax;
//	    pid->iState -= error;
//    }

    // derivative
    pid->dTerm_1 = (d * error - pid->dState) * f;
    pid->dState += pid->dTerm_1;

//    if (pid->dTerm_1 > *pid->dMax) {
//	    pid->dTerm_1 = *pid->dMax;
//    }
//    else if (pid->dTerm_1 < -*pid->dMax) {
//	    pid->dTerm_1 = -*pid->dMax;
//    }

    pid->pv_1 = position;
    pid->sp_1 = setpoint;
    pid->co_1 = pid->pTerm_1 + pid->iTerm_1 + pid->dTerm_1;

    if (pid->co_1 > *pid->oMax) {
	    pid->co_1 = *pid->oMax;
    }
    else if (pid->co_1 < -*pid->oMax) {
	    pid->co_1 = -*pid->oMax;
    }

    return pid->co_1;
}
*/

/*
// P & I tracking setpoint, D resisting change in position
float pidUpdate3(pidStruct_t *pid, float setpoint, float position) {
	float error;

	error = setpoint - position;

	// calculate the proportional term
	pid->pTerm_1 = pid->pGain * error;
	if (pid->pTerm_1 > pid->pMax) {
		pid->pTerm_1 = pid->pMax;
	}
	else if (pid->pTerm_1 < -pid->pMax) {
		pid->pTerm_1 = -pid->pMax;
	}

	// calculate the integral state with appropriate limiting
	pid->iState += error;
	pid->iTerm_1 = pid->iGain * pid->iState;
	if (pid->iTerm_1 > pid->iMax) {
		pid->iTerm_1 = pid->iMax;
		pid->iState -= error;
	}
	else if (pid->iTerm_1 < -pid->iMax) {
		pid->iTerm_1 = -pid->iMax;
		pid->iState -= error;
	}

	// derivative
//	pid->dTerm_1 = pid->dGain * (position - (2.0f * pid->pv_1) + pid->pv_2);
	pid->dTerm_1 = pid->dGain * (position - pid->pv_1);
	if (pid->dTerm_1 > pid->dMax) {
		pid->dTerm_1 = pid->dMax;
	}
	else if (pid->dTerm_1 < -pid->dMax) {
		pid->dTerm_1 = -pid->dMax;
	}

	pid->pv_2 = pid->pv_1;
	pid->pv_1 = position;
	pid->sp_1 = setpoint;
	pid->co_1 = pid->pTerm_1 + pid->iTerm_1 - pid->dTerm_1;

	if (pid->co_1 > pid->oMax) {
		pid->co_1 = pid->oMax;
	}
	else if (pid->co_1 < -pid->oMax) {
		pid->co_1 = -pid->oMax;
	}

	return pid->co_1;
}

// type B PID controller
float pidUpdateB(pidStruct_t *pid, float setpoint, float position) {
	float error;

	error = setpoint - position;

	// calculate the proportional term
	pid->pTerm_1 = pid->pGain * error;
	if (pid->pTerm_1 > pid->pMax) {
		pid->pTerm_1 = pid->pMax;
	}
	else if (pid->pTerm_1 < -pid->pMax) {
		pid->pTerm_1 = -pid->pMax;
	}

	// calculate the integral state with appropriate limiting
	pid->iState += error;
	pid->iTerm_1 = pid->iGain * pid->iState;
	if (pid->iTerm_1 > pid->iMax) {
		pid->iTerm_1 = pid->iMax;
		pid->iState -= error;
	}
	else if (pid->iTerm_1 < -pid->iMax) {
		pid->iTerm_1 = -pid->iMax;
		pid->iState -= error;
	}

	// derivative
	pid->dTerm_1 = pid->dGain * (position - (2.0f * pid->pv_1) + pid->pv_2);
	if (pid->dTerm_1 > pid->dMax) {
		pid->dTerm_1 = pid->dMax;
	}
	else if (pid->dTerm_1 < -pid->dMax) {
		pid->dTerm_1 = -pid->dMax;
	}

	pid->sp_1 = setpoint;
	pid->pv_2 = pid->pv_1;
	pid->pv_1 = position;
	pid->co_1 = pid->pTerm_1 + pid->iTerm_1 - pid->dTerm_1;

	if (pid->co_1 > pid->oMax) {
		pid->co_1 = pid->oMax;
	}
	else if (pid->co_1 < -pid->oMax) {
		pid->co_1 = -pid->oMax;
	}

	return pid->co_1;
}

// type C PID controller
float pidUpdateC(pidStruct_t *pid, float setpoint, float position) {
	float error;

	error = setpoint - position;

	pid->pTerm_1 = (position - pid->pv_1) * pid->pGain;
	if (pid->pTerm_1 > pid->pMax) {
		pid->pTerm_1 = pid->pMax;
	}
	else if (pid->pTerm_1 < -pid->pMax) {
		pid->pTerm_1 = -pid->pMax;
	}

	pid->iTerm_1 = error * pid->iGain;
	if (pid->iTerm_1 > pid->iMax) {
		pid->iTerm_1 = pid->iMax;
	}
	else if (pid->iTerm_1 < -pid->iMax) {
		pid->iTerm_1 = -pid->iMax;
	}

	pid->dTerm_1 = pid->dGain * (position - (2.0f * pid->pv_1) + pid->pv_2);
	if (pid->dTerm_1 > pid->dMax) {
		pid->dTerm_1 = pid->dMax;
	}
	else if (pid->dTerm_1 < -pid->dMax) {
		pid->dTerm_1 = -pid->dMax;
	}

	pid->sp_1 = setpoint;
	pid->pv_2 = pid->pv_1;
	pid->pv_1 = position;
	pid->co_1 = pid->co_1 - pid->pTerm_1 + pid->iTerm_1 - pid->dTerm_1;

	if (pid->co_1 > pid->oMax) {
		pid->co_1 = pid->oMax;
	}
	else if (pid->co_1 < -pid->oMax) {
		pid->co_1 = -pid->oMax;
	}

	return pid->co_1;
}
*/

void pidZeroIntegral(pidStruct_t *pid, float pv, float iState) {
    float i = pid->iParam ? configGetParamValue(pid->iParam) : 0.0f;
    if (i != 0.0f)
	pid->iState = iState / i;
    pid->dState = -pv;
    pid->sp_1 = pv;
    pid->co_1 = 0.0f;
    pid->pv_1 = pv;
    pid->pv_2 = pv;
}
