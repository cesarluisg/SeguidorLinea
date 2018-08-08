#ifndef _PID_SOURCE_
#define _PID_SOURCE_


#include <stdio.h>
#include <string.h>
#include "pid.h"

//using namespace std;

#define PID_MAX 3

PID_s _pid[PID_MAX];
int _pidCount;

/**
 * Init
 */
extern void pidInit( void )
{
	_pidCount = 0;

	memset(&_pid, 0, sizeof(_pid));

}

extern int pidGetPID( PID_s **pid )
{
	if (_pidCount < PID_MAX)
	{
		*pid = &_pid[_pidCount];
		_pidCount++;
		return true;
	}
	return false;
}


extern void pidSet( PID_s *pid, double dt, double max, double min, double Kp, double Kd, double Ki, double pre_error, double integral)
{
	pid->dt = dt;
	pid->max = max;
	pid->min = min;
	pid->Kp = Kp;
	pid->Kd = Kd;
	pid->Ki = Ki;
	pid->pre_error = pre_error;
	pid->integral = integral;
}


extern double pidCalculate( PID_s *pid, double setpoint, double pv )
{
    
    // Calculate error
    double error = setpoint - pv;

    // Proportional term
    double Pout = pid->Kp * error;

    // Integral term
    pid->integral += error * pid->dt;
    double Iout = pid->Ki * pid->integral;

    // Derivative term
    if (0.0 == pid->dt) {
    	/* Y aca que hacemos? */
    	return 0;
    }
	double derivative = (error - pid->pre_error) / pid->dt;
	double Dout = pid->Kd * derivative;

    // Calculate total output PID
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    if( output > pid->max )
        output = pid->max;
    else if( output < pid->min )
        output = pid->min;

    // Save error to previous error
    pid->pre_error = error;

    return output;
}


#endif
