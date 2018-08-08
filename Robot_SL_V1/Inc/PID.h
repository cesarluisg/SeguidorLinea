#ifndef _PID_H_
#define _PID_H_

#include <stdbool.h>

typedef struct _PID_s {
	double dt;
	double max;
	double min;
	double Kp;
	double Kd;
	double Ki;
	double pre_error;
	double integral;
} PID_s;


extern void pidInit( void );
extern int pidGetPID( PID_s **pid );
extern void pidSet( PID_s *pid, double dt, double max, double min, double Kp, double Kd, double Ki, double pre_error, double integral);
extern double pidCalculate( PID_s *pid, double setpoint, double pv );

#endif
