#ifndef _PID_H_
#define _PID_H_


extern PIDInit( double dt, double max, double min, double Kp, double Kd, double Ki );
extern double calculate( double setpoint, double pv );

struct _PID_s {
	double _dt;
	double _max;
	double _min;
	double _Kp;
	double _Kd;
	double _Ki;
	double _pre_error;
	double _integral;
};

typedef _PID_s PID_s;  

#endif
