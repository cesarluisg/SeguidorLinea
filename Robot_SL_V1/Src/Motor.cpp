/*
 * Motor.cpp
 *
 *  Created on: 4 ago. 2018
 *      Author: CesarLuis
 */

#include "Motor.h"

Motor::Motor() {
	// TODO Auto-generated constructor stub

}

Motor::~Motor() {
	// TODO Auto-generated destructor stub
}


enum_motor_error Motor::setPotencia(unsigned short int potencia)
{
	return MOTOR_SUCCESS;
}

enum_motor_error Motor::setFreno(enum_motor_freno freno)
{
	return MOTOR_SUCCESS;
}

enum_motor_error setMotorNumber(int motorId)
{
	return MOTOR_SUCCESS;
}
