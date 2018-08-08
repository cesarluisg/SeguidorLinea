/*
 * Motor.h
 *
 *  Created on: 4 ago. 2018
 *      Author: CesarLuis
 */

#ifndef MOTOR_H_
#define MOTOR_H_

enum enum_motor_freno {MOTOR_FRENO_FRENAR, MOTOR_FRENO_LIBERAR};
enum enum_motor_error {MOTOR_SUCCESS}

class Motor {
private:
	unsigned short int potencia; //0 a 100
	enum_motor_freno freno;
public:
	Motor();
	virtual ~Motor();
	enum_motor_error setPotencia(unsigned short int potencia);
	enum_motor_error setFreno(enum_motor_freno freno); // MOTOR_FRENO_FRENAR deshabilita salida de potencia
};

#endif /* MOTOR_H_ */
