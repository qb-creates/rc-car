#ifndef MOTOR_H
#define MOTOR_H

#include "stdint.h"

extern bool reverseMotor;
extern bool motorStoped;
void configureMotorPWM(void);
void configureSteeringPWM(void);
void stopMotor(void);
void setMotorSpeed(int8_t speed);
void setMotorDirection(bool forward);

#endif