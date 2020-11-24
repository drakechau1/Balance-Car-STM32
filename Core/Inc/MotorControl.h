/*
 * MotorControl.h
 *
 *  Created on: Nov 19, 2020
 *      Author: duc
 */

#ifndef INC_MOTORCONTROL_H_
#define INC_MOTORCONTROL_H_

#include "main.h"
#include <stdlib.h>

void initMotor(void);
void stopMotor(void);
void turnMotor_L(int32_t);
void turnMotor_R(int32_t);
#endif /* INC_MOTORCONTROL_H_ */
