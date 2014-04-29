/*
 * motorControl.h
 *
 * Created: 30/03/2014 4:16:21 PM
 *  Author: Kianoosh
 */ 


#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_
#include <stdint.h>
#define MOTOR_L (0)
#define MOTOR_R (1)
#define MAX_SPEED (2000)
// initialize PWM
extern void MotorControl_initMotorControl();

//function to set motor speed on each side
//choose scale between 0-100 and direction so -100to100
extern void MotorControl_setMotorSpeed(uint8_t motor, int speed);

extern int MotorControl_GetSpeed(uint8_t motor);

#endif /* MOTORCONTROL_H_ */