/*
 * motorControl.h
 *
 * Created: 30/03/2014 4:16:21 PM
 *  Author: Kianoosh
 */ 

#ifdef F_CPU
#undef F_CPU 
#define F_CPU 8000000UL
#else
#define F_CPU 8000000UL
#endif

#ifndef MOTORCONTROL_H_
#define MOTORCONTROL_H_
//#define F_CPU 1000000UL
#include <stdint.h>
#define MOTOR_L (0)
#define MOTOR_R (1)
#define MAX_SPEED (825) //measured from oscilloscope with voltage 6.5V
#define GEAR_RATIO (30)
#define ENCODER_FINS (3)
#define MEASURE_PERIOD (4 * 65535.0 / (F_CPU))

// initialize PWM
extern void MotorControl_InitMotorControl();

//function to set motor speed on each side
//choose scale between 0-100 and direction so -100to100
extern void MotorControl_SetMotorSpeed(uint8_t motor, int speed);

extern int MotorControl_GetSpeed(uint8_t motor);
extern void MotorControl_CountEncoder();

#endif /* MOTORCONTROL_H_ */