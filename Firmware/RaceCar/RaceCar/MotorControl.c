/*
 * MotorControl.c
 *
 * Created: 30/03/2014 4:19:12 PM
 *  Author: Kianoosh
 
 NOTE: WATCH OUT ABOUT THE PRE-SCLAERS CHOSEN FOR EACH TIMER. MAKE SURE THEY SERVE THE PURPOSE.
 DO CALCULATIONS FOR EACH AND FINALLY CHOOSE A PROPER PRESCALER
 */ 

#include "MotorControl.h"
#include "SerialComm.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#define F_CPU 1000000UL

static volatile uint32_t period_l = 0;
static volatile int8_t direction_l = 0;
static volatile uint32_t period_r = 0;
static volatile int8_t direction_r = 0;
static volatile int8_t speed_l_measured = 0;
static volatile int8_t speed_r_measured = 0;
uint8_t pinc_2;
uint8_t pinc_4;
uint8_t pinc_3;
uint8_t pinc_5;

static volatile uint8_t PINC2_Previous = 0;
static volatile uint8_t PINC4_Previous = 0;
static volatile uint16_t overflow1L = 0;
static volatile uint16_t overflow1R = 0;
static volatile uint16_t prev_tcntL = 0;
static volatile uint16_t curr_tcntL = 0;
static volatile uint16_t prev_tcntR = 0;
static volatile uint16_t curr_tcntR = 0;

static volatile int error_l = 0; //speed error in current cycle
static volatile int error_l_sum = 0; //integral error
static volatile int error_l_der = 0; //derivative error
static volatile int error_l_previous = 0;

static volatile int error_r = 0; //speed error in current cycle
static volatile int error_r_sum = 0; //integral error
static volatile int error_r_der = 0; // derivative error
static volatile int error_r_previous = 0;


extern void MotorControl_initMotorControl()
{
	// initialize PWM
	//phase correct pwm
	//count from 0x00 to 0xff
	//clear OC0A on match when upcounting
	//set OC0A on match when downcounting
	//frequency of pwm will be f_clock/(512 * prescale) 
	TCCR0A = (1<<COM0A1) | (1<<COM0B1) | (1<<WGM00); 
	TCCR0B  = (1<<CS00); //prescaler = 1
	TCCR2A = (1<<COM2A1) | (1<<COM2B1) | (1<<WGM20);
	TCCR2B  = (1<<CS20);
	// Set pins as outputs (OC0A==PD6, OC0B==PD5, OC2A==PB3, OC2B==PD3)
	DDRB |= (1<<DDB3);
	DDRD |= (1<<DDD3)|(1<<DDD5)|(1<<DDD6);
			
	// set duty cycle = 0
	OCR0A = 0;
	OCR0B = 0;
	OCR2A = 0;
	OCR2B = 0;
	
	//init encoder reading
	//Alistair wants them on  PC2 == PCINT10,PC3 == PCINT11, PC4 == PCINT12, PC5 == PCINT13
	//put PC2(right motor) and PC4(left motor) for forwrd
	//put PC3 and PC5 for reverse
	PCIFR = (1<<PCIF1);
	PCICR = (1<<PCIE1);
	PCMSK1 = (1<<PCINT12)|(1<<PCINT10);
	
	//init timer1 for speed calculation
	//TCCR1A = ;
	TCCR1B = (1<< CS10); //prescaler = 1
	TIMSK1 = (1<<TOIE0); //timer overflow enable
	//TCCR1C = ;
	
}
//function to set motor speed on each side
//choose scale between 0-100 and direction
//make sure you define MAX_SPEED later and use that for finding the scale
extern void MotorControl_setMotorSpeed(uint8_t motor, int speed)
{
	
	if (speed > 100) speed = 100;
	if (speed < -100) speed = -100;
	uint8_t forward = speed>0;
	//transform speed scale to pwm counter number
	//if duty cycle d is desired the counter has to be set to:
	// x = 2d*0xff/(1+d) where 0<=d<=1
	int absspeed = speed>=0 ? speed : -speed; 
	uint8_t u8speed = (uint8_t) (absspeed*255/100); 
	
	if (motor == MOTOR_L) //if left motor
	{
		OCR0A = forward ? u8speed : 0;
		OCR0B = forward ? 0 : u8speed;
		
	}
	else // if right motor
	{
		OCR2A = forward ? u8speed : 0;
		OCR2B = forward ? 0 : u8speed;
		
	}	
}

//this function returns motor speed from scale -100 to 100
extern int MotorControl_GetSpeed(uint8_t motor)
{	
	if (motor == MOTOR_L)
	{
		return speed_l_measured; 
	}
	
	else if (motor == MOTOR_R)
	{
		return speed_r_measured;
	}
}

//use external interrupts to measure encoder ticks
//Alistair wants them on  PC2 == PCINT10,PC3 == PCINT11, PC4 == PCINT12, PC5 == PCINT13
//put PC2(right motor) and PC4(left motor) for forward
//put PC3 and PC5 for reverse
ISR(PCINT1_vect)
{
	pinc_2 = (PINC & (1<<PINC2))>>PINC2 == 1;
	pinc_4 = (PINC & (1<<PINC4))>> PINC4 == 1;
	pinc_3 = (PINC & (1<<PINC3))>>PINC3 == 1;
	pinc_5 = (PINC & (1<<PINC5))>>PINC5 == 1;
	PORTB^=0b00010000;
	//SerialComm_sendByte(pinc_2);
	//SerialComm_sendByte(97);
	if (pinc_2 == 1 && PINC2_Previous == 0) //check if it's rising edge
	{
		if(pinc_3 == 0) //if other pin is low, PC2 is ahead
		{
			direction_r = 1;  // forward
		}
		else //if other pin is high already, PC2 is behind
		{
			direction_r = -1; //reverse
		}
		if(overflow1R < 20*F_CPU/65535)
		{
			//time between two rising edge is one period
			curr_tcntR = TCNT1;
			period_r = (overflow1R * 65535) + curr_tcntR - prev_tcntR;// ticks
			speed_r_measured = (int8_t)(100*60*direction_r/(period_r*MAX_SPEED/F_CPU));
			overflow1R = 0;
			prev_tcntR = curr_tcntR;
			
					
		}
		else //the time between two encoder ticks is too long ie motor is very slow
		{
			speed_r_measured = 0;
			
		}

		SerialComm_sendTextf("speed: %d", speed_r_measured);
		
	}
	
	if (pinc_4 == 1 && PINC4_Previous == 0)
	{
		if(pinc_5 == 0)
		{
			direction_l = 1;
		}
		else
		{
			direction_l = -1;
		}
		
		if(overflow1L < 20* F_CPU/65535)
		{
			//time between two rising edge is one period
			period_l = (overflow1L * 65535) + (curr_tcntL - prev_tcntL); //ticks
			speed_l_measured = (int)(100*60*direction_l/(period_l*MAX_SPEED/F_CPU)); //speed -100 to 100
			overflow1L = 0;
			prev_tcntL = curr_tcntL;
			
				
		}
		else//the time between two encoder ticks is too long ie motor is very slow
		speed_l_measured = 0;
	}
	
	PINC2_Previous = pinc_2;
	PINC4_Previous = pinc_4;
	PCIFR = (1<<PCIF1);
	
}



ISR(TIMER1_OVF_vect)
{
	overflow1R++;
	overflow1L++;
	//write the motor control in interrupt 
	//so the control loop is executed in regular intervals
	error_l = speed_l_desired - speed_l_measured;
	error_r = speed_r_desired - speed_r_measured;
	
	//TODO: error_sums may need to be reset to zero
	error_l_sum += error_l;
	error_r_sum += error_r;
	error_l_der = error_l - error_l_previous;
	error_r_der = error_r - error_r_previous;
	MotorControl_setMotorSpeed(MOTOR_L, kp*error_l+ki * error_l_sum + kd * error_l_der );
	MotorControl_setMotorSpeed(MOTOR_R, kp*error_r+ki * error_r_sum + kd * error_r_der );
	
	error_l_previous = error_l;
	error_r_previous = error_r;
	
	
	
}