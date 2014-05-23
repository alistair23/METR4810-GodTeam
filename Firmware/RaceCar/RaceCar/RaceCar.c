/*
 * RaceCar.c
 *
 * Created: 30/03/2014 4:15:16 PM
 *  Author: Kianoosh
 */ 

#include "MotorControl.h"
#include <avr/io.h>
#include <util/delay.h>

#include "SerialComm.h"

//#define BAUD 2400
int main(void)
{
	//DDRD = 0xff;
	
	//MotorControl_setMotorSpeed(MOTOR_L, 50);
	//OCR0A = (uint8_t) (100*255/100);
	SerialComm_initUSART(F_CPU/16/BAUD - 1);
	MotorControl_InitMotorControl();
	speed_l_desired = 0;
	speed_r_desired = 0;
	//MotorControl_SetMotorSpeed(MOTOR_L, 70);
	//MotorControl_SetMotorSpeed(MOTOR_R, 70);
	//uint16_t ubrr = F_CPU/16UL/BAUD-1;
	//UBRR0H = (uint8_t)(ubrr >> 8);
	//UBRR0L = (uint8_t) ubrr;
	// Enable receiver and transmitter
	//UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<UDRIE0);;//|(1<<RXCIE0)|(1<<UDRIE0);
	// Set frame format: 8 data bits, 1 stop bit
	//UCSR0C = (1<<UCSZ00)|(1<<UCSZ01);

	DDRB = 0xff;
	//PORTB = 0xff;
	
		
    while(1)
    {
       
	   MotorControl_CountEncoder();
	   SerialComm_ProcessPackets();
	   
	   //PORTD= 0xff;
	  // _delay_ms(100);
	   //PORTD = 0;
	   //PORTB = 0;  
		//uint8_t s = MotorControl_GetSpeed(MOTOR_L);
		//SerialComm_sendByte(97);   
	   //PORTB=(1<<PORTB2);
	   //uint8_t byte = 97;
	   //SerialComm_sendByte(byte);
	   //while(!( UCSR0A & (1<<UDRE0))) {}
	   //UDR0 = 98;
	   //SerialComm_sendByte(97);
	   //_delay_ms(200);
    }
}