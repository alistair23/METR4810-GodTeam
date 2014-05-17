/*
 * SerialComm.h
 *
 * Created: 30/03/2014 4:18:01 PM
 *  Author: Kianoosh
 */ 

#ifdef F_CPU
#undef F_CPU
#define F_CPU 8000000UL
#else
#define F_CPU 8000000UL
#endif

#ifndef SERIALCOMM_H_
#define SERIALCOMM_H_
#include <stdint.h>
//#define BAUD 2400
#define BAUD 9600
#define BUFFER_SIZE 512
#define PCKT_QUEUE_SIZE 8

extern int8_t speed_l_desired;
extern int8_t speed_r_desired;
extern uint8_t motor_enable;
extern float kp;
extern uint8_t kd;
extern float ki;

//initialize USART
extern void SerialComm_initUSART(uint16_t ubrr);

//send debug message
extern void SerialComm_sendText(char* msg);
extern void SerialComm_sendTextf(char* format, ...);
//send byte
extern void SerialComm_sendByte(uint8_t byte);

//read byte
extern void SerialComm_readByte();
extern void SerialComm_sendData(void* data, uint8_t size);

void sendNextByte();
//process received data
extern void SerialComm_ProcessPackets();
//Debug 
uint8_t SerialComm_canprint();


//parse data
//data coming from PC would be:
// speed of each motor, start, stop command
//and debug command requesting encoder readings, etc

#endif /* SERIALCOMM_H_ */