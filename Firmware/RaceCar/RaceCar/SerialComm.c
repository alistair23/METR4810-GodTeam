/*
 * SerialComm.c
 *
 * Created: 30/03/2014 4:18:48 PM
 *  Author: Kianoosh
 *NOTE: SerialComm's send buffer has a limited size. If the rate of writing to this buffer is more than
 *the rate of transmission (which is controlled by baud rate) it will saturate the buffer and hog the CPU
 * Always make sure the rate with which the data is sent back to PC is always less than the transmission rate 
 */ 

#include "MotorControl.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <util/atomic.h>
#include "SerialComm.h"

extern volatile float error_l_sum; //integral error
extern volatile float error_r_sum;
extern volatile int8_t speed_l_measured;
extern volatile int8_t speed_r_measured;
int8_t speed_l_desired;
int8_t speed_r_desired;
uint8_t motor_enable;
uint8_t car_enable;
float kp;
uint8_t kd;
float ki;

static volatile uint8_t currentlyTransmitting = 0;
static volatile uint8_t updatingBuffer = 0;

//DEBUG
static volatile uint8_t print;

//for the length of string don't forget the null character
char mytext[13] = "Hello World\n"; 

typedef struct
{
	uint8_t Type;
	uint8_t Length; // 8-bit length value
	uint8_t *Data;
	
} Packet;
struct
{
	uint8_t Buffer[BUFFER_SIZE];
	uint8_t Start; 
	uint8_t End;
	uint16_t Count;
	
}Buffer;

static volatile uint8_t data_in;
static volatile uint8_t rx_stage = 0;
static volatile uint8_t data_size = 0;

static volatile Packet current_pckt;
static volatile uint8_t data_cursor = 0;
static volatile uint8_t pckt_queue_count = 0;
static volatile Packet pckt_queue[PCKT_QUEUE_SIZE];


enum PCKTCODE_IN
	{
		PCKTCODE_CONTROL_IN = 0x01,
		PCKTCODE_SETGAIN_IN,
		PCKTCODE_DEBUG
	};
//initialize USART
extern void SerialComm_initUSART(uint16_t ubrr)
{
	// Set baud rate
	UBRR0 = ubrr;
	// Enable receiver and transmitter
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0)|(1<<UDRIE0); 
	// Set frame format: 8 data bits, 1 stop bit
	UCSR0C =(1<<UCSZ00)|(1<<UCSZ01);
	Buffer.Start = 0;
	Buffer.End = 0;
	Buffer.Count = 0;
	
	sei();
}

uint16_t BufferGetCount()
{
	uint16_t size;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		size =  Buffer.Count;
	}
	return size;
}

int BufferIsFull()
{
	return BufferGetCount() ==  BUFFER_SIZE;
}

int BufferIsEmpty()
{
	return BufferGetCount() == 0;
}


void SendNextByte()
{
	//take next byte from the buffer and put it in UDR
	if (BufferIsEmpty()) 
	{
		currentlyTransmitting = 0;
	}
	else if (BufferGetCount() == 1)
	{
		currentlyTransmitting = 1;
		UDR0 = Buffer.Buffer[Buffer.Start];
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			
			Buffer.Count--;
		}
		
	}
	else
	{
		UDR0 = Buffer.Buffer[Buffer.Start];
		currentlyTransmitting = 1;
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			Buffer.Start = (Buffer.Start + 1) % BUFFER_SIZE;
			Buffer.Count--;
		}
		
	}
	
}



extern void EnqueueByte(uint8_t byte)
{
	while (BufferIsFull()) {}
	if(BufferIsEmpty())
	{
		
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			Buffer.Buffer[Buffer.End] = byte;
			//Buffer.End= (Buffer.End + 1) % BUFFER_SIZE;//changed
			Buffer.Count++;
		}
		
	}
	else
	{
		
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			Buffer.End= (Buffer.End + 1) % BUFFER_SIZE;
			Buffer.Buffer[Buffer.End] = byte;
			Buffer.Count++;
		}
		
	}
}



//send debug message
extern void SerialComm_sendText(char* msg)
{
	if (updatingBuffer) return;
	updatingBuffer = 1;
	for (int i = 0; msg[i]!=0; i++)
	{
		EnqueueByte(msg[i]);
	}
	EnqueueByte('\n');
	updatingBuffer = 0;
	
}

extern void SerialComm_sendTextf( char *fmt, ... )
{
	char buf[256];
	va_list arg;
	va_start (arg, fmt);
	vsprintf(buf, fmt, arg);
	va_end (arg);
	SerialComm_sendText(buf);
}

//send byte
extern void SerialComm_sendByte(uint8_t byte)
{
	updatingBuffer = 1;
	EnqueueByte(byte);	
	updatingBuffer = 0;
}

extern void SerialComm_sendData(void* data, uint8_t size)
{
	if(updatingBuffer)
	{ return;} //if buffer is being update don't do anything
	
	updatingBuffer = 1;
	
	uint8_t *databytes = data;
	for (int i = 0; i < size; i++)
	{
		EnqueueByte(databytes[i]); // send next data byte
	}
	updatingBuffer = 0;	
}

//read byte
void SerialComm_readByte()
{
	
}


//data process packet
void ProcessPacket(Packet pckt)
{
	
	uint8_t len = pckt.Length;
	SerialComm_sendData(current_pckt.Data, current_pckt.Length);
	SerialComm_sendText("\n");
	uint8_t reset_l = 0;
	uint8_t reset_r = 0;
	switch(pckt.Type)
	{
		case PCKTCODE_CONTROL_IN:
		
		if (len >= 2)
		{
			if (speed_l_desired != (int8_t) (*(pckt.Data)) )
				reset_l = 1;
			else
				reset_l = 0;
			if (speed_r_desired != (int8_t) (*(pckt.Data + 1)) )
				reset_r = 1;
			else
				reset_r = 0;

			speed_l_desired = (int8_t) (*(pckt.Data));
			speed_r_desired = (int8_t) (*(pckt.Data+1));
			
			if (reset_l)
				error_l_sum = (speed_l_desired - speed_l_measured) * MEASURE_PERIOD;
			if (reset_r)
				error_r_sum = (speed_r_desired - speed_r_measured) * MEASURE_PERIOD;
		}
		break;

		case PCKTCODE_SETGAIN_IN:
		if (len >=3)
		{
			kp = ((float) (*pckt.Data)/100);
			kd = *(pckt.Data + 1);
			ki =  ((float) (*(pckt.Data + 2))/100);
			SerialComm_sendText("gains are set now\n");
			
		}
		break;
		
		case PCKTCODE_DEBUG:
		SerialComm_sendData(pckt.Data, pckt.Length); //echo back the debug
		break;

		
	}
	free(pckt.Data);
	pckt.Type = 0;
	pckt.Length = 0;
	pckt.Data = NULL;
	
}
//data process packet
extern void SerialComm_ProcessPackets()
{
	
	if(pckt_queue_count == 0) return;
	else
	{
		
		while(pckt_queue_count>0)
		{
			SerialComm_sendText("Setting Speed...");
			ProcessPacket(pckt_queue[pckt_queue_count-1]);
			pckt_queue_count--;
		}			
	}
	
	
}

//Debug

uint8_t SerialComm_canprint()
{
	return print;
}


//data coming from PC would be:
// speed of each motor, start, stop command
//and debug command requesting encoder readings, etc

ISR(USART_UDRE_vect)
{
	SendNextByte();
	//UDR0 = 97;
}

ISR(USART_RX_vect)
{
	//incoming data will include:
	// go status, left motor speed, right motor speed
	// 1 byte,      1 byte        ,  1 byte
	// but we would like debug functions to set gains
	// take packet id, parameters ie
	//take PKT_SETGAIN, kp, kd, ki
	//take PKT_SETSPEED, gosignal, vleft, vright
	uint8_t byte = UDR0; //must read the byte to clear interrupt bit
	//SerialComm_sendByte(byte);
	//PORTB^=0xff; //for debug
	
    //SerialComm_SendTextf("This is the byte: %d", byte);
	//SerialComm_sendText(mytext);
	//SerialComm_sendText(mytext);
	//SerialComm_sendText(mytext);
	//uint8_t d[2] = {6,7};
	//SerialComm_sendData(d, 2);	 
	//SerialComm_sendByte(MotorControl_GetSpeed(MOTOR_R));
	//speed_l_desired = (int8_t) byte;
	SerialComm_sendTextf("L : %d\n", speed_l_desired);
	
	switch (rx_stage)
	{
		case 0 :
			current_pckt.Type = byte;
			SerialComm_sendTextf("Type: %d", current_pckt.Type);
			rx_stage++;
			
		break;
		
		case 1 :
			current_pckt.Length = byte; 
			current_pckt.Data = malloc(byte);
			rx_stage++;
			break;
		
		case 2:
			current_pckt.Data[data_cursor] = byte;
			//if (rx_stage == 2)
				//speed_l_desired = byte;
			data_cursor++;
			if (data_cursor == current_pckt.Length && pckt_queue_count < PCKT_QUEUE_SIZE)
			{
				pckt_queue[pckt_queue_count] = current_pckt;
				pckt_queue_count++;
				data_cursor = 0;
				rx_stage = 0;
				//SerialComm_sendData(current_pckt.Data, current_pckt.Length);
				//SerialComm_sendText("\n");
			}
			break;
	}		
	SerialComm_sendText("rx_stage:");
	SerialComm_sendByte(rx_stage);
	
	//based on Packet type set the variables and propagate them in
	//main loop. Variables should be extern so they can be accessed in the 
	//main loop
	
}



