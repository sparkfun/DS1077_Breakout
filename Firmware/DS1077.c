 
/******************************************************************************
DS1077.c
DS1077 Demo Code
Viliam Klein@ SparkFun Electronics
11/21/2008
https://github.com/sparkfun/DS1077_Breakout

This is an example interface for the DS1077, using an ATMega168 at 8MHz. 

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "types.h"
#include "defs.h"
#include "i2c.h"

#define FOSC 8000000
#define BAUD 9600
#define MYUBRR 103

#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))

#define STATUS_LED 0
#define SLEEP 0

#define SLA_W 0xB0//write address
#define SLA_R 0xB1//read address

#define DIV 0x01
#define MUX 0x02
#define BUS 0x0D
#define E2 0x3F

//Define functions
//======================
void i2cSendStart(void);
void i2cSendStop(void);
void i2cWaitForComplete(void);
void i2cSendByte(unsigned char data);


void ioinit(void);      // initializes IO
static int uart_putchar(char c, FILE *stream);
uint8_t uart_getchar(void);

static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);

void delay_ms(uint16_t x); 
void delay_us(uint8_t x);

void two_byte_w(uint8_t cmd, uint8_t msb, uint8_t lsb);
void one_byte_w(uint8_t cmd, uint8_t msb);
void one_byte_r(uint8_t cmd);
void two_byte_r(uint8_t cmd);

uint8_t byte1 = 0x00;
uint8_t byte2 = 0x00;
//======================

int main (void)
{
	
	ioinit();
	

	//Set out1 to 40kHZ, and out2 to 133MHz
	//Also enables CTRL1 as a control pin for out1
	two_byte_w(MUX, 0x11, 0x00);
	//Sets N to create 40kHz wave on out1
	two_byte_w(DIV, 0xCF, 0xC0);
	//printf for debugging 
	two_byte_r(MUX);
	two_byte_r(DIV);
	
	return(0);
	
}

void one_byte_r(uint8_t cmd)
{
	i2cSendStart();		
    i2cWaitForComplete();
	delay_us(10);	
	printf("TWSR is: %x\n", (TWSR & 0xFC));
	
	
	// send slave device address with write
	i2cSendByte(SLA_W);	
	i2cWaitForComplete();
	delay_us(10);
	printf("TWSR is: %x\n", (TWSR & 0xFC));
	
	TWDR = cmd;
	TWCR = (1<<TWINT)|(1<<TWEN);	
	i2cWaitForComplete();
	delay_us(10);
	
	i2cSendStart();		
    i2cWaitForComplete();
	delay_us(10);	
	printf("TWSR is: %x\n", (TWSR & 0xFC));
	
	
	// send slave device address with write
	i2cSendByte(SLA_R);	
	i2cWaitForComplete();
	delay_us(10);
	printf("TWSR is: %x\n", (TWSR & 0xFC));
	
	if( inb(TWSR) == TW_MR_SLA_ACK)
	{

		i2cReceiveByte(TRUE);
		i2cWaitForComplete();
		delay_us(10);
		byte1 = i2cGetReceivedByte();
		printf("\tbyte1 is: %x\n", byte1);
		
		
		i2cReceiveByte(TRUE);
		i2cWaitForComplete();
		delay_us(10);
		byte2 = i2cGetReceivedByte();
		printf("\tbyte2 is: %x\n", byte2);
		
		
	}
	else
	{
			// device did not ACK it's address,
			// data will not be transferred
			// return error
			//retval = I2C_ERROR_NODEV;
	}
	
	delay_us(10);
    i2cSendStop();
	
}


void two_byte_r(uint8_t cmd)
{
	i2cSendStart();		
    i2cWaitForComplete();
	delay_us(10);	
	printf("TWSR is: %x\n", (TWSR & 0xFC));
	
	
	// send slave device address with write
	i2cSendByte(SLA_W);	
	i2cWaitForComplete();
	delay_us(10);
	printf("TWSR is: %x\n", (TWSR & 0xFC));
	
	TWDR = cmd;
	TWCR = (1<<TWINT)|(1<<TWEN);	
	i2cWaitForComplete();
	delay_us(10);
	
	i2cSendStart();		
    i2cWaitForComplete();
	delay_us(10);	
	printf("TWSR is: %x\n", (TWSR & 0xFC));
	
	
	// send slave device address with write
	i2cSendByte(SLA_R);	
	i2cWaitForComplete();
	delay_us(10);
	printf("TWSR is: %x\n", (TWSR & 0xFC));
	
	if( inb(TWSR) == TW_MR_SLA_ACK)
	{

		i2cReceiveByte(TRUE);
		i2cWaitForComplete();
		delay_us(10);
		byte1 = i2cGetReceivedByte();
		printf("\tbyte1 is: %x\n", byte1);
		
		
		i2cReceiveByte(TRUE);
		i2cWaitForComplete();
		delay_us(10);
		byte2 = i2cGetReceivedByte();
		printf("\tbyte2 is: %x\n", byte2);
		
		
	}
	else
	{
			// device did not ACK it's address,
			// data will not be transferred
			// return error
			//retval = I2C_ERROR_NODEV;
	}
	
	delay_us(10);
    i2cSendStop();
	
}


void one_byte_w(uint8_t cmd, uint8_t msb)
{
	TWCR = 0x00;
	TWBR = 64;
	//TWSR = (1 << TWPS1);
	cbi(TWCR, TWEA);	
	sbi(TWCR, TWEN);
	delay_us(10);
	
	//Send start condition 
	i2cSendStart();		
    i2cWaitForComplete();
	delay_us(10);	
	printf("TWSR is: %x\n", (TWSR & 0xFC));
	
	
	// send slave device address with write
	i2cSendByte(SLA_W);	
	i2cWaitForComplete();
	delay_us(10);
	printf("TWSR is: %x\n", (TWSR & 0xFC));
	
	TWDR = cmd;
	TWCR = (1<<TWINT)|(1<<TWEN);	
	i2cWaitForComplete();
	delay_us(10);
	
	TWDR = msb;
	TWCR = (1<<TWINT)|(1<<TWEN);	
	i2cWaitForComplete();
	delay_us(10);

	
	delay_us(10);
    i2cSendStop();
	
}


void two_byte_w(uint8_t cmd, uint8_t msb, uint8_t lsb)
{
	TWCR = 0x00;
	TWBR = 64;
	//TWSR = (1 << TWPS1);
	cbi(TWCR, TWEA);	
	sbi(TWCR, TWEN);
	delay_us(10);
	
	//Send start condition 
	i2cSendStart();		
    i2cWaitForComplete();
	delay_us(10);	
	printf("TWSR is: %x\n", (TWSR & 0xFC));
	
	
	// send slave device address with write
	i2cSendByte(SLA_W);	
	i2cWaitForComplete();
	delay_us(10);
	printf("TWSR is: %x\n", (TWSR & 0xFC));
	
	TWDR = cmd;
	TWCR = (1<<TWINT)|(1<<TWEN);	
	i2cWaitForComplete();
	delay_us(10);
	
	TWDR = msb;
	TWCR = (1<<TWINT)|(1<<TWEN);	
	i2cWaitForComplete();
	delay_us(10);
	
	TWDR = lsb;
	TWCR = (1<<TWINT)|(1<<TWEN);	
	i2cWaitForComplete();
	delay_us(10);
	
	delay_us(10);
    i2cSendStop();
	
}
	

void ioinit (void)
{
    //setup for UART and IO pins
	
	//1 = output, 0 = input
    DDRB = 0b11111111; //All outputs
    DDRC = 0b11111111; //All outputs
    DDRD = 0b11111110; //PORTD (RX on PD0)
	stdout = &mystdout; //Required for printf init
//	int MYUBRR = 103;

	UBRR0H = (MYUBRR) >> 8;
	UBRR0L = MYUBRR;

	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	UCSR0C = (3<<UCSZ00);
	UCSR0A = (1<<U2X0);

	//Init timer 0 for delay_us timing
	//8,000,000 / 8 = 1,000,000
    TCCR0B = (1<<CS01); //Set Prescaler to 8. CS01=1


}

static int uart_putchar(char c, FILE *stream)
{
    if (c == '\n') uart_putchar('\r', stream);
  
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
    
    return 0;
}

uint8_t uart_getchar(void)
{
    while( !(UCSR0A & (1<<RXC0)) );
    return(UDR0);
}

//General short delays
//general short delays
//Uses internal timer do a fairly accurate 1us
//Because we are using 16MHz and a prescalar of 8 on Timer0, we have to double x
void delay_us(uint8_t x)
{
	TCCR0B = 0x01;
	//TIFR = (1<<TOV0); //Clear any interrupt flags on Timer0
	TCNT0 = 0;
	
	//while ((TIFR & 0x01) == 0x00);
	while (TCNT0 < x);
	
	TCCR0B = 0x00;
	
}

//General short delays
void delay_ms(uint16_t x)
{
	for ( ; x > 0 ; x--)
		delay_us(255);
}


//==========================//
//
//
//
//
//
//
//
//
//I2C functions
//
//
//
//
//
//
//
//==========================//


void i2cSendStart(void)
{
	// send start condition
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
}

void i2cSendStop(void)
{
	// transmit stop condition
        TWCR = (1<<TWSTO);
}

void i2cWaitForComplete(void)
{
	// wait for i2c interface to complete operation
        while (!(TWCR & (1<<TWINT)));
}

void i2cSendByte(unsigned char data)
{
	// save data to the TWDR
	TWDR = data;
	// begin send
	TWCR = (1<<TWINT)|(1<<TWEN);
}

void i2cReceiveByte(unsigned char ackFlag)
{
	// begin receive over i2c
	if( ackFlag )
	{
		// ackFlag = TRUE: ACK the recevied data
		outb(TWCR, (inb(TWCR)&TWCR_CMD_MASK)|BV(TWINT)|BV(TWEA));
	}
	else
	{
		// ackFlag = FALSE: NACK the recevied data
		outb(TWCR, (inb(TWCR)&TWCR_CMD_MASK)|BV(TWINT));
	}
}

unsigned char i2cGetReceivedByte(void)
{
	// retieve received data byte from i2c TWDR
	return( inb(TWDR) );
}



