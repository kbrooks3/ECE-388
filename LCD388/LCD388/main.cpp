/*
 * Digital Tape Measure 2.c
 *
 * Created: 4/17/2019 12:52:39 PM
 * Author : kbrooks3
 */ 
#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 16000000
#include <util/delay.h>

void lcdinitialize();
void lcdcommand(char cmnd);
void lcddata(char data);
void lcdprint(char * str );
void lcd_gotoxy(char x, char y);

int val;
int dig;
char x, y;
char strtarget[20];
char stractual[20];

int main(void)
{
   	DDRD = DDRD | 0b11111111;								//ports are output
   	DDRB = DDRB | 0b11111111;
   	DDRC = DDRC | 0b00100000;

	//DDRC = DDRC | 0b00100000;
	
	
	//Timer 0 is to strobe the LED
	
	//Timer 1 is simply a counter
	//TCNT1 = 0;											//max value based on inches per cycle
	TCCR1A = (0b00<<COM1A0)|(0b00<<COM1B0)|(0b00<<WGM10);	//Normal Mode
	TCCR1B = (0b0<<WGM12)|(0b011<<CS10);					//clock scalar by 64
	TIMSK1 = (0b0<<TOIE1);									//disable interrupts for timer1
	
											//C function to enable global interrupts
	
    while (1) 
    {
	sprintf(strtarget,"%.4d inches SCL",val);			//val becomes a string
	sprintf(stractual,"%.4d inches WUT",val);			//val becomes a string
	lcdinitialize();
	lcd_gotoxy(1,1);						//write on the first line
	lcdprint(strtarget);							//that string is sent to the LCD
	lcd_gotoxy(1,2);						//write on the second line
	lcdprint(stractual);

	
	
		PORTC = 0b00100000;
		_delay_us(10);
		PORTC = 0b00000000;
		
		while ((PINC&(0b00010000))==0b00000000)
		{
			TCNT1 = 0;
		}
		while ((PINC&(0b00010000))==0b00010000)
		{
			
		}
		
		if (TCNT1 >= 2664)
		{
			val = 9999;
		}
		else
		{
			val = TCNT1/37;
		}
	
    
	_delay_ms(250);
	}
}




void lcdinitialize()	//starts the LCD
{
	PORTB = PORTB&(0<<2);	
	_delay_us(2000);
	//lcdcommand(0x33);	//8 bit bus mode
	//lcdcommand(0x32);	//8 bit bus mode
	lcdcommand(0x28);	//4 bit bus mode, 2 line display, 5x11 display
	lcdcommand(0x0E);	//display on, cursor on, blink off
	lcdcommand(0x01);	//clear display
	_delay_us(2000);
	lcdcommand(0x06);	//entry mode: cursor moves right, DDRAM increments, no display shift
}

void lcdcommand(char cmnd)
{
	PORTD = (PORTD&0x0F)|(cmnd&0xF0);	//cmnd sent 4 bits at a time because in 4 bit mode
	PORTB = 0b00000100;	//enable 1
	
	_delay_us(1);
	PORTB = 0b00000000;	//enable 0
	_delay_us(100);
	PORTD = (PORTD&0x0F)|(cmnd<<4);
	PORTB = 0b00000100;	//enable 1
	_delay_us(1);
	PORTB = 0b00000000;	//enable 0
	_delay_us(100);
	
}

void lcd_gotoxy(char x, char y)
{
	unsigned char firstCharAdr[]={0x80,0xC0,0x94,0xD4};//table 12-5
	lcdcommand(firstCharAdr[y-1] + x - 1);
	_delay_us(100);
}

void lcdprint( char * str )		//sets up array to print
{
	char i = 0 ;
	while(str[i]!=0)
	{
		lcddata(str[i]);	//sends string characters to lcddata one at a time
		i++ ;
	}
}

void lcddata(char data)
{
	PORTD = (PORTD&0x0F)|(data&0xF0);	//mask off high byte of character sent
	PORTB = 0b00000101;	//RS is 1
		//RW is 0
	 //enable is 1
	_delay_us(1);
	PORTB = 0b00000001;	//enable is 0
	PORTD = (PORTD&0x0F)|(data<<4);
	PORTB = 0b00000101;
	_delay_us(1);
	PORTB = 0b00000001;
	_delay_us(100);
	
}