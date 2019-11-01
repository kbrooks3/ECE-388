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
char scl[4];

int main(void)
{
   	DDRD = DDRD | 0b11110000;	//d4-7 output to screen
   	//DDRB = DDRB | 0b11111111;	//NEEDS TO BE CHANGED FOR ENCODER AND SERVO
   	DDRC = DDRC | 0b00010111;	//c[0-2] (RS, RW, and E) and 4 (Trig) set to output
	DDRC = DDRC & 0b11010111;	//c3 (Switch) and 5 (Echo) set to input

	//Timer for sensor
	TCCR1A = (0b00<<COM1A0)|(0b00<<COM1B0)|(0b00<<WGM10);	//Normal Mode
	TCCR1B = (0b0<<WGM12)|(0b010<<CS10);			//clock scalar by 8
	TIMSK1 = (0b0<<TOIE1);					//disable interrupts for timer1
	
	sei();							//C function to enable global interrupts
	
    while (1) 
    {
	sprintf(strtarget,"%.4d in SCL",val);			//val becomes a string
	//The line below needs to be changed to include a character string for scl
	//Something like sprintf(stractual,"%.4d in %c", val, scl); 
	sprintf(stractual,"%.4d in WUT",val);			//val becomes a string
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
		//These values need to be changed to account for the increase in resolution and faster clock
	    	
		if (TCNT1 >= 2664)
		{
			val = 9999;
		}
		else
		{
			val = TCNT1/37;
		}
	
    
	_delay_ms(10);
	}
}




void lcdinitialize()	//starts the LCD
{
	PORTC = PORTC&(0<<2);	
	_delay_us(2000);
	lcdcommand(0x28);	//4 bit bus mode, 2 line display, 5x11 display
	lcdcommand(0x0E);	//display on, cursor on, blink off
	lcdcommand(0x01);	//clear display
	_delay_us(2000);
	lcdcommand(0x06);	//entry mode: cursor moves right, DDRAM increments, no display shift
}

void lcdcommand(char cmnd)
{
	PORTD = (PORTD&0x0F)|(cmnd&0xF0);	//cmnd sent 4 bits at a time because in 4 bit mode
	PORTC = 0b00000100;	//enable 1
	
	_delay_us(1);
	PORTC = 0b00000000;	//enable 0
	_delay_us(100);
	PORTD = (PORTD&0x0F)|(cmnd<<4);
	PORTC = 0b00000100;	//enable 1
	_delay_us(1);
	PORTC = 0b00000000;	//enable 0
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
	PORTC = 0b00000101;	//RS is 1
				//RW is 0
	 			//enable is 1
	_delay_us(1);
	PORTC = 0b00000001;	//enable is 0
	PORTD = (PORTD&0x0F)|(data<<4);
	PORTC = 0b00000101;
	_delay_us(1);
	PORTC = 0b00000001;
	_delay_us(100);
	
}
