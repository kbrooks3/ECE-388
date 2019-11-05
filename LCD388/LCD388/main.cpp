/*
Updated 11/5/19 to add all of the code for the peripherals.
The only glaring thing that needs to be edited is the 
 */ 
#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#define F_CPU 16000000
#include <util/delay.h>

#define LEFT 3200 	//timer length
#define RIGHT 2400 	//timer length
#define CENTER 2800 	//timer length
#define DSCALER		//still need to find a scalar
#define TOLERANCE 0.025 //inches

void lcdinitialize();
void lcdcommand(char cmnd);
void lcddata(char data);
void lcdprint(char * str );
void lcd_gotoxy(char x, char y);
void tilt(int deltax);
void angle(int pulseLength);
void sensortrigger();

volatile uint32_t actual; //inches
volatile uint32_t targetx; //inches
int val;			//current variable for distance, needs to be changed
char x, y;
char strtarget[20];
char stractual[20];
volatile uint32_t count = 0;
volatile uint32_t button = 0;
volatile uint32_t scale = 1; //change to data type with better precision 
char SCL[4];


int main(void)
{
   	DDRD = DDRD | 0b11110000;	//d4-7 output to screen
   	//DDRB = DDRB | 0b11111111;	//NEEDS TO BE CHANGED FOR ENCODER AND SERVO
   	DDRB = (DDRB & 0b11111010); 	//set b2 and b0 to input, connected to CLK and DT
	DDRC = DDRC | 0b00010111;	//c[0-2] (RS, RW, and E) and 4 (Trig) set to output
	DDRC = DDRC & 0b11010111;	//c3 (Switch) and 5 (Echo) set to input

	PCICR |= (1<<PCIE0); 
    	PCMSK0 |= (1<<1); //Pin B1 interrupt
    	PCICR |= (1<<PCIE1);
    	PCMSK1 |= (1<<4); //Pin C4 interrupt

	//Timer for sensor, changing from 1 to 3
	TCCR3A = (0b00<<COM3A0)|(0b00<<COM3B0)|(0b00<<WGM30);	//Normal Mode
	TCCR3B = (0b0<<WGM32)|(0b010<<CS30);			//clock scalar by 8
	TIMSK3 = (0b0<<TOIE3);					//disable interrupts for timer3
	
	//Timer for PWM
	TCCR1A = (0b10 << COM1A0) | (0 << COM1B0) | (0b10 <<  WGM10); //
	TCCR1B = (0b11 << WGM12) | (0b010 << CS10);
	ICR1 = 40000; 						// 20ms with /8 prescalar
	OCR1A = 1000; 						// 1000 -> 4000 (0.5 ms - 2 ms) // sets the duty cycle

	sei();							//C function to enable global interrupts
	
    while (1) 
    {
	sprintf(strtarget,"%.4d in SCL",val);			//val becomes a string
	sprintf(stractual,"%.4d in %c",actual, SCL);			//val becomes a string
	lcdinitialize();
	lcd_gotoxy(1,1);						//write on the first line
	lcdprint(strtarget);							//that string is sent to the LCD
	lcd_gotoxy(1,2);						//write on the second line
	lcdprint(stractual);
	sensortrigger();
	
	
			
		while ((PINC&(0b00010000))==0b00000000)
		{
			TCNT3 = 0;
		}
		while ((PINC&(0b00010000))==0b00010000)
		{
			
		}
		//These values need to be changed to account for the increase in resolution and faster clock
	    	
		if (TCNT3 >= 2664)
		{
			actual = 9999;
		}
		else
		{
			actual = TCNT3/37;
		}
	
    	deltax = target - actual;
	tilt(deltax);
	_delay_ms(10);
	}
}


void sensortrigger()
{
	PORTC = 0b00100000;	//trigger for sensor
	_delay_us(10);
	PORTC = 0b00000000;	//turn off trigger for sensor
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

ISR(PCINT0_vect) //Pin B2 (rotation) interrupt
{
    if ((PINB & 0b00000100) == 0b00000000) //check falling edge
    {
        if ((PINB & 0b00000001) == 0b00000000) //counterclockwise
        {
            target = target - scale; //decrement by scale
        }
        else //clockwise
        {
            target = target + scale; //increment by scale
        }
    }
}

ISR(PCINT1_vect) //Pin C3 (button) interrupt
{
    if ((PINC & 0b00001000) == 0b00000000)
    {
        button = 1; // button is pressed
        // Change scale based on previous scale
        If (scale == 1){
            	scale = 0.25; 	// 1 / 4 inch scale
            	SCL = “1/4";
        }
        else if (scale == 0.25){
            	scale = 0.025; 	// 1 / 40 inch scale
            	SCL = “1/40";
        }
        else if (scale == 0.025){
            	scale = 1; 	// 1 inch scale
		SCL = “1”;
        }
    }
    else
    {
        button = 0;
    }
}

//Functions for Servo follow

void tilt(int deltax)
{
    if (deltax > 0)
    {
        angle(RIGHT)
        _delay_ms(abs(deltax)*DSCALER)
        angle(CENTER)
    }
    
    if (deltax < 0)
    {
        angle(LEFT)
        _delay_ms(abs(deltax)*DSCALER)
        angle(CENTER)
    }
}

void angle(int pulseLength)
{
   if (pulselength == LEFT)
   {
	   OCR1A = LEFT;	//change top value of timer to 3200 for 1.6 ms
   }
	else if (pulselength == RIGHT)
	{	
		OCR1A = RIGHT;	//change top value of timer to 2400 for 1.2 ms
	}
	else
	{
		OCR1A = CENTER;	//change top value of timer to 2800 for 1.4 ms
	}
}
