
/*
AS OF 11/14/19
THIS IS ITERATIVE CODE TO TRY TO GET EVERYTHING WORKING

	1. INCORPORATE THE MOTOR (TEST: DOES IT TURN AT ALL? CAN IT TURN APPROPRIATELY?)
	
  
 */ 

#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>

#define	LCD_DPRT  PORTD
#define	LCD_DDDR  DDRD
#define	LCD_DPIN  PIND
#define	LCD_CPRT  PORTC
#define	LCD_CDDR  DDRC
#define	LCD_CPIN  PINB
#define	LCD_RS  0
#define	LCD_RW  1
#define	LCD_EN  2

#define LEFT 3200 	//timer length
#define RIGHT 2400 	//timer length
#define CENTER 2800 	//timer length

uint16_t Distance=1234;
float sound = 13503.9;
float Reading; 
#define F_CPU 16000000
#include <util/delay.h>
#include <avr/interrupt.h>
char strtarget[20];
char stractual[20];
char strscale[4];


//volatile uint32_t targetx = 0; //inches
int targetx = 0;
volatile uint32_t button = 0;
volatile uint32_t scale = 1; //change to data type with better precision
char SCL[4];

void lcdCommand( unsigned char cmnd );
void lcdData( unsigned char data );
void lcd_init();
void lcd_gotoxy(unsigned char x, unsigned char y);
void lcd_print(char * str );

void tilt(int targetx);
void angle(int direction);

int main(void)
{
	uint16_t ticks;
	DDRC= 1<<5;
	DDRB |= 1<<1;//(DDRB | 0b00000010);	//b1 = PWM
	DDRB = 0<<0;
	DDRB = 0<<2;//(DDRB & 0b11111010); 	//set b2 and b0 to input, connected to CLK and DT
	char str[20]; 
	
	PCICR |= (1<<PCIE0);
		PCMSK0 |= (1<<0); //Pin B1 interrupt
		PCICR |= (1<<PCIE1);
		PCMSK1 |= (1<<3); //Pin C3 interrupt

	
	TCCR3A=(0b00<<COM3A0)|(0b00<<COM3B0)|(0b00<<WGM30);
	TCCR3B=(0b00<<WGM32)|(0b100<<CS30);
	TIMSK3 = (0b0<<TOIE3);
	
	TCCR1A = (0b10 << COM1A0) | (0 << COM1B0) | (0b10 <<  WGM10); //
	TCCR1B = (0b11 << WGM12) | (0b010 << CS10);
	ICR1 = 40000; 						// 20ms with /8 prescalar
	OCR1A = 1000; 						// 1000 -> 4000 (0.5 ms - 2 ms) // sets the duty cycle
	
	
	sei();
	
	while (1)
	{
		
		sprintf(strtarget,"%.4d inches SCL",targetx);			//val becomes a string
		sprintf(stractual,"%.4d inches ",Distance);			//val becomes a string
		//sprintf(strscale,"%.4d", SCL);
		//sprintf(SCL,"1");
		lcd_init();
		lcd_gotoxy(1,1);
		lcd_print(strtarget);							//that string is sent to the LCD
		lcd_gotoxy(1,2);						//write on the second line
		lcd_print(stractual);
		lcd_print(SCL);
		
		tilt(targetx);
		//angle(CENTER);
		
		
		PORTC=1<<PINC5;
		_delay_us(10);
		PORTC=0<<PINC5;

		while ((PINC&(0b00010000))<=0b00001111)   //   while (((PINC)&(1<<PINC4))==0)
		;
		TCNT3=0;
		
		while ((PINC&(0b00010000))>=0b00010000) //   while (((PINC)&(1<<PINC4))==1)
		;
		ticks=TCNT3;
		if(ticks >= 2375)
		{
			Distance = 9999;
		}
		else
		{
			Reading = ticks * (sound / 62500) / 2;
			Distance = Reading;
		}
		

		_delay_ms(10);
	}
}

void lcdCommand( unsigned char cmnd )
{
	LCD_DPRT = (LCD_DPRT&0x0F)|(cmnd & 0xF0);
	LCD_CPRT &= ~ (1<<LCD_RS);
	LCD_CPRT &= ~ (1<<LCD_RW);
	LCD_CPRT |= (1<<LCD_EN);
	_delay_us(1);
	LCD_CPRT &= ~ (1<<LCD_EN);
	_delay_us(100);
	LCD_DPRT = (LCD_DPRT&0x0F)|(cmnd<<4);
	LCD_CPRT |= (1<<LCD_EN);
	_delay_us(1);
	LCD_CPRT &= ~ (1<<LCD_EN);
	_delay_us(100);
}

void lcdData( unsigned char data )
{
	LCD_DPRT = (LCD_DPRT&0x0F)|(data & 0xF0);
	LCD_CPRT |= (1<<LCD_RS);
	LCD_CPRT &= ~ (1<<LCD_RW);
	LCD_CPRT |= (1<<LCD_EN);
	_delay_us(1);
	LCD_CPRT &= ~ (1<<LCD_EN);
	LCD_DPRT = (LCD_DPRT&0x0F)|(data<<4);
	LCD_CPRT |= (1<<LCD_EN);
	_delay_us(1);
	LCD_CPRT &= ~ (1<<LCD_EN);
	_delay_us(100);
}

void lcd_init()
{
	LCD_DDDR = 0xFF;
	LCD_CDDR = (LCD_CDDR | 0b00000111);
	LCD_CPRT &=~(1<<LCD_EN);
	lcdCommand(0x33);
	lcdCommand(0x32);
	lcdCommand(0x28);
	lcdCommand(0x0e);
	lcdCommand(0x01);
	_delay_us(2000);
	lcdCommand(0x06);
}

void lcd_gotoxy(unsigned char x, unsigned char y)
{
	unsigned char firstCharAdr[]={0x80,0xC0,0x94,0xD4} ;

	lcdCommand(firstCharAdr[y-1] + x - 1);
	_delay_us(100);
}

void lcd_print(char * str )
{
	unsigned char i = 0 ;

	while(str[i]!=0)
	{
		lcdData(str[i]);
		i++ ;
	}
}

ISR(PCINT0_vect) //Pin B0 (rotation) interrupt
{
	if ((PINB & 0b00000100) == 0b00000000) //check falling edge
	{
		if ((PINB & 0b00000001) == 0b00000000) //counterclockwise
		{
			targetx = targetx - scale; //decrement by scale
		}
		else //clockwise
		{
			targetx = targetx + scale; //increment by scale
		}
	}
}

ISR(PCINT1_vect) //Pin C3 (button) interrupt
{
	if ((PINC & 0b00001000) == 0b00000000)
	{
		button = 1; // button is pressed
		// Change scale based on previous scale
		if (scale == 1){
			scale = 2; 	// 1 / 4 inch scale
			sprintf(SCL,"1/4");
		}
		else if (scale == 2){
			scale = 3; 	// 1 / 40 inch scale
			sprintf(SCL,"1/40");
		}
		else if (scale == 3){
			scale = 1; 	// 1 inch scale
			sprintf(SCL,"1");
		}
	}
	else
	{
		button = 0;
	}
}

void tilt(int targetx)
{
	if (targetx > 0)
	{
		angle(RIGHT);
		//_delay_ms(abs(deltax)*DSCALER);
		//angle(CENTER);
	}
	
	if (targetx < 0)
	{
		angle(LEFT);
		//_delay_ms(abs(deltax)*DSCALER);
		//angle(CENTER);
	}
}

void angle(int direction)
{
	if (direction == LEFT)
	{
		OCR1A = LEFT;	//change top value of timer to 3200 for 1.6 ms
	}
	else if (direction == RIGHT)
	{
		OCR1A = RIGHT;	//change top value of timer to 2400 for 1.2 ms
	}
	else
	{
		OCR1A = CENTER;	//change top value of timer to 2800 for 1.4 ms
	}
}