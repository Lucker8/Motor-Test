#include <stdio.h>
#include <avr/io.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include "i2cmaster.h"
#include "basic.h"
#include "lcd.h"



void initialise(void)
{
  //counter 0 counts rotations of motor
  /*counter0setup();
  //timer 1 controls PWM output
  pwm1setup();
  //timer 2 measures time to calculate RPM from roations
  timer2setup();*/
  //ADC module set to free running mode, switching between two channels
  adc_setup();
  //OCR1A=0;
  

}

void adc_setup(void)
{
	cli();
	ADMUX=(1<<REFS0);  //sets Vref to Vcc
	ADCSRA=(1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2)|(1<<ADEN)|(1<<ADATE)|(1<<ADIE); //enable adc, set prescaler to 128, enable source trigger, enable interrupts

	sei();

	ADMUX &= 0xf0; // clears previously selected channel
	ADMUX |= 1; // set the desired channel
	//start the first conversion
	ADCSRA |= (1<<ADSC);
}

void counter0setup (void)
{

			//Setting up counter on timer 0
	DDRD &= ~(1 << DDD4); //clear PD4
	PORTD |= (1 << PORTD4);  // turn on pullup
	TIMSK0|=(1<<TOIE0); //enable interrupt on overflow
	TCCR0B |= (1 << CS02) | (1 << CS01) | (1 << CS00); //Turn on counter for rising edge
}

void timer2setup (void)
{
				//setting up timer on timer on timer 2
	TCCR2A|=(1<<WGM21); //CTC mode
	OCR2A=16000000/1024*0.016-1; //will trigger every 0.016s
	TIMSK2|=(1<<OCIE2A);
	TCCR2B|=(1<<CS22)|(1<<CS21)|(1<<CS20);
}


void pwm1setup(void)
{
				//Setting up PWM on timer 1
	TCCR1A|=(1<<COM1A1)|(1<<WGM11);
	//TIMSK1|=(1<<TOIE1);
	ICR1=70;
	OCR1A=0;
	TCCR1B|=(1<<WGM13)|(1<<WGM12)|(1<<CS11);
}
