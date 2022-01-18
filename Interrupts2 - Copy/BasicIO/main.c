/*
 exam number: 493947
 */ 

 
 
 #define F_CPU 16000000UL

 #include <stdio.h>
 #include <avr/io.h>
 #include <util/delay.h>
 #include <stdlib.h>
 #include <avr/interrupt.h>
#include "i2cmaster.h"
#include "basic.h"
#include "usart.h"
#include "PID.h"
#include "lcd.h"

volatile unsigned RPMOVFL=0;
volatile int counttim2=0,RPM_current=0;
volatile char RPMflag=0, adc_flag=0;
volatile char timer2comp=1;
volatile float current_I_v=0;
volatile float desired_RPM_v=0; 
volatile uint8_t channel_f =1;

void PWM_start(void);

int main (void)
{
	/*uart_init();
	io_redirect();*/
	i2c_init();
	LCD_init();	
	
	DDRB =0xFF;
	DDRD = 0x00; //I/O board:PD4…7 as outputs, for LEDs
	DDRC= 0xF0;
	PORTC= 0x30;
	
	//setup
	
	initialise();
	PWM_start();
	
	//set a low start RPM for motor
	
	float RPM_errorcorrection=0, I_errorcorrection=0,current_I,adc_time=0.000208,Kp,Ki,desired_RPM;


	
	OCR1A=0;
	
	while(1)
	{	
		desired_RPM=((desired_RPM_v/VREF)*RPM_MAX);//-12.65; 
		if(desired_RPM_v<=0.06) desired_RPM=0;
		desired_RPM=round(desired_RPM);
		
											
		if(RPMflag)
		{
			cli();
			LCD_set_cursor(0,1);
			printf("Current RPM: %d",RPM_current);
			LCD_set_cursor(0,0);
			printf("Desired RPM: %.0f   ",desired_RPM);
			LCD_set_cursor(0,2);
			printf("Duty cycle: %.2f ",OCR1A/OCR1A_MAX);
			RPM_errorcorrection=PID_RPM(RPM_current, desired_RPM,timer2comp*0.016)*OCR1A_MAX/RPM_MAX; //8.7 max current;  70 OCR/8.7 A;
	
			if((OCR1A+round(RPM_errorcorrection))>OCR1A_MAX)
			{
				OCR1A=OCR1A_MAX;
			}
			else if((OCR1A+round(RPM_errorcorrection))<OCR1A_MIN)
			{
				OCR1A=OCR1A_MIN;
			}
			else OCR1A+=round(RPM_errorcorrection);
			
			sei();
			RPMflag=0;
			
		}	
	}
}

ISR(TIMER2_COMPA_vect) //overflows every 0.016s
{
	counttim2++;
	
	sei();
	if(counttim2>=timer2comp) //62 overflows gives 0.992s
	{
		RPM_current=(((256*RPMOVFL+TCNT0)/(timer2comp*0.016))*60)/2; //RPMoverflows+current count divided by time interval divided by 60 to go from RPS to RPM
		timer2comp=round(-0.0138*RPM_current+60.62);
		RPMflag=1;
		RPMOVFL=0;
		TCNT0=0;
		counttim2=0;
	}
}

ISR(ADC_vect)
{
	if(channel_f)			//channel flag=1-> ADC1 (A1)
	{
		desired_RPM_v=(ADC*VREF)/1024;
		if(desired_RPM_v<0) desired_RPM_v=0;
		//channel_f=0;   //current sensor doesn't work
	} 
	else
	{					//channel flag=0 -> ADC0 (A0)
		current_I_v=((ADC*VREF)/1024);
		channel_f=1;
	}
	ADMUX &= 0xf0; // clears previously selected channel
	ADMUX |= channel_f; // set the desired channel

}


ISR(TIMER0_OVF_vect)
{
	RPMOVFL++;
}

void PWM_start(void)
{
	OCR1A=0;
	LCD_set_cursor(0,0);
	printf("I'm in setup");
	for(int i=0;i<10;i++)
	{
		
		OCR1A++;
		_delay_ms(50);
	}
	LCD_clear();
}

