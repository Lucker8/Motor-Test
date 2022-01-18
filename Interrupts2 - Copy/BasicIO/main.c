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
	
	DDRB =0xFF; //all pins in group B set to output
	DDRD = 0x00; //all pins in group D set to input
	DDRC= 0xF0; //PC 0-3 are inputs since they are ADC, rest are outputs for LCD
	PORTC= 0x30; //enable pullups for I2C PC 4-5
	
	//setup
	
	initialise(); //initialise timers and ADC
	PWM_start();  //slowly ramp up motor voltage with PWM
	
	//set a low start RPM for motor
	
	float RPM_errorcorrection=0, I_errorcorrection=0,current_I,adc_time=0.000208,Kp,Ki,desired_RPM;


	
	OCR1A=0;
	
	while(1)
	{	
		desired_RPM=((desired_RPM_v/VREF)*RPM_MAX); //calculate new desired RPM from measurement over potentiometer
		if(desired_RPM_v<=0.06) desired_RPM=0; //if so low that it is measurement error, just set to 0
		desired_RPM=round(desired_RPM); //Round up
		
											
		if(RPMflag)
		{
			cli(); //disable interrupts so prints are not interferred
			LCD_set_cursor(0,1); //set print positions and print values
			printf("Current RPM: %d",RPM_current);
			LCD_set_cursor(0,0);
			printf("Desired RPM: %.0f   ",desired_RPM);
			LCD_set_cursor(0,2);
			printf("Duty cycle: %.2f ",OCR1A/OCR1A_MAX);
			//calculate PID error and scale to system with maximum pulsewidth divided by maximum RPM
			RPM_errorcorrection=PID_RPM(RPM_current, desired_RPM,timer2comp*0.016)*OCR1A_MAX/RPM_MAX; 
			//if for some reason error is greater or less than PWM max or minimum,
			//just set it to max or minimum.
			if((OCR1A+round(RPM_errorcorrection))>OCR1A_MAX) 
			{
				OCR1A=OCR1A_MAX;
			}
			else if((OCR1A+round(RPM_errorcorrection))<OCR1A_MIN)
			{
				OCR1A=OCR1A_MIN;
			}
			else OCR1A+=round(RPM_errorcorrection);
			
			sei(); //re-enable interrupts
			RPMflag=0; //clear flag so PID controller will not run before another RPM measurement
			
		}	
	}
}

ISR(TIMER2_COMPA_vect) //overflows every 0.016s
{
	counttim2++; //count overflows
	
	sei();
	if(counttim2>=timer2comp) //if appropriate amount of time has passed, calculate RPM
	{
		//RPMoverflows+current count divided by time interval divided by 60 to go from RPS to RPM
		//further divided by 2, because 2 pole pairs
		RPM_current=(((256*RPMOVFL+TCNT0)/(timer2comp*0.016))*60)/2; 
		//calculate new appropriate time based on current measure RPM
		timer2comp=round(-0.0138*RPM_current+60.62);
		RPMflag=1; //Since an RPM measurement has been made, PID controller can be run again
		RPMOVFL=0; //reset all counters and timers (except PWM)
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
	RPMOVFL++; //keeps track of number of overflows of rotation counter
}

void PWM_start(void)
{
	OCR1A=0;
	LCD_set_cursor(0,0);
	printf("I'm in setup");
	for(int i=0;i<10;i++) //slowly ramp up motor voltage
	{
		
		OCR1A++;
		_delay_ms(50);
	}
	LCD_clear();
}

