/*
 exam number: 493947
 */ 

 
 
 #define F_CPU 16000000UL
 #define V_REF 5.0

 #include <stdio.h>
 #include <avr/io.h>
 #include <util/delay.h>
 #include <stdlib.h>
 #include <avr/eeprom.h>
 #include <avr/interrupt.h>
#include "i2cmaster.h"
#include "ds1621.h"
#include "usart.h"
#include "PID.h"
#include "lcd.h"


volatile int counttim2=0;
volatile unsigned RPMOVFL=0;
volatile int RPMcurrent=0;
volatile float current=0;
volatile char RPMflag=0;
volatile char timer2comp=1;
volatile float current_I_v=0;
volatile float desired_RPM_v=0; 
volatile uint8_t channel_f =0;

void counter0setup (void);
void timer2setup (void);
void pwm1setup (void);

void adc_setup(void) 
{
	cli();
	ADMUX=(1<<REFS0);  //sets Vref to Vcc
	ADCSRA=(1<<ADPS0)|(1<<ADPS1)|(1<<ADPS2)|(1<<ADEN)|(1<<ADATE)|(1<<ADIE); //enable adc, set prescaler to 128, enable source trigger, enable interrupts
	
	sei();

	ADMUX &= 0xf0; // clears previously selected channel
	ADMUX |= channel_f; // set the desired channel
	//start the first conversion
	ADCSRA |= (1<<ADSC);
}

void init(void)
{
	OCR1A=0;
	for(int i=0;i<10;i++)
	{
		OCR1A++;
		_delay_ms(5);
	}
}


int main (void)
{
	uart_init();
	io_redirect();
	/*i2c_init();
	LCD_init();	*/
	
	DDRB =0xFF;
	DDRD = 0x00; //I/O board:PD4…7 as outputs, for LEDs
	DDRC= 0xF0;
	PORTC= 0x30;
	
	
	
	//setup and start all 3 timers
	//counter 0 counts rotations of motor
	//counter0setup(); 
	//timer 2 measures time to calculate RPM from roations
	//timer2setup();
	//timer 1 controls PWM output
	pwm1setup();
	adc_setup();
	init();
	//set a low start RPM for motor
	int *ptr,n=0,adc_flag=0,a=0;
	float RPMtarget=2000,RPM_errorcorrection=0, I_errorcorrection=0,current_I,adc_time=0.000208,desired_RPM,Kp,Ki;
	ptr = (int*) calloc(5, sizeof(int));
	if(ptr==NULL) return 0;


	
	OCR1A=55;
	//LCD_set_cursor(0,0);
	printf("im in setup");
	_delay_ms(500);
	//LCD_clear();
	while(1)
	{	
		Kp=	0.77;							//Kn
		Ki=Kp/(0.00143+4*timer2comp);		//Kn/Tn
		desired_RPM=((desired_RPM_v/4.9)*RPM_MAX)-12.65; //rename to V ref,might be 4.8	
		if(desired_RPM_v<=0.06) desired_RPM=0;												//MAYBE do both in ISR
		current_I=current_I_v  /0.57; //calculated from IV characteristic of our current sensing circuit
		*(ptr+n)=current_I;
		/*
		if(n==5) 
		{
			n=0;
			adc_flag=1;
		}
		else n++;
		if(adc_flag) {
			for(int i=0;i<5;i++) a+=*(ptr+i);
			printf("%.2f",a);
		}*/
		//LCD_set_cursor(0,0);  
		//printf("RPM: ");
		//LCD_set_cursor(0,1);
		//printf("Wanted RPM %.2f, RAW=%.2f \n\n",desired_RPM,desired_RPM_v);
		printf("Current=%.2f, |Raw=%.2f\n\n",current_I,current_I_v);
		
		if(RPMflag && adc_flag)
		{
			
			printf("%d\n",RPMcurrent);
			printf("%d\n",TCNT0);
			cli();
			/*RPM_errorcorrection=PID_RPM(RPMcurrent, RPMtarget,timer2comp*0.016)*I_MAX/RPM_MAX; //8.7 max current;  70 OCR/8.7 A;
			I_errorcorrection=PID_CURRENT(current_I,RPM_errorcorrection,adc_time)*(OCR1A_MAX/I_MAX);		//OCR based on current
			
			if((OCR1A+round(I_errorcorrection))>OCR1A_MAX)
			{
				OCR1A=OCR1A_MAX;
			}
			else if((OCR1A+round(I_errorcorrection))<OCR1A_MIN)
			{
				OCR1A=OCR1A_MIN;
			}
			else OCR1A+=round(I_errorcorrection);
			
			printf("error:%f OCR:%d\n", I_errorcorrection, OCR1A);
			
			scanf("%d", &RPMcurrent);
			*/
			sei();
			RPMflag=0;
		}	
	}
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
	sei();
	ICR1=70;
	OCR1A=0;
	TCCR1B|=(1<<WGM13)|(1<<WGM12)|(1<<CS11);
}

ISR(TIMER2_COMPA_vect) //overflows every 0.016s
{
	counttim2++;
	
	sei();
	if(counttim2>=(timer2comp)) //62 overflows gives 0.992s
	{
		RPMcurrent=(((256*RPMOVFL+TCNT0)/(timer2comp*0.016))*60)/2; //RPMoverflows+current count divided by time interval divided by 60 to go from RPS to RPM
		timer2comp=round(0.0138*RPMcurrent+60.62);
		RPMflag=1;
		RPMOVFL=0;
		TCNT0=0;
		counttim2=0;
	}
}

ISR(TIMER0_OVF_vect)
{
	RPMOVFL++;
}

ISR(ADC_vect)
{
	if(channel_f)
	{
		desired_RPM_v=(ADC*4.9)/1024;
		if(desired_RPM_v<0) desired_RPM_v=0;
		channel_f=0;
	}
	else
	{
		current_I_v=((ADC*4.9)/1024);
		channel_f=1;
	}
	ADMUX &= 0xf0; // clears previously selected channel
	ADMUX |= channel_f; // set the desired channel

}


/*leftover code
//TCCR0A|=(1<<COM0A1)|(1<<COM0A0)|(1<<WGM00)|(1<<WGM01); //set timer 0 to CTC
//TCCR0B|=(1<<WGM02)|(1<<CS01)|(1<<CS02);
//OCR0A=255;

delay_ms(100);
		cli();
		dutycycle=(float)resulthigh/(resulthigh+resultlow);
		printf("high=%d low=%d dutycycle=%f\n", resulthigh, resultlow, dutycycle);
		sei();
		_delay_ms(1);
		OCR1A++;
		if(OCR1A>34)
		{
			OCR1A=0;
		}	
		ISR(TIMER0_COMPA_vect)
		{
			cli();
			if (!(PIND & 0x40))
			{
				countlow++;
			}
			else
			{
				counthigh++;
			}
			sei();
		}

		ISR(TIMER1_OVF_vect)
		{
			resultlow=countlow;
			resulthigh=counthigh;
			countlow=0;
			counthigh=0;
		}
		
		if(current>8.5) //make sure current does not exceed 8.5A
		{
			OCR1A=OCR1A-5; //if current exceeds, reduce pulse width
			
			//should print that current threshold has been exceeded
		}
		if(RPMflag==1)
		{
			
			if(RPMcurrent<RPMtarget)
			{
				OCR1A++;
			}
			else if(RPMcurrent>RPMtarget)
			{
				OCR1A--;
			}
			
			//print thing here
			RPMflag=0;
		}
		*/