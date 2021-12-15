/*
 * PID.c
 *
 * Created: 22/11/2021 10:58:57
 *  Author: Henning
 */ 

#include <stdio.h>
#include "lcd.h"
#include <avr/io.h>
#include <stdlib.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include "i2cmaster.h"
#include "ds1621.h"
#include "usart.h"
#include "PID.h"



float PID_RPM(float RPM_current, float RPM_setpoint, float RPM_time,float Kp, float Ki)
{
	static float RPM_lastinput=0;
	static double I_error=0;
	
	float P_error=RPM_setpoint-RPM_current;
	
	I_error+=P_error*RPM_time;
	
	float D_error=0;//(RPM_current-RPM_lastinput)/RPM_time;
	RPM_lastinput=RPM_current;
	return (PID_KP*P_error+PID_KI*I_error+PID_KD*D_error);
}

float PID_CURRENT(float i_current,float i_desired, float i_time) //i stands for electric current
{
	double i_err=i_desired-i_current;
	static double I2_error=0;
	//static float i_lastinput=0;
	
	I2_error+=i_err*i_time;
	
	//i_lastinput=i_current;
	return (PID_KC*i_err+PID_KI2*I2_error);
	
}
