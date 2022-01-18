/*
 * PID.h
 *
 * Created: 22/11/2021 10:59:19
 *  Author: Henning
 */ 


#ifndef PID_H_
#define PID_H_

#define PID_KC 0.00073/0.000208 //Kp term for the current ctrl La/Tdesired
#define PID_KI2 0.7/0.000208 //calculate Time desired Ra/Tdesired
#define PID_KP 0.2 //should be Kn
#define PID_KI 0.01 //should be Kn/Tn 
#define PID_KD 0.1
#define I_MAX 8.7
#define I_MIN 0
#define RPM_MAX 3430
#define RPM_MIN 0
#define OCR1A_MAX 70.0
#define OCR1A_MIN 0

float PID_RPM(float RPM_current, float RPM_setpoint, float RPM_time);


#endif /* PID_H_ */