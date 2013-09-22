#ifndef _MOTORCONTROLLER_H_
#define _MOTORCONTROLLER_H_

#include "arm_math.h"

typedef struct {
	float threshold;
	float A;
	float B;
	float C;
	float KP;
	float KI;
	float KD;
	float A_t;
	float B_t;
	float KP_t;
	float KI_t;
	float KD_t;
	arm_pid_instance_f32 PID;
} MotorControllerParameters_Struct;


// returns PWM for motor
// overall controller structure:
// 			  speed
// PWM = -----------------    - C  +  KP * error
// 		  A * voltage + B
//
// when 'abs(speed)' is < than 'threshold' we are using:
//
// 			  speed
// PWM = -----------------    +  KP_t * error
// 		  A_t * voltage + B_t
// effect of this is when speed == 0, the PWM is also equal to 0
// call Ferdek for more info about these parameters
// all calculations were made by normalizing voltage to 8.0V, so that's why this is default when volt. corr. is OFF
float motorController(float speed, float error, float voltage, MotorControllerParameters_Struct *params);

#endif /* _MOTORCONTROLLER_H_ */
