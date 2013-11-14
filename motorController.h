#ifndef _MOTORCONTROLLER_H_
#define _MOTORCONTROLLER_H_

#include "arm_math.h"

typedef struct {
	float Kp;
	float Ki;
	float Kd;
} PID_Params;

typedef struct {
	PID_Params p1, p2;
	float state[3];
} PID2_Instance_Struct;

typedef struct {
	float threshold;
	float A;
	float B;
	float C;
	float A_t;
	float B_t;
	PID2_Instance_Struct pid2;
} MotorControllerState_Struct;

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
float motorController(float speed, float error, float voltage, MotorControllerState_Struct *state);

void pid2_init(PID2_Instance_Struct* instance);
float pid2_eval(PID2_Instance_Struct* S, uint8_t option, float in);

#endif /* _MOTORCONTROLLER_H_ */
