#ifndef _MOTORCONTROLLER_H_
#define _MOTORCONTROLLER_H_

#include "arm_math.h"

typedef struct {
	float Kp;
	float Ki;
	float Kd;
} PID_Params;

typedef struct {
	PID_Params forward, backward;
	float state[3];
} PID2_Instance_Struct;

typedef struct {
	float K;
	float B;
} MotorControllerPredictionParams;

typedef struct {
	MotorControllerPredictionParams forward;
	MotorControllerPredictionParams backward;
	PID2_Instance_Struct pid2;
} MotorControllerState_Struct;

// returns PWM for motor
// overall controller structure:
// 
// PWM = K * speed - B  +  PID(error)
// 
//
float motorController(float speed, float error, volatile MotorControllerState_Struct *state);

void pid2_init(volatile PID2_Instance_Struct* instance);
//direction == 0 is FORWARD
float pid2_eval(volatile PID2_Instance_Struct* S, uint8_t direction, float in);

#endif /* _MOTORCONTROLLER_H_ */
