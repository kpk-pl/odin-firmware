#include <stm32f4xx.h>
#include <math.h>
#include "motorController.h"
#include "TaskPrintfConsumer.h"

float motorController(float speed, float error, float voltage, MotorControllerParameters_Struct *params) {
	float PWM = 0.0f;
	if (fabsf(speed) < params->threshold) {
		//PWM = speed/(params->A_t * voltage + params->B_t) + params->KP_t * error;
		params->PID.Kp = params->KP;
		params->PID.Ki = params->KI;
		params->PID.Kd = params->KD;
		arm_pid_init_f32(&(params->PID), 0);
		PWM = speed/(params->A_t * voltage + params->B_t) + arm_pid_f32(&(params->PID), error);
	} else {
		params->PID.Kp = params->KP_t;
		params->PID.Ki = params->KI_t;
		params->PID.Kd = params->KD_t;
		arm_pid_init_f32(&(params->PID), 0);
		PWM = speed/(params->A * voltage + params->B) - params->C + arm_pid_f32(&(params->PID), error);
		//PWM = speed/(params->A * voltage + params->B) - params->C + params->KP * error;
	}

	if (PWM > 1.0f) PWM = 1.0f;
	else if (PWM < -1.0f) PWM = -1.0f;

	return PWM;
}
