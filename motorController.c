#include <stm32f4xx.h>
#include <math.h>
#include "motorController.h"

float motorController(float speed, float error, float voltage, MotorControllerParameters_Struct *params) {
	float PWM = 0.0f;
	if (fabsf(speed) < params->threshold) {
		PWM = speed/(params->A_t * voltage + params->B_t) + params->KP_t * error;
	} else {
		PWM = speed/(params->A * voltage + params->B) - params->C + params->KP * error;
	}

	if (PWM > 1.0f) PWM = 1.0f;
	else if (PWM < -1.0f) PWM = -1.0f;

	return PWM;
}
