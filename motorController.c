#include <stm32f4xx.h>
#include <math.h>
#include "motorController.h"
#include "TaskPrintfConsumer.h"

float motorController(float speed, float error, float voltage, MotorControllerState_Struct *state) {
	float PWM = 0.0f;
	if (fabsf(speed) < state->threshold) {
		//PWM = speed/(params->A_t * voltage + params->B_t) + params->KP_t * error;
		PWM = speed/(state->A_t * voltage + state->B_t) + pid2_eval(&(state->pid2), 0, error);
	} else {
		//PWM = speed/(params->A * voltage + params->B) - params->C + params->KP * error;
		PWM = speed/(state->A * voltage + state->B) - state->C + pid2_eval(&(state->pid2), 1, error);
	}

	if (PWM > 1.0f) PWM = 1.0f;
	else if (PWM < -1.0f) PWM = -1.0f;

	return PWM;
}

void pid2_init(PID2_Instance_Struct* instance) {
	instance->state[0] = 0.0f;
	instance->state[1] = 0.0f;
	instance->state[2] = 0.0f;
}

float pid2_eval(PID2_Instance_Struct* S, uint8_t option, float in) {
	float out;
	PID_Params* p = (option == 0 ? &(S->p1) : &(S->p2));
	/* y[n] = y[n-1] + A0 * x[n] + A1 * x[n-1] + A2 * x[n-2]  */
	/* A0 = Kp + Ki + Kd */
	/* A1 = -Kp - 2*Kd */
	/* A2 = Kd */
	out = (p->Kp + p->Ki + p->Kd)*in + (-p->Kp - 2.0f*p->Kd)*S->state[0] + (p->Kd)*S->state[1] + S->state[2];

    S->state[1] = S->state[0];
    S->state[0] = in;
    S->state[2] = out;

    return out;
}
