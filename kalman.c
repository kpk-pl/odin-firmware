#include "kalman.h"

void KalmanInit(Kalman_State * kalman_state) {
	kalman_state->Q_angle = 0.001f;
	kalman_state->Q_bias = 0.003f;
	kalman_state->R_measure = 0.03f;
	kalman_state->bias = 0.0f;
	kalman_state->P[0][0] = kalman_state->P[0][1] = kalman_state->P[1][0] = kalman_state->P[1][1] = 0.0f;
	kalman_state->angle = 0.0f;
}

float KalmanGet(Kalman_State * kalman_state, float newAngle, float newRate, float dt) {
	float rate = newRate - kalman_state->bias;
	kalman_state->angle += dt * rate;

	kalman_state->P[0][0] += dt * (dt * kalman_state->P[1][1] - kalman_state->P[0][1] -
			kalman_state->P[1][0] + kalman_state->Q_angle);
	kalman_state->P[0][1] -= dt * kalman_state->P[1][1];
	kalman_state->P[1][0] -= dt * kalman_state->P[1][1];
	kalman_state->P[1][1] += kalman_state->Q_bias * dt;

	float S = kalman_state->P[0][0] + kalman_state->R_measure;

	float K[2];
	K[0] = kalman_state->P[0][0] / S;
	K[1] = kalman_state->P[1][0] / S;

	float y = newAngle - kalman_state->angle;

	kalman_state->angle += K[0] * y;
	kalman_state->bias += K[1] * y;

	kalman_state->P[0][1] -= K[0] * kalman_state->P[0][0];
	kalman_state->P[0][1] -= K[0] * kalman_state->P[0][1];
	kalman_state->P[1][0] -= K[1] * kalman_state->P[0][0];
	kalman_state->P[1][1] -= K[1] * kalman_state->P[0][1];

	return kalman_state->angle;
}
