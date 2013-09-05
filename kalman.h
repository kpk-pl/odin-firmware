#ifndef _KALMAN_H_
#define _KALMAN_H_

typedef struct {
	float Q_angle;
	float Q_bias;
	float R_measure;
	float bias;
	float P[2][2];
	float angle;
} Kalman_State;

void KalmanInit(Kalman_State * kalman_state);
float KalmanGet(Kalman_State * kalman_state, float newAngle, float newRate, float dt);


#endif /* _KALMAN_H_ */
