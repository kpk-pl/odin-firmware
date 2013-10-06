#ifndef _TRAJECTORY_CONTROLLER_H_
#define _TRAJECTORY_CONTROLLER_H_

#include <stm32f4xx.h>

#include "main.h"
#include "pointsBuffer.h"

typedef struct {
	float k_x;
	float k;
	float k_x;
} TrajectoryControllerGains_Struct;

void calculateTrajectoryControll();

#endif /* _TRAJECTORY_CONTROLLER_H_ */
