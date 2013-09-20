#ifndef _TASKTRAJECTORY_H_
#define _TASKTRAJECTORY_H_

#include "compilation.h"
#ifdef FOLLOW_TRAJECTORY

/* Typedef for gains used by trajectory controller */
typedef struct {
	float k_x;
	float k;
	float k_s;
} TrajectoryControlerGains_Struct;

void TaskTrajectory(void *);			// Task for controlling trajectory using Ferdek's regulator

#endif /* FOLLOW_TRAJECTORY */
#endif /* _TASKTRAJECTORY_H_ */
