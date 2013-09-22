#ifndef _TASKTRAJECTORY_H_
#define _TASKTRAJECTORY_H_

#include "FreeRTOS.h"
#include "task.h"

extern xTaskHandle trajectoryTask;		/*!< Export this task's handle */

/**
 *  \brief Typedef for gains used by trajectory controller
 */
typedef struct {
	float k_x;
	float k;
	float k_s;
} TrajectoryControlerGains_Struct;

extern TrajectoryControlerGains_Struct globalTrajectoryControlGains;  /*!< Export trajectory controller settings */

/**
 * \brief Handles downloading and processing trajectory following
 */
void TaskTrajectory(void *);			// Task for controlling trajectory using Ferdek's regulator

/**
 * \brief Sets all OS-related objects for use. It should be called on startup, only once, before scheduler starts
 */
void TaskTrajectoryConstructor();

#endif /* _TASKTRAJECTORY_H_ */
