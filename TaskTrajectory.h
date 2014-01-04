#ifndef _TASKTRAJECTORY_H_
#define _TASKTRAJECTORY_H_

#include "FreeRTOS.h"
#include "task.h"
#include "ff.h"

extern xTaskHandle trajectoryTask;		/*!< Export this task's handle */

/**
 *  \brief Typedef for gains used by trajectory controller
 */
typedef struct {
	float k_x;
	float k;
	float k_s;
} TrajectoryControlerGains_Struct;

/**
 * \brief Typedef for marking source of trajectory points
 */
typedef enum {
	TrajectorySource_Streaming = 0,
	TrajectorySource_File
} TrajectorySource_Type;

/**
 * \brief Request to perform trajectory following
 */
typedef struct {
	TrajectorySource_Type source;
	union {
		uint32_t stream_points;
		FIL *file_ptr;
	};
} TrajectoryRequest_Struct;

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
