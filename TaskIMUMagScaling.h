#ifndef _TASKIMUMAGSCALING_H_
#define _TASKIMUMAGSCALING_H_

#include "compilation.h"
#ifdef USE_IMU_TELEMETRY

void TaskIMUMagScaling(void *);			// Task for magnetometer scaling. This is not an infinite task.

#endif /* USE_IMU_TELEMETRY */
#endif /* _TASKIMUMAGSCALING_H_ */
