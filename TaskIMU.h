#ifndef _TASK_IMU_H_
#define _TASK_IMU_H_

#include "compilation.h"

#ifdef USE_IMU_TELEMETRY

#include "FreeRTOS.h"
#include "timers.h"

void TaskIMU(void *);

/* Handler for timer overflow when I2C hangs up */
void imuWatchdogOverrun(xTimerHandle xTimer);

#endif /* USE_IMU_TELEMETRY */

#endif /* _TASK_IMU_H_ */
