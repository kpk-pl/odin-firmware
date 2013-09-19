#ifndef _TASK_IMU_H_
#define _TASK_IMU_H_

#include "FreeRTOS.h"
#include "timers.h"

void TaskIMU(void *);

/* Handler for timer overflow when I2C hangs up */
void imuWatchdogOverrun(xTimerHandle xTimer);

#endif /* _TASK_IMU_H_ */
