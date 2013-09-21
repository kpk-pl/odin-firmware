#ifndef _TASK_IMU_H_
#define _TASK_IMU_H_

#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"

extern xTaskHandle imuTask;		/*!< Handle to IMU task exported for other units */

/**
 * \brief Handled reading from I2C sensors, data synchronization, scaling and filtering.
 *
 * It sends data to telemetry queue for further processing
 */
void TaskIMU(void *);

/**
 *  \brief Handler for timer overflow when I2C hangs up
 *  @param xTimer Handle to timer which overflowed
 */
void imuWatchdogOverrun(xTimerHandle xTimer);

/**
 * \brief Sets all OS-related objects for use. It should be called on startup, only once, before scheduler starts
 */
void TaskIMUConstructor();

#endif /* _TASK_IMU_H_ */
