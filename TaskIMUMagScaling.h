#ifndef _TASKIMUMAGSCALING_H_
#define _TASKIMUMAGSCALING_H_

#include <stm32f4xx.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

extern xSemaphoreHandle imuMagScalingReq;
extern xQueueHandle magnetometerScalingQueue;
extern xTaskHandle imuMagScalingTask;
extern volatile FunctionalState globalMagnetometerScalingInProgress;

/**
 * Task responsible for scaling magnetometer when the robot is initially turned on.
 *
 * It allows the user the user to request scaling in the few seconds after starting. Scaling procedure
 * makes the robot turn while program collects scaling data. Please take into consideration that during
 * scaling user should not issue any driving commands, as this would corrupt scaling and future
 * magnetometers readings. After scaling or if scaling was not requested this tasks deletes itself
 * and frees allocated objects from stack.
 */
void TaskIMUMagScaling(void *);			// Task for magnetometer scaling. This is not an infinite task.

/**
 * \brief Sets all OS-related objects for use. It should be called on startup, only once, before scheduler starts
 */
void TaskIMUMagScalingConstructor();

#endif /* _TASKIMUMAGSCALING_H_ */
