#ifndef _TASKIMUMAGSCALING_H_
#define _TASKIMUMAGSCALING_H_

#include <stm32f4xx.h>

#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

extern xQueueHandle imuScalingQueue;
extern xTaskHandle imuScalingTask;

extern volatile bool globalScaleMagnetometerRequest;
extern volatile bool globalDoneIMUScaling;

typedef struct {
	float AngleGyro;
	float AngleMag;
} IMUAngles_Type;

/**
 * Task responsible for scaling IMU. It receives updates from IMU just after start. Until robot moves it
 * computes the best approximation of Gyro drift. After robot moves this is discarded and Gyro drift is
 * used in computations in IMU task.
 * During the period before the robot moves for the first time, this task can receive a request to
 * scale magnetometer. If such request is received, Gyro scaling stops and robot turns 360 degrees
 * in intervals, measuring local magnetic fields. This scaling may be used later by IMU task to perform
 * data interpolation to improve magnetometer accuracy.
 * After the robot ends magnetometer scaling procedure, or moves for the first time without magnetometer
 * scaling, then this task ends and NULLs all its resources. From this point IMU works and sends updates
 * to telemetry task.
 */
void TaskIMUScaling(void *);			// Task for IMU scaling. This is not an infinite task.

/**
 * \brief Sets all OS-related objects for use. It should be called on startup, only once, before scheduler starts
 */
void TaskIMUScalingConstructor();

#endif /* _TASKIMUMAGSCALING_H_ */
