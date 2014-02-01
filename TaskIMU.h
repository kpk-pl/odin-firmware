#ifndef _TASK_IMU_H_
#define _TASK_IMU_H_

#include <stdbool.h>
#include "arm_math.h"

#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"

#include "compilation.h"

#define MAG_IMPROV_DATA_POINTS (72)
#if 360 % MAG_IMPROV_DATA_POINTS != 0 || MAG_IMPROV_DATA_POINTS % 2 != 0
#error "Number of scaling points for magnetometer is not right, use a number that divides 360 and is even."
#endif

extern xTaskHandle imuTask;										/*!< Handle to IMU task exported for other units */
extern volatile float globalMagnetometerImprovData[];			/*!< Handle to magnetometer interpolation data */
extern volatile float globalMagStartOrientation;				/*!< Handle to magnetometer orientation when in start position */

#ifdef USE_GYRO_FOR_IMU
extern volatile float globalGyroDrift;							/*!< Handle to gyro drift */
#endif

/**
 * \brief Handled reading from I2C sensors, data synchronization, scaling and filtering.
 *
 * It sends data to telemetry queue for further processing
 */
void TaskIMU(void *);

/**
 * \brief Sets all OS-related objects for use. It should be called on startup, only once, before scheduler starts
 */
void TaskIMUConstructor();

#endif /* _TASK_IMU_H_ */
