#ifndef _TASK_IMU_H_
#define _TASK_IMU_H_

#include <stdbool.h>
#include "arm_math.h"

#include "FreeRTOS.h"
#include "timers.h"
#include "task.h"

#define MAG_IMPROV_DATA_POINTS (72)
#if 360 % MAG_IMPROV_DATA_POINTS != 0 || MAG_IMPROV_DATA_POINTS % 2 != 0
#error "Number of scaling points for magnetometer is not right, use a number that divides 360 and is even."
#endif

extern xTaskHandle imuTask;										/*!< Handle to IMU task exported for other units */
extern arm_linear_interp_instance_f32 globalMagnetometerImprov;	/*!< Handle to magnetometer interpolation structure */
extern float globalMagnetometerImprovData[];					/*!< Handle to magnetometer interpolation data */
extern volatile bool globalDoneIMUScaling;						/*!< Handle to flag indicating wheather scaling was performed or nor */

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
