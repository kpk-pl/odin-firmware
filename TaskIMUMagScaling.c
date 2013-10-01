#include "TaskIMUMagScaling.h"
#include "priorities.h"
#include "stackSpace.h"
#include "main.h"
#include "TaskIMU.h"
#include "TaskMotorCtrl.h"
#include "TaskPrintfConsumer.h"
#include "TaskTelemetry.h"

#ifdef DRIVE_COMMANDS
#include "TaskDrive.h"
#endif
#ifdef FOLLOW_TRAJECTORY
#include "TaskTrajectory.h"
#endif

static void movingCenterAlignedAvarage(float *data, uint32_t points, uint8_t order);

volatile FunctionalState globalMagnetometerScalingInProgress = DISABLE;	/*!< ENABLE if currently doing scaling with robot turning. Set in TaskIMUMagScaling */

xSemaphoreHandle imuMagScalingReq;				/*!< Semaphore to indicate that magnetometer scaling was requested */
xQueueHandle magnetometerScalingQueue = NULL;	/*!< Queue to which magnetometer data should be send during magnetometer scaling in TaskIMUMagScaling */
xTaskHandle imuMagScalingTask;					/*!< Handle to this task */

void TaskIMUMagScaling(void *p) {
	xSemaphoreTake(imuMagScalingReq, 0);	// initial take

	uint8_t taken = xSemaphoreTake(imuMagScalingReq, 5000/portTICK_RATE_MS);	// wait for max 5s for request
	if (taken == pdFALSE) goto finish;
	// ok, there was a request. Check if robot moved

	TelemetryData_Struct telemetry;
	getTelemetryRaw(&telemetry);
	if (fabsf(telemetry.X) > 0.1f || fabsf(telemetry.Y) > 0.1f || fabsf(telemetry.O) > 0.01f) goto finish;
	// ok, robot is not moving, try to do scaling

#ifdef FOLLOW_TRAJECTORY
	vTaskSuspend(trajectoryTask);
#endif
#ifdef DRIVE_COMMANDS
	vTaskSuspend(driveTask);
#endif

	magnetometerScalingQueue = xQueueCreate(10,	sizeof(float));
	globalMagnetometerScalingInProgress = ENABLE;

	float imuAngle;
	portTickType startTick, endTick;

	taken = xQueueReceive(magnetometerScalingQueue, &imuAngle, 5000/portTICK_RATE_MS);
	if (taken != pdFALSE) { // something really came in, doing scaling
		safePrint(23, "Scaling magnetometer\n");

		sendSpeeds(-0.6f, 0.6f, portMAX_DELAY);
		startTick = xTaskGetTickCount();

		// turning around, save all reading data in orientation intervals
		uint16_t i = 0;
		while(i < 2*720) {
			do {
				vTaskDelay(1);
				getTelemetryRaw(&telemetry);
				taken = xQueueReceive(magnetometerScalingQueue, &imuAngle, 0);	// read everything as soon as possible
			} while (telemetry.O < globalMagnetometerImprov.xSpacing * i);

			if ((i+360)%720 < 360) imuAngle -= TWOM_PI;
			if (i >= 720) imuAngle -= TWOM_PI;
			globalMagnetometerImprovData[(i+360)%720] = imuAngle;

			i+=2;
			if (i == 720) i += 1;
		}

		globalMagnetometerImprovData[720] = globalMagnetometerImprovData[0] + 2.0f*M_PI;

		sendSpeeds(0.0f, 0.0f, portMAX_DELAY);
		endTick = xTaskGetTickCount();
	}

	movingCenterAlignedAvarage(globalMagnetometerImprovData, globalMagnetometerImprov.nValues, 35);
	movingCenterAlignedAvarage(globalMagnetometerImprovData, globalMagnetometerImprov.nValues, 23);

#ifdef FOLLOW_TRAJECTORY
	vTaskResume(trajectoryTask);
#endif
#ifdef DRIVE_COMMANDS
	vTaskResume(driveTask);
#endif

	safePrint(55, "Scaling took %d ms, stack high-water mark at %d\n", (endTick-startTick)/portTICK_RATE_MS, uxTaskGetStackHighWaterMark(NULL));

	globalDoneIMUScaling = true;
	globalMagnetometerScalingInProgress = DISABLE;
	vTaskDelay(500/portTICK_RATE_MS);				// delay to make sure other tasks know that global flag is DISABLED
	vQueueDelete(magnetometerScalingQueue);
	magnetometerScalingQueue = NULL;
finish:
	globalMagnetometerScalingInProgress = DISABLE; 	// just to be sure it is disabled (if not done so by default at initialization)
	vTaskDelete(NULL);								// delete this task
}

void TaskIMUMagScalingConstructor() {
	vSemaphoreCreateBinary(imuMagScalingReq);
	xTaskCreate(TaskIMUMagScaling, NULL, TASKIMUMAGSCALING_STACKSPACE, NULL,	PRIORITY_TASK_IMUMAGSCALING, &imuMagScalingTask);
}

void movingCenterAlignedAvarage(float *data, uint32_t points, uint8_t order) {
	if (order % 2 != 1) return;			// only odd orders are available

	const uint8_t half = order/2;
	const uint32_t size = points + order - 1;
	float filtered[size];
	for (uint32_t i = 0; i<size; ++i)
		filtered[i] = 0;

	for (uint32_t i = half; i < size-half; ++i) {
		for (uint32_t j = i-half; j <= i+half; ++j) {
			float32_t addon;
			if (j < half) addon = data[points - half - 1 + j] - 2.0f * M_PI;
			else if (j >= points + half) addon = data[j - points - half + 1] + 2.0f * M_PI;
			else addon = data[j - half];
			filtered[i] += addon;
		}
	}

	for (uint32_t i = 0; i < points; ++i)
		data[i] = filtered[i+half] / (float)order;
}
