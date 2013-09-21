#include "TaskIMUMagScaling.h"
#include "priorities.h"
#include "stackSpace.h"
#include "main.h"
#include "TaskIMU.h"

extern xQueueHandle motorCtrlQueue;		/*!< Queue with speeds for motor regulator. It should contain type (MotorSpeed_Struct) */

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
#else
	vTaskSuspend(driveTask);
#endif

	magnetometerScalingQueue = xQueueCreate(10,	sizeof(float));
	globalMagnetometerScalingInProgress = ENABLE;

	float imuAngle;
	MotorSpeed_Struct motorsSpeed;

	taken = xQueueReceive(magnetometerScalingQueue, &imuAngle, 5000/portTICK_RATE_MS);
	if (taken != pdFALSE) { // something really came in, doing scaling
		safePrint(23, "Scaling magnetometer\n");

		motorsSpeed.LeftSpeed = -0.3f;
		motorsSpeed.RightSpeed = 0.3f;
		xQueueSendToBack(motorCtrlQueue, &motorsSpeed, portMAX_DELAY);

		// turning around, save all reading data in orientation intervals
		uint16_t i = 0;
		while(telemetry.O < 2*M_PI) {
			do {
				vTaskDelay(1);
				getTelemetryRaw(&telemetry);
				taken = xQueueReceive(magnetometerScalingQueue, &imuAngle, 0);	// read everything as soon as possible
			} while (telemetry.O < globalMagnetometerImprov.xSpacing * i);

			if (i >= 360) imuAngle -= 2.0f*M_PI;
			globalMagnetometerImprovData[(i+360)%720] = imuAngle;
			i++;
		}

		globalMagnetometerImprovData[720] = globalMagnetometerImprovData[0] + 2.0f*M_PI;

		motorsSpeed.LeftSpeed = motorsSpeed.RightSpeed = 0.0f;
		xQueueSendToBack(motorCtrlQueue, &motorsSpeed, portMAX_DELAY);
	}

#ifdef FOLLOW_TRAJECTORY
	vTaskResume(trajectoryTask);
#else
	vTaskResume(driveTask);
#endif

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
	xTaskCreate(TaskIMUMagScaling, NULL, TASKIMUMAGSCALING_TASKSPACE, NULL,	PRIORITY_TASK_IMUMAGSCALING, &imuMagScalingTask);

}
