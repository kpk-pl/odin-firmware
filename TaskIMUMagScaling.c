#include "TaskIMUMagScaling.h"
#include "priorities.h"
#include "stackSpace.h"
#include "main.h"
#include "TaskIMU.h"
#include "TaskMotorCtrl.h"
#include "TaskPrintfConsumer.h"
#include "TaskTelemetry.h"

#include "TaskDrive.h" // needed at all cost

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
	getTelemetry(&telemetry, TelemetryStyle_Raw);
	if (fabsf(telemetry.X) > 0.1f || fabsf(telemetry.Y) > 0.1f || fabsf(telemetry.O) > 0.01f) goto finish;
	// ok, robot is not moving, try to do scaling

#ifdef FOLLOW_TRAJECTORY
	vTaskSuspend(trajectoryTask);
#endif

	magnetometerScalingQueue = xQueueCreate(10,	sizeof(float));
	globalMagnetometerScalingInProgress = ENABLE;

	float imuAngle;
	portTickType startTick, endTick;

	taken = xQueueReceive(magnetometerScalingQueue, &imuAngle, 5000/portTICK_RATE_MS);
	if (taken != pdFALSE) { // something really came in, doing scaling
		safePrint(23, "Scaling magnetometer\n");

		//sendSpeeds(-0.6f, 0.6f, portMAX_DELAY);
		startTick = xTaskGetTickCount();

		DriveCommand_Struct turn_command = {
			.Type = DriveCommand_Type_Angle,
			.UsePen = false,
			.Speed = 0.3f,
			.Param1 = 1.0f,	// absolute angle
			.Smooth = true
		};
		DriveCommand_Struct *dc;

		// turning around, save all reading data in orientation intervals
		uint16_t i = 0;
		while(i < MAG_IMPROV_DATA_POINTS) {
			// turn to desired orientation (absolute angles)
			turn_command.Param2 = (i * (360 / MAG_IMPROV_DATA_POINTS));
			dc = (DriveCommand_Struct*)pvPortMalloc(sizeof(DriveCommand_Struct));
			*dc = turn_command;
			xQueueSendToBack(driveQueue, &dc, portMAX_DELAY);
			vTaskDelay(100/portTICK_RATE_MS);	// wait for the other task to start processing command
			xSemaphoreTake(motorControllerMutex, portMAX_DELAY);	// if the semaphore is taken, it was free, as other task gave it - driving is done after this point
			xSemaphoreGive(motorControllerMutex);

			xQueueReset(magnetometerScalingQueue);  // clear queue content

			float sum = 0.0f;
			for (uint8_t count = 0; count < 25; count++) {
				xQueueReceive(magnetometerScalingQueue, &imuAngle, portMAX_DELAY);
				if (i >= MAG_IMPROV_DATA_POINTS/2) imuAngle -= TWOM_PI;
				sum += imuAngle;
			}

			globalMagnetometerImprovData[(i + MAG_IMPROV_DATA_POINTS/2) % MAG_IMPROV_DATA_POINTS] = sum / 25.0f;

			i++;
		}

		turn_command.Param2 = 360.0f;
		dc = (DriveCommand_Struct*)pvPortMalloc(sizeof(DriveCommand_Struct));
		*dc = turn_command;
		xQueueSendToBack(driveQueue, &dc, portMAX_DELAY);

		globalMagnetometerImprovData[MAG_IMPROV_DATA_POINTS] = globalMagnetometerImprovData[0] + TWOM_PI;

		endTick = xTaskGetTickCount();
	}

	movingCenterAlignedAvarage(globalMagnetometerImprovData, globalMagnetometerImprov.nValues, 5);

	// check if collected & filtered data is valid
	// characteristic should be growing in time and should round zero point in Cartesian
	uint8_t cartesian = 0;
	uint16_t i;
	for (i = 0; i<MAG_IMPROV_DATA_POINTS; ++i) {
		if (globalMagnetometerImprovData[i] > globalMagnetometerImprovData[i+1]) break;
		float norm = normalizeOrientation(globalMagnetometerImprovData[i]);
		if (norm < 0.0f) {
			if (norm < -HALFM_PI) cartesian |= 0x08;
			else cartesian |= 0x04;
		}
		else {
			if (norm < HALFM_PI) cartesian |= 0x02;
			else cartesian |= 0x01;
		}
	}
	if (i != MAG_IMPROV_DATA_POINTS || cartesian != 0x0F)
		safePrint(34, "Magnetometer scaling went wrong!\n");

#ifdef FOLLOW_TRAJECTORY
	vTaskResume(trajectoryTask);
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
