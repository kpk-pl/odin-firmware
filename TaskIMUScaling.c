#include "priorities.h"
#include "stackSpace.h"
#include "main.h"
#include "compilation.h"

#include "TaskIMUScaling.h"
#include "TaskIMU.h"
#include "TaskMotorCtrl.h"
#include "TaskPrintfConsumer.h"
#include "TaskTelemetry.h"

#ifndef DRIVE_COMMANDS
#error "Drive commands MUST be enabled for TaskIMUScaling to work"
#endif
#include "TaskDrive.h"

#define MAGNETOMETER_MEASUREMENTS_PER_POINT 25

/**
 * \brief Destroys all task resources and ends the task
 */
static void TaskIMUScalingDestructor();

static void movingCenterAlignedAvarage(volatile float *data, uint32_t points, uint8_t order);

xQueueHandle imuScalingQueue = NULL;			/*!< Queue to which magnetometer data should be send during magnetometer scaling in TaskIMUMagScaling */
xTaskHandle imuScalingTask = NULL;				/*!< Handle to this task */
volatile bool globalDoneIMUScaling = false;		/*!< Flag to indicate that scaling finished and IMU is ready to send data to telemetry */

volatile bool globalScaleMagnetometerRequest = false;

void TaskIMUScaling(void *p) {
	IMUAngles_Type IMUAngles;

#ifdef USE_GYRO_FOR_IMU
	float gyroDrift = 0.0f;
	float prevGyro = 0.0f;
#else
	float prevMag = NAN;
#endif
	float magStartO = 0.0f;
	uint32_t samplesCount = 0;

	/* Loop until robot moves OR magnetometer request present */
	while (!movedSinceReset() && !globalScaleMagnetometerRequest) {
		portBASE_TYPE ok = xQueueReceive(imuScalingQueue, &IMUAngles, 40/portTICK_RATE_MS); // try to receive measurements from IMU task
		if (ok == pdTRUE) {
#ifdef USE_GYRO_FOR_IMU
			if (fabsf(IMUAngles.AngleGyro - prevGyro) > 0.1f)								// robot was moved during data acquisition
				break;
			prevGyro = IMUAngles.AngleGyro;
			gyroDrift += IMUAngles.AngleGyro;
#else
			if (prevMag == NAN)
				prevMag = IMUAngles.AngleMag;
			else if (fabsf(IMUAngles.AngleMag - prevMag) > 0.17)
				break;
#endif
			magStartO += IMUAngles.AngleMag;
			samplesCount++;
		}
	}

	/* Update globals */
#ifdef USE_GYRO_FOR_IMU
	globalGyroDrift = prevGyro / samplesCount;
#endif
	globalMagStartOrientation = magStartO / samplesCount;

	if (globalLogEvents) {
#ifdef USE_GYRO_FOR_IMU
		safePrint(57, "[IMUScale] Gyro drift updated: %.5g, %d samples\n", globalGyroDrift, samplesCount);
#endif
		safePrint(63, "[IMUScale] Magnetometer start orientation updated: %.5g\n", globalMagStartOrientation);
	}

	/* Check if there is a request and if robot did not moved */
	if (!globalScaleMagnetometerRequest || movedSinceReset())
		TaskIMUScalingDestructor();

	/* try to acquire lock on motors, should be free as robot does not move */
	portBASE_TYPE ok = xSemaphoreTake(motorControllerMutex, 0);
	if (ok == pdFALSE) {
		if (globalLogEvents)
			safePrint(42, "[IMUScale] Could not acquire motor mutex\n");
		TaskIMUScalingDestructor();
	}
	if (globalLogEvents)
		safePrint(52, "[IMUScale] Starting magnetometer scaling procedure\n");

	/* Prepare drive command */
	DriveCommand_Struct turn_command = {
		.Type = DriveCommand_Type_Angle,
		.UsePen = false,
		.Speed = 0.08f,
		.Param1 = DRIVECOMMAND_ANGLE_PARAM1_ABSOLUTE,
		.Smooth = true
	};
	DriveCommand_Struct *dc;

	/* turning around, save all reading data in orientation intervals */
	for (uint16_t i = 0; i < MAG_IMPROV_DATA_POINTS; ++i) {
		turn_command.Param2 = ((float)i * (360.0f / (float)MAG_IMPROV_DATA_POINTS));
		dc = (DriveCommand_Struct*)pvPortMalloc(sizeof(DriveCommand_Struct));
		*dc = turn_command;
		xQueueSendToBack(driveQueue, &dc, portMAX_DELAY);

		// TODO: Should be cleaner than this
		xSemaphoreGive(motorControllerMutex); 					// now, task drive should acquire this mutex
		vTaskDelay(100/portTICK_RATE_MS);						// wait for the other task to start processing command
		xSemaphoreTake(motorControllerMutex, portMAX_DELAY); 	// recover mutex - now driving should end

		float sum = 0.0f;
		for (uint8_t count = 0; count < MAGNETOMETER_MEASUREMENTS_PER_POINT; count++) {
			xQueueReceive(imuScalingQueue, &IMUAngles, portMAX_DELAY);
			if (i >= MAG_IMPROV_DATA_POINTS/2) IMUAngles.AngleMag -= TWOM_PI;
			sum += IMUAngles.AngleMag;
		}

		globalMagnetometerImprovData[(i + MAG_IMPROV_DATA_POINTS/2) % MAG_IMPROV_DATA_POINTS] = sum / MAGNETOMETER_MEASUREMENTS_PER_POINT;
	}

	/* Go to start orientation */
	turn_command.Param2 = 0.0f;
	dc = (DriveCommand_Struct*)pvPortMalloc(sizeof(DriveCommand_Struct));
	*dc = turn_command;
	xQueueSendToBack(driveQueue, &dc, portMAX_DELAY);

	/* Release motor mutex forever */
	xSemaphoreGive(motorControllerMutex);

	/* Adjust last point in series */
	globalMagnetometerImprovData[MAG_IMPROV_DATA_POINTS] = globalMagnetometerImprovData[0] + TWOM_PI;

	/* Smoothen data series */
	movingCenterAlignedAvarage(globalMagnetometerImprovData, MAG_IMPROV_DATA_POINTS+1, 7);

	/*
	 * check if collected & filtered data is valid
	 * characteristic should be growing in time and should round zero point in Cartesian
	 */
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
	if (globalLogEvents) {
		if (i != MAG_IMPROV_DATA_POINTS || cartesian != 0x0F)
			safePrint(45, "[IMUScale] Magnetometer scaling went wrong!\n");
		else {
			safePrint(38, "[IMUScale] Magnetometer scaling done\n");
		}
	}

	/* End this task */
	TaskIMUScalingDestructor();
}

void TaskIMUScalingConstructor() {
	imuScalingQueue = xQueueCreate(1, sizeof(IMUAngles_Type));
	xTaskCreate(TaskIMUScaling, NULL, TASKIMUSCALING_STACKSPACE, NULL, PRIORITY_TASK_IMUSCALING, &imuScalingTask);
}

void TaskIMUScalingDestructor() {
	globalDoneIMUScaling = true;

	if (imuScalingTask != NULL) {
		xTaskHandle thisTask = imuScalingTask;
		imuScalingTask = NULL;						// this will let other tasks know that task ended
		vTaskDelay(1000/portTICK_RATE_MS);			// safe delay
		vQueueDelete(imuScalingQueue);				// delete queue
		imuScalingQueue = NULL;
		vTaskDelete(thisTask);						// really finish this task
	}
}

void movingCenterAlignedAvarage(volatile float *data, uint32_t points, uint8_t order) {
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
