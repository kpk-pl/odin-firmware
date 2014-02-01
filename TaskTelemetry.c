#include <stm32f4xx.h>

#include "TaskTelemetry.h"
#include "main.h"
#include "compilation.h"
#include "priorities.h"
#include "stackSpace.h"
#include "complementary.h"

#include "TaskPrintfConsumer.h"
#include "TaskMotorCtrl.h"
#include "TaskDrive.h"

float globalOdometryCorrectionGain = 1.0075f;	/*!< Gain that is used to correct odometry data (turning angle) */
float globalPositionScale = 1.0f;				/*!< Scale for position; if set to 2 then robot will drive 80cm when told to drive 40cm */
bool globalUseIMUUpdates = true;
xQueueHandle telemetryQueue;					/*!< Queue to which telemetry updates are sent to */
xTaskHandle telemetryTask;						/*!< This task's handle */

static bool globalMovedSinceReset = false;

/**
 * \brief Global variable that holds current up-to-date telemetry data.
 *
 * Only telemetryTask should write to it.
 * To read from this one should use getTelemetry function than provides mutual exclusion to ensure data coherence
 */
volatile TelemetryData_Struct globalTelemetryData = {0.0f, 0.0f, 0.0f};

void TaskTelemetry(void * p) {
	TelemetryUpdate_Struct update;
#ifdef USE_IMU_TELEMETRY
	portTickType startTime = xTaskGetTickCount();
	Complementary_State filter;
	bool useIMU = false;
#endif

	while(1) {
#ifdef USE_IMU_TELEMETRY
		if (!useIMU) { // use IMU data after timeout to let magnetometer scaling kick in
			if ((xTaskGetTickCount() - startTime)/portTICK_RATE_MS > 10000) useIMU = true;
		}
		ComplementaryInit(&filter, 0.999f);	// 40s time constant with 25Hz sampling
#endif

		/* Wait indefinitely while there is no update */
		xQueueReceive(telemetryQueue, &update, portMAX_DELAY);

		/* Handle multiple updates */
		switch(update.Source) {
		case TelemetryUpdate_Source_Odometry:
			taskENTER_CRITICAL();
			{
				globalTelemetryData.X += update.dX;
				globalTelemetryData.Y += update.dY;
				globalTelemetryData.O += update.dO * globalOdometryCorrectionGain;
				if (globalLogTelemetry && (fabsf(update.dX) > 0.1f || fabsf(update.dY) > 0.1f || fabsf(update.dO) > 0.001f)) {
					safePrint(52, "Odometry update: X:%.2f Y:%.2f O:%.1f\n", globalTelemetryData.X, globalTelemetryData.Y, globalTelemetryData.O / DEGREES_TO_RAD);
				}
			}
			taskEXIT_CRITICAL();
			break;
		case TelemetryUpdate_Source_IMU:
#ifdef USE_IMU_TELEMETRY
			if (!useIMU || !globalUseIMUUpdates) break;
			taskENTER_CRITICAL();
			{
				globalTelemetryData.O = ComplementaryGet(&filter, globalTelemetryData.O, update.dO);
				if (globalLogTelemetry)
					safePrint(27, "IMU update: O:%.1f\n", globalTelemetryData.O / DEGREES_TO_RAD);
			}
			taskEXIT_CRITICAL();
#endif
			break;
		default:
			if (globalLogEvents)
				safePrint(37, "Invalid telemetry update type: %d\n", update.Source);
			break;
		}

		if (!globalMovedSinceReset) {
			if (globalTelemetryData.X != 0.0f || globalTelemetryData.Y != 0.0f || globalTelemetryData.O != 0.0f) {
				globalMovedSinceReset = true;
			}
		}
	}
}

void TaskTelemetryConstructor() {
	xTaskCreate(TaskTelemetry, NULL, TASKTELEMETRY_STACKSPACE, NULL,	PRIORITY_TASK_TELEMETRY, &telemetryTask);
	telemetryQueue = xQueueCreate(30, sizeof(TelemetryUpdate_Struct));
}

bool movedSinceReset() {
	return globalMovedSinceReset;
}

float normalizeOrientation(float in) {
/*	while (in > M_PI) in -= TWOM_PI;
	while (in <= -M_PI) in += TWOM_PI;
	return in;
*/
	return in - floorf((in + M_PI)/TWOM_PI) * TWOM_PI;
}

void scaleOdometryCorrectionParam(int turns) {
	if (isCurrentlyDriving()) return;

	DriveCommand_Struct *cmd1 = pvPortMalloc(sizeof(DriveCommand_Struct));
	DriveCommand_Struct *cmd2 = pvPortMalloc(sizeof(DriveCommand_Struct));
	DriveCommand_Struct *cmd3 = pvPortMalloc(sizeof(DriveCommand_Struct));

	*cmd3 = *cmd1 = (DriveCommand_Struct){
		.Type = DriveCommand_Type_Line,
		.UsePen = true,
		.Speed = 0.07f,
		.Param1 = 100.0f,
		.Smooth = true
	};
	*cmd2 = (DriveCommand_Struct){
		.Type = DriveCommand_Type_Angle,
		.UsePen = true,
		.Speed = 0.07f,
		.Param1 = 0.0f,
		.Param2 = 360.0f * turns - 180.0f,
		.Smooth = true
	};

	xQueueSend(driveQueue, &cmd1, portMAX_DELAY);
	xQueueSend(driveQueue, &cmd2, portMAX_DELAY);
	xQueueSend(driveQueue, &cmd3, portMAX_DELAY);

	vTaskDelay(1000/portTICK_RATE_MS);
	while (isCurrentlyDriving()) {
		vTaskDelay(10/portTICK_RATE_MS);
	}

	safePrint(85, "Scaling done!\nOdometry correction param will be: %.6g/(%d + dO)\n",
		(360.0f*turns-180.0f)*globalOdometryCorrectionGain, 360*turns-180);
}

void getTelemetry(TelemetryData_Struct *data, TelemetryStyle_Type style) {
	taskENTER_CRITICAL();
	{
		data->X = globalTelemetryData.X;
		data->Y = globalTelemetryData.Y;
		data->O = globalTelemetryData.O;
	}
	taskEXIT_CRITICAL();

	if (style & TelemetryStyle_Scaled) {
		data->X /= globalPositionScale;
		data->Y /= globalPositionScale;
	}
	if (style & TelemetryStyle_Normalized) {
		data->O = normalizeOrientation(data->O);
	}
}
