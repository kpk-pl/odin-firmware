#include <stm32f4xx.h>

#include "TaskTelemetry.h"
#include "main.h"
#include "compilation.h"
#include "priorities.h"
#include "stackSpace.h"
#include "complementary.h"
#include "hwinterface.h"

#include "TaskPrintfConsumer.h"
#include "TaskMotorCtrl.h"
#include "TaskDrive.h"

float globalIMUComplementaryFilterTimeConstant = 0.97f;	/*!< Used to initialize complementary filter */
float globalOdometryCorrectionGain = 1.0049f;	/*!< Gain that is used to correct odometry data (turning angle) */
float globalPositionScale = 1.0f;				/*!< Scale for position; if set to 2 then robot will drive 80cm when told to drive 40cm */
bool globalUseIMUUpdates = true;
xQueueHandle telemetryQueue;					/*!< Queue to which telemetry updates are sent to */
xTaskHandle telemetryTask;						/*!< This task's handle */

static bool globalMovedSinceReset = false;
static volatile portTickType globalVSYNCTimestamp = 0;

/**
 * \brief Global variable that holds current up-to-date telemetry data.
 *
 * Only telemetryTask should write to it.
 * To read from this one should use getTelemetry function than provides mutual exclusion to ensure data coherence
 */
volatile TelemetryData_Struct globalTelemetryData = {0.0f, 0.0f, 0.0f};

void TaskTelemetry(void * p) {
	TelemetryUpdate_Struct update;
	TelemetryData_Struct odometryData = {0.0f, 0.0f, 0.0f};
	TelemetryData_Struct tempData;
#ifdef USE_IMU_TELEMETRY
	Complementary_State filter;
#endif

	while(1) {
#ifdef USE_IMU_TELEMETRY
		ComplementaryInit(&filter, globalIMUComplementaryFilterTimeConstant);
#endif

		/* Wait indefinitely while there is no update */
		xQueueReceive(telemetryQueue, &update, portMAX_DELAY);

		/* Handle multiple updates */
		switch(update.Source) {
		case TelemetryUpdate_Source_Odometry:
			odometryData.X += update.dX;
			odometryData.Y += update.dY;
			odometryData.O += update.dO * globalOdometryCorrectionGain;

			taskENTER_CRITICAL();
			{
#ifdef USE_IMU_TELEMETRY
				if (!globalUseIMUUpdates) {
					globalTelemetryData = odometryData;
				}
				else {
#endif
					globalTelemetryData.X += update.dX;
					globalTelemetryData.Y += update.dY;
					globalTelemetryData.O += update.dO * globalOdometryCorrectionGain;
#ifdef USE_IMU_TELEMETRY
				}
#endif
				tempData = globalTelemetryData;
			}
			taskEXIT_CRITICAL();
			break;
		case TelemetryUpdate_Source_IMU:
#ifdef USE_IMU_TELEMETRY
			if (globalUseIMUUpdates) {
				taskENTER_CRITICAL();
				{
					globalTelemetryData.O = ComplementaryGet(&filter, globalTelemetryData.O, update.dO);
					tempData = globalTelemetryData;
				}
				taskEXIT_CRITICAL();
			}
#endif
			break;
		default:
			safeLog(Log_Type_Error, 37, "Invalid telemetry update type: %d\n", update.Source);
			break;
		}

		if (fabsf(update.dX) > 0.1f || fabsf(update.dY) > 0.1f || fabsf(update.dO) > 0.001f) {
			safeLog(Log_Type_Telemetry, 39, "X: %.2f Y: %.2f O: %.1f\n", tempData.X, tempData.Y, tempData.O / DEGREES_TO_RAD);
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

#ifdef USE_IMU_TELEMETRY
	bool imuupd = globalUseIMUUpdates;
	globalUseIMUUpdates = false;
#endif
	OnOff penen = getPenState();
	enablePen(ENABLE);

	DriveCommand_Struct *cmd1 = pvPortMalloc(sizeof(DriveCommand_Struct));
	DriveCommand_Struct *cmd2 = pvPortMalloc(sizeof(DriveCommand_Struct));
	DriveCommand_Struct *cmd3 = pvPortMalloc(sizeof(DriveCommand_Struct));

	*cmd3 = *cmd1 = (DriveCommand_Struct){
		.Type = DriveCommand_Type_Line,
		.UsePen = true,
		.Speed = 0.07f,
		.Param1 = 200.0f,
		.Smooth = true
	};
	cmd3->Param1 = 100;
	*cmd2 = (DriveCommand_Struct){
		.Type = DriveCommand_Type_Angle,
		.UsePen = true,
		.Speed = 0.07f,
		.Param1 = DRIVECOMMAND_ANGLE_PARAM1_RELATIVE,
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

	if (penen == OFF)
		enablePen(DISABLE);
#ifdef USE_IMU_TELEMETRY
	globalUseIMUUpdates = imuupd;
#endif

	safeLog(Log_Type_Log, 85, "Scaling done!\nOdometry correction param will be: (%d + dO)/%.6g\n",
			360*turns-180, (360.0f*turns-180.0f)/globalOdometryCorrectionGain);
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

/* Called from ISR */
void radioCameraVSYNCHandler() {
	globalVSYNCTimestamp = xTaskGetTickCountFromISR();
}
