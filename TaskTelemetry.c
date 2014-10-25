#include <stm32f4xx.h>

#include "TaskTelemetry.h"
#include "main.h"
#include "memory.h"
#include "compilation.h"
#include "priorities.h"
#include "stackSpace.h"
#include "hwinterface.h"
#include "radioRcvr.h"

#include "TaskPrintfConsumer.h"
#include "TaskMotorCtrl.h"
#include "TaskDrive.h"

typedef struct {
	TelemetryData_Struct Telemetry;
	portTickType Timestamp;
} TimestampedTelemetryData_Struct;

static void telemetrySaveToHistory(const TelemetryData_Struct *data, const portTickType timestamp);
static int32_t telemetrySearchHistoryTimestamp(const portTickType timestamp, TelemetryData_Struct *outResult);

/**
 * Delay between time when camera captures the frame and when this firmware receives VSYNC impulse
 * This delay should be constant and should include delays from:
 * - time between frame grabbing by camera and VSYNC impuls generation in camera
 * - time between VSYNC signal detection by radio transmitter and receiver (with over the air and firmwares propagation delays)
 * - time between radio receiver VSYNC reception and interrupt generation
 * - time between interrupt generation and interrupt reception and handling in this firmware
 */
volatile float globalCameraTransmissionDelayMs = 0.0f;
volatile float globalCameraTelemetryFilterConstant = 0.97f;	/*!< Complementary filter for camera telemetry updates */
volatile float globalOdometryCorrectionGain = 1.0049f;		/*!< Gain that is used to correct odometry data (turning angle) */
volatile float globalPositionScale = 1.0f;					/*!< Scale for position; if set to 2 then robot will drive 80cm when told to drive 40cm */
xQueueHandle telemetryQueue;					/*!< Queue to which telemetry updates are sent to */
xTaskHandle telemetryTask;						/*!< This task's handle */

static bool globalMovedSinceReset = false;
static volatile portTickType globalVSYNCTimestamp = 0; 		/*!< Timestamp of the last camera VSYNC pulse */

#define TELEMETRY_HISTORY_BUFFER_LEN 2000
static CCMEM TimestampedTelemetryData_Struct globalTelemetryHistory[TELEMETRY_HISTORY_BUFFER_LEN];  /*!< Circular buffer for historical telemetry data based on odometry updates */
static uint32_t globalTelemetryHistoryIterator = 0;			/*!< Iterator for telemetry history buffer, points to the next free element */
static bool globalTelemetryHistoryRollover = false;			/*!< True if the whole buffer was filled at least once */

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
	float correctedOrientation;
	int32_t returnCode;
	float currentFilterConstant, currentComplementaryConstant;

	radioSetup();

	while(1) {
		/* Wait indefinitely while there is no update */
		xQueueReceive(telemetryQueue, &update, portMAX_DELAY);

		/* Handle multiple updates */
		switch(update.Source) {
		case TelemetryUpdate_Source_Odometry:
			correctedOrientation = update.dO * globalOdometryCorrectionGain;
			odometryData.X += update.dX;
			odometryData.Y += update.dY;
			odometryData.O += correctedOrientation;

			taskENTER_CRITICAL();
			{
				globalTelemetryData.X += update.dX;
				globalTelemetryData.Y += update.dY;
				globalTelemetryData.O += correctedOrientation;
				tempData = globalTelemetryData;
			}
			taskEXIT_CRITICAL();

			telemetrySaveToHistory(&tempData, update.Timestamp);
			break;
		case TelemetryUpdate_Source_Camera:
			returnCode = telemetrySearchHistoryTimestamp(globalVSYNCTimestamp, &tempData);
			if (returnCode == 0) {
				currentFilterConstant = globalCameraTelemetryFilterConstant;
				currentComplementaryConstant = 1.0f - currentFilterConstant;
				taskENTER_CRITICAL();
				{
					//globalTelemetryData.X += (currentFilterConstant * tempData.X + currentComplementaryConstant * update.dX) - tempData.X;
					//globalTelemetryData.Y += (currentFilterConstant * tempData.Y + currentComplementaryConstant * update.dY) - tempData.Y;
					//globalTelemetryData.O += (currentFilterConstant * tempData.O + currentComplementaryConstant * update.dO) - tempData.O;
					globalTelemetryData.X += currentComplementaryConstant * (update.dX - tempData.X);
					globalTelemetryData.Y += currentComplementaryConstant * (update.dY - tempData.Y);
					globalTelemetryData.O += currentComplementaryConstant * (update.dO - tempData.O);
					tempData = globalTelemetryData;
				}
				taskEXIT_CRITICAL();
			} else {
				safeLog(Log_Type_Error, 46, "Cannot find timestamp in history, retv = %d\n", returnCode);
				continue;
			}
			safeLog(Log_Type_Camera, 120, "Radio: %.3f %.3f %.3f "
					"Odo: %.3f %.3f %.3f "
					"Filt: %.3f %.3f %.3f "
					"Time: %ld\n",
					update.dX, update.dY, update.dO,
					odometryData.X, odometryData.Y, odometryData.O,
					tempData.X, tempData.Y, tempData.O,
					globalVSYNCTimestamp);
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
	return in - floorf((in + M_PI)/TWOM_PI) * TWOM_PI;
}

void scaleOdometryCorrectionParam(int turns) {
	if (isCurrentlyDriving()) return;

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

void radioCameraVSYNCHandlerFromISR() {
	globalVSYNCTimestamp = xTaskGetTickCountFromISR() - (int32_t)(globalCameraTransmissionDelayMs/(float)portTICK_RATE_MS);
}

void telemetrySaveToHistory(const TelemetryData_Struct *data, const portTickType timestamp) {
	globalTelemetryHistory[globalTelemetryHistoryIterator] = (TimestampedTelemetryData_Struct) {
		.Timestamp = timestamp,
		.Telemetry = *data,
	};

	globalTelemetryHistoryIterator = (globalTelemetryHistoryIterator+1) % TELEMETRY_HISTORY_BUFFER_LEN;

	if (globalTelemetryHistoryIterator == 0) {
		globalTelemetryHistoryRollover = true;
	}
}

int32_t telemetrySearchHistoryTimestamp(const portTickType timestamp, TelemetryData_Struct *outResult) {
	/* Cannot assign globalTelemetryHistoryIterator-1 if there was not a rollover */
	if (globalTelemetryHistoryIterator == 0 && !globalTelemetryHistoryRollover) {
		return -1;
	}

	uint32_t iterator = (globalTelemetryHistoryIterator - 1) % TELEMETRY_HISTORY_BUFFER_LEN;
	while (globalTelemetryHistory[iterator].Timestamp > timestamp) {
		/* Searched through all history without success */
		if (iterator == globalTelemetryHistoryIterator) {
			return -2;
		}

		/* Handle rollover */
		if (iterator == 0) {
			if (!globalTelemetryHistoryRollover) {
				return -3;
			}
			iterator = TELEMETRY_HISTORY_BUFFER_LEN-1;
		}
		else {
			iterator--;
		}
	}

	/* The requested timestamp is too new and there are no entries in history */
	if (iterator == (globalTelemetryHistoryIterator - 1) % TELEMETRY_HISTORY_BUFFER_LEN) {
		return -4;
	}

	/* Find next entry in history, if there are no return error */
	uint32_t next_iterator = (iterator + 1) % TELEMETRY_HISTORY_BUFFER_LEN;
	if (next_iterator == globalTelemetryHistoryIterator) {
		return -5;
	}

	/* Calculate where between iterator and next_iterator is requested timestamp */
	float ratio = (float)(timestamp - globalTelemetryHistory[iterator].Timestamp) /
			(float)(globalTelemetryHistory[next_iterator].Timestamp - globalTelemetryHistory[iterator].Timestamp);
	float complementary_ratio = 1.0f - ratio;

	/* Calculate the position in between measurements from history */
	outResult->X = complementary_ratio * globalTelemetryHistory[iterator].Telemetry.X + ratio * globalTelemetryHistory[next_iterator].Telemetry.X;
	outResult->Y = complementary_ratio * globalTelemetryHistory[iterator].Telemetry.Y + ratio * globalTelemetryHistory[next_iterator].Telemetry.Y;
	outResult->O = complementary_ratio * globalTelemetryHistory[iterator].Telemetry.O + ratio * globalTelemetryHistory[next_iterator].Telemetry.O;

	/* Return with success */
	return 0;
}
