#include <stm32f4xx.h>

#include "TaskTelemetry.h"
#include "main.h"
#include "compilation.h"
#include "priorities.h"
#include "stackSpace.h"

#include "TaskPrintfConsumer.h"

xQueueHandle telemetryQueue;		/*!< Queue to which telemetry updates are sent to */
xTaskHandle telemetryTask;			/*!< This task's handle */

void TaskTelemetry(void * p) {
	TelemetryUpdate_Struct update;
#ifdef USE_IMU_TELEMETRY
	portTickType startTime = xTaskGetTickCount();
	bool useIMU = false;
#endif

	while(1) {
#ifdef USE_IMU_TELEMETRY
		if (!useIMU) { // use IMU data after timeout to let magnetometer scaling kick in
			if ((xTaskGetTickCount() - startTime)/portTICK_RATE_MS > 10000) useIMU = true;
		}
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
				globalTelemetryData.O += update.dO;
				if (globalLogTelemetry && (fabsf(update.dX) > 0.1f || fabsf(update.dY) > 0.1f || fabsf(update.dO) > 0.001f)) {
					safePrint(52, "Odometry update: X:%.2f Y:%.2f O:%.1f\n", globalTelemetryData.X, globalTelemetryData.Y, globalTelemetryData.O / DEGREES_TO_RAD);
				}
			}
			taskEXIT_CRITICAL();
			break;
#ifdef USE_IMU_TELEMETRY
		case TelemetryUpdate_Source_IMU:
			if (useIMU) {
			}
			break;
#endif
		default:
			if (globalLogEvents) safePrint(37, "Invalid telemetry update type: %d\n", update.Source);
			break;
		}
	}
}

void TaskTelemetryConstructor() {
	xTaskCreate(TaskTelemetry, NULL, TASKTELEMETRY_STACKSPACE, NULL,	PRIORITY_TASK_TELEMETRY, &telemetryTask);
	telemetryQueue = xQueueCreate(30, sizeof(TelemetryUpdate_Struct));
}

float normalizeOrientation(float in) {
	return (in > M_PI ? in - 2.0f*M_PI : (in <= -M_PI ? in + 2.0f*M_PI : in));
}

void getTelemetry(TelemetryData_Struct *data) {
	/* Ensure that data is coherent and nothing is changed in between */
	taskENTER_CRITICAL();
	{
		data->X = globalTelemetryData.X;
		data->Y = globalTelemetryData.Y;
		data->O = normalizeOrientation(globalTelemetryData.O);
	}
	taskEXIT_CRITICAL();
}

void getTelemetryRaw(TelemetryData_Struct *data) {
	/* Ensure that data is coherent and nothing is changed in between */
	taskENTER_CRITICAL();
	{
		data->X = globalTelemetryData.X;
		data->Y = globalTelemetryData.Y;
		data->O = globalTelemetryData.O;
	}
	taskEXIT_CRITICAL();
}
