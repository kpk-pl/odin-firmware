#include <stdbool.h>
#include <math.h>

#include "stackSpace.h"
#include "priorities.h"
#include "hwinterface.h"
#include "hardware.h"

#include "TaskPenCtrl.h"
#include "TaskTelemetry.h"

xTaskHandle penCtrlTask;
xQueueHandle penCommandQueue;

volatile bool globalPenIsDown = false;
volatile PenLine_Type globalCurrentPen = PenLine_Continuous;

void TaskPenCtrl(void *p) {
	portTickType wakeTime = xTaskGetTickCount();
	TelemetryData_Struct telemetry, telemetryOld;
	portTickType tick, tickOld;
	PenLine_Type copyPen;

	float timings[4][4] = {
			{1.0f,	20.0f,	1.0f,	20.0f},
			{20.0f,	20.0f,	20.0f,	20.0f},
			{50.0f,	50.0f,	50.0f,	50.0f},
			{1.0f,	25.0f,	30.0f,	25.0f}
	};

	float distance;
	bool received, reset = false;
	uint8_t stateMachine;

	setPenUp();

	while (1) {
		// wait for the next sampling period
		vTaskDelayUntil(&wakeTime, 10/portTICK_RATE_MS);

		// receive request to set pen up or down
		if (xQueueReceive(penCommandQueue, &received, 0) == pdTRUE) {
			if (received != globalPenIsDown) {
				if (received == false) {
					setPenUp();
				}
				else {
					reset = true; // force reset of the state machine
				}
				globalPenIsDown = received;
			}
		}

		// do not do anything if pen is up
		if (!globalPenIsDown) continue;

		// reset state machine if line type changed
		if (copyPen != globalCurrentPen || reset) {
			setPenDown();
			stateMachine = 0;
			getTelemetry(&telemetryOld, TelemetryStyle_Raw);
			tickOld = xTaskGetTickCount();
			copyPen = globalCurrentPen;
			reset = false;
		}

		getTelemetry(&telemetry, TelemetryStyle_Raw);
		distance += hypotf(telemetryOld.X - telemetry.X, telemetryOld.Y - telemetry.Y);
		telemetryOld = telemetry;

		// do not allow too frequent changes
		tick = xTaskGetTickCount();
		if ((tick - tickOld) / portTICK_RATE_MS < 100) continue;

		// state machine for various line types
		switch (copyPen) {
		case PenLine_Continuous:
			// do nothing, always down
			break;
		case PenLine_Dotted /* 0 */:
		case PenLine_DashedShort /* 1 */:
		case PenLine_DashedLong /* 2 */:
		case PenLine_DotDash /* 3 */:
			if (distance > timings[copyPen][stateMachine]) {
				if (stateMachine % 2)
					setPenDown();
				else
					setPenUp();

				stateMachine = (stateMachine+1)%4;
				tickOld = tick;
				distance = 0.0f;
			}
			break;
		default:
			break;
		}
	}
}

void TaskPenCtrlConstructor() {
	xTaskCreate(TaskPenCtrl, NULL, TASKPENCTRL_STACKSPACE, NULL, PRIORITY_TASK_PENCTRL, &penCtrlTask);
	penCommandQueue = xQueueCreate(1, sizeof(bool));
}

void setPenLineType(PenLine_Type type) {
	globalCurrentPen = type;
}
