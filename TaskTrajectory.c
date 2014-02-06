#include "compilation.h"
#ifdef FOLLOW_TRAJECTORY

#include "TaskTrajectory.h"
#include "main.h"
#include "priorities.h"
#include "stackSpace.h"
#include "pointsBuffer.h"

#include "TaskMotorCtrl.h"
#include "TaskPrintfConsumer.h"
#include "TaskTelemetry.h"

/**
 * \brief Function for calculating motors speed using trajectory control
 * @param currentPosition Current position from telemetry
 * @param trajectoryPoint Target position and velocities
 * @param outputSpeeds Calculated speeds
 * @return none
 */
static void calculateTrajectoryControll(const TelemetryData_Struct * currentPosition,
								 TrajectoryPoint_Struct* trajectoryPoint,
								 MotorSpeed_Struct * outputSpeeds);

xTaskHandle trajectoryTask;		/*!< This task's handle */
xQueueHandle trajectoryRequestQueue; /*!< Queue to which requests to follow trajectory are sent */
xSemaphoreHandle trajectoryStopSemaphore; /*!< Semaphore used to issue immediate stop */

TrajectoryControlerGains_Struct globalTrajectoryControlGains = {	/*!< Controller data */
	.k_x = 10.0f,
	.k = 7000.0f,
	.k_s = 25.0f
};

void TaskTrajectory(void *p) {
	TelemetryData_Struct telemetry;
	MotorSpeed_Struct motorSpeed;
	portTickType wakeTime = xTaskGetTickCount();
	TrajectoryPoint_Struct nextPoint;
	bool taken = false;

	if (!globalUsingCLI) {
		bool send1 = true, send2 = false;

		while(1) {
			/* Wait for next sampling period */
			vTaskDelayUntil(&wakeTime, 10/portTICK_RATE_MS);

			if (TBgetUsedSpace() < 0.45f) {
				if (!send1) {
					safePrint(35, "<#Please send %d more points#>\n", TBgetSize()/2);
					send1 = true;
				}
			}
			else {send1 = false;}

			if (TBgetNextPoint(&nextPoint)) {
				if (!taken) {
					xSemaphoreTake(motorControllerMutex, portMAX_DELAY);
					taken = true;
				}
				getTelemetry(&telemetry, TelemetryStyle_Common);
				calculateTrajectoryControll(&telemetry, &nextPoint, &motorSpeed);
				sendSpeeds(motorSpeed.LeftSpeed, motorSpeed.RightSpeed);
				send2 = false;
			}
			else {
				if (taken) {
					xSemaphoreGive(motorControllerMutex);
					taken = false;
				}
				if (!send2) {
					safePrint(25, "Trajectory buffer empty\n");
					sendSpeeds(.0f, .0f);
					send2 = true;
				}
			}
		}
	}
	else {
// TODO: Needs finish and testing
// TODO: Implement immediate stop mechanism
		TrajectoryRequest_Struct request;
		FIL *file = NULL;
		while(1) {
			/* Wait for any request */
			xQueueReceive(trajectoryRequestQueue, &request, portMAX_DELAY);

			/* Handle file streaming */
			if (request.source == TrajectorySource_File) {
				/* Try to open the file */
				file = pvPortMalloc(sizeof(FIL));
				FRESULT res = f_open(file, request.fileName, FA_READ | FA_OPEN_EXISTING);
				if (res != FR_OK) {
					vPortFree(file);
					file = NULL;
					safePrint(90, "[Trajectory] Cannot open file %s for reading\n", request.fileName);
				}
				else {
					safePrint(90, "[Trajectory] Reading trajectory from file %s\n", request.fileName);
				}

				/* Free allocated space for name buffer */
				vPortFree(request.fileName);

				/* If file cannot be opened, continue from the top */
				if (res != FR_OK)
					continue;
			}

			/* Reset wake time */
			wakeTime = xTaskGetTickCount();

			/* Execute request */
			while(1) {
				/* Get next point */
				bool ok;
				if (request.source == TrajectorySource_Streaming) {
					ok = TBgetNextPoint(&nextPoint);
				}
				else if (request.source == TrajectorySource_File) {
					UINT numBytes;
					FRESULT res = f_read(file, (void*)&nextPoint, sizeof(TrajectoryPoint_Struct), &numBytes);
					ok = (res == FR_OK && numBytes == sizeof(TrajectoryPoint_Struct));
				}
				else
					break;

				/* Wait for next sampling period; here because f_read is non-deterministic */
				vTaskDelayUntil(&wakeTime, 10/portTICK_RATE_MS);

				if (ok) {
					if (!taken) {
						xSemaphoreTake(motorControllerMutex, portMAX_DELAY);
						taken = true;
					}
					getTelemetry(&telemetry, TelemetryStyle_Common);
					calculateTrajectoryControll(&telemetry, &nextPoint, &motorSpeed);
					sendSpeeds(motorSpeed.LeftSpeed, motorSpeed.RightSpeed);
				}
				else {
					if (taken) {
						sendSpeeds(0.0f, 0.0f);
						xSemaphoreGive(motorControllerMutex);
						taken = false;
					}
					break;
				}
			}

			if (request.source == TrajectorySource_File) {
				f_close(file);
				vPortFree(file);
				file = NULL;
			}
		}
	}
}

void TaskTrajectoryConstructor() {
	xTaskCreate(TaskTrajectory,	NULL, TASKTRAJECTORY_STACKSPACE, NULL, PRIORITY_TASK_TRAJECTORY, &trajectoryTask);
	trajectoryRequestQueue = xQueueCreate(10, sizeof(TrajectoryRequest_Struct));
	trajectoryStopSemaphore = xSemaphoreCreateBinary();
}

void calculateTrajectoryControll(const TelemetryData_Struct * currentPosition,
								 TrajectoryPoint_Struct* trajectoryPoint,
								 MotorSpeed_Struct * outputSpeeds)
{
	float s, c;//, s_r, c_r;
	float e_x, e_y, e_s, e_c, e_c_term;
	float v_b, w_b;
	float v, w;

	float diff_x  = trajectoryPoint->X - currentPosition->X * 1e-3f;
	float diff_y  = trajectoryPoint->Y - currentPosition->Y * 1e-3f;
	float diff_fi = trajectoryPoint->O - currentPosition->O; // unused?

	s = sinf(currentPosition->O);
	c = cosf(currentPosition->O);
	//s_r = sinf(trajectoryPoint->O);
	//c_r = cosf(trajectoryPoint->O);

	//calculate error model
	e_x =  c * diff_x + s * diff_y;
	e_y = -s * diff_x + c * diff_y;
	//e_s = s_r * c - c_r * s;
	//e_c = c_r * c + s_r * s - 1.0f;
	e_s = sinf(diff_fi);
	e_c = cosf(diff_fi) - 1.0f;

	//calculate feedback signal for n=2 and a=7 (see whitepaper)
	e_c_term = 1.0f + e_c / 7.0f; //a = 7;
	e_c_term = powf(e_c_term, 2.0f);

	v_b = globalTrajectoryControlGains.k_x * e_x;
	w_b = globalTrajectoryControlGains.k * trajectoryPoint->V * e_y * e_c_term +
		  globalTrajectoryControlGains.k_s * e_s * powf(e_c_term, 2.0f); //n = 2

	//[m/s]
	v = trajectoryPoint->V * (e_c + 1.0f) + v_b;
	w = trajectoryPoint->W + w_b;

	// [rad/s]
	//outputSpeeds->LeftSpeed  = (v - w * ROBOT_DIAM / (1000.0f * 2.0f)) / (WHEEL_DIAM / 2000.0f);
	//outputSpeeds->RightSpeed = (v + w * ROBOT_DIAM / (1000.0f * 2.0f)) / (WHEEL_DIAM / 2000.0f);
	outputSpeeds->LeftSpeed = v * 1000.0f / RAD_TO_MM_TRAVELED - w;
	outputSpeeds->RightSpeed = v * 1000.0f / RAD_TO_MM_TRAVELED + w;
}

#endif /* FOLLOW_TRAJECTORY */
