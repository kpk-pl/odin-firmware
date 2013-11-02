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
								 TrajectoryPoint_Ptr trajectoryPoint,
								 MotorSpeed_Struct * outputSpeeds);

xTaskHandle trajectoryTask;		/*!< This task's handle */

TrajectoryControlerGains_Struct globalTrajectoryControlGains = {	/*!< Controller data */
	.k_x = 10.0f,
	.k = 7000.0f,
	.k_s = 25.0f
};

void TaskTrajectory(void *p) {
	TelemetryData_Struct telemetry;
	MotorSpeed_Struct motorSpeed;
	portTickType wakeTime = xTaskGetTickCount();
	bool send1 = true, send2 = false;
	bool taken = false;

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

		TrajectoryPoint_Ptr point = TBgetNextPoint();
		if (point != NULL) {
			if (!taken) {
				xSemaphoreTake(motorControllerMutex, portMAX_DELAY);
				taken = true;
			}
			getTelemetryScaled(&telemetry);
			calculateTrajectoryControll(&telemetry, point, &motorSpeed);
			xQueueSendToBack(motorCtrlQueue, &motorSpeed, portMAX_DELAY); // order motors to drive with different speed, wait for them to accept
			send2 = false;
		}
		else {
			if (taken) {
				xSemaphoreGive(motorControllerMutex);
				taken = false;
			}
			if (!send2) {
				safePrint(25, "Trajectory buffer empty\n");
				sendSpeeds(.0f, .0f, portMAX_DELAY);
				send2 = true;
			}
		}

	}
}

void TaskTrajectoryConstructor() {
	xTaskCreate(TaskTrajectory,	NULL, TASKTRAJECTORY_STACKSPACE, NULL, PRIORITY_TASK_TRAJECTORY, &trajectoryTask);
}

void calculateTrajectoryControll(const TelemetryData_Struct * currentPosition,
								 TrajectoryPoint_Ptr trajectoryPoint,
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

/* ISR for WiFi DMA Rx */
void RawStreamDMAIncoming(void) {
	TBDMATransferCompletedSlot();
}

#endif /* FOLLOW_TRAJECTORY */
