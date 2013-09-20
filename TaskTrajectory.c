#include "compilation.h"
#ifdef FOLLOW_TRAJECTORY

#include "TaskTrajectory.h"
#include "main.h"
#include "pointsBuffer.h"

/*
 * @brief Function for calculating motors speed using trajectory control
 * @param currentPosition Current position from telemetry
 * @param trajectoryPoint Target position and velocities
 * @param outputSpeeds Calculated speeds
 * @retval none
 */
static void calculateTrajectoryControll(const TelemetryData_Struct * currentPosition,
								 TrajectoryPoint_Ptr trajectoryPoint,
								 MotorSpeed_Struct * outputSpeeds);

void TaskTrajectory(void *p) {
	TelemetryData_Struct telemetry;
	MotorSpeed_Struct motorSpeed;
	portTickType wakeTime = xTaskGetTickCount();
	uint8_t requestNumber = 2;

	while(1) {
		/* Wait for next sampling period */
		vTaskDelayUntil(&wakeTime, 10/portTICK_RATE_MS);

		float usedSpace = TBgetUsedSpace();
		if (usedSpace < 0.1f && requestNumber == 2) { safePrint(35, "<#Please send %d more points#>\n", 7*TBgetSize()/8); requestNumber = 3;}
		else if (usedSpace < 0.2f && requestNumber == 1) { safePrint(35, "<#Please send %d more points#>\n", 3*TBgetSize()/4); requestNumber = 2;}
		else if (usedSpace < 0.45f && requestNumber == 0) { safePrint(35, "<#Please send %d more points#>\n", TBgetSize()/2); requestNumber = 1;}
		else if (usedSpace >= 0.45f) requestNumber = 0;

		TrajectoryPoint_Ptr point = TBgetNextPoint();
		if (point != NULL) {
			getTelemetry(&telemetry);
			calculateTrajectoryControll(&telemetry, point, &motorSpeed);
		}
		else {
			motorSpeed.LeftSpeed = motorSpeed.RightSpeed = 0.0f;
		}

		//xQueueSendToBack(motorCtrlQueue, &motorSpeed, portMAX_DELAY); // order motors to drive with different speed, wait for them to accept
	}
}

void calculateTrajectoryControll(const TelemetryData_Struct * currentPosition,
								 TrajectoryPoint_Ptr trajectoryPoint,
								 MotorSpeed_Struct * outputSpeeds)
{
	float s, c, s_r, c_r;
	float e_x, e_y, e_s, e_c, e_c_term;
	float v_b, w_b;
	float v, w;

	float diff_x  = trajectoryPoint->X * 1e4f - currentPosition->X; // trajectoryPoint is in metres but here mm are needed
	float diff_y  = trajectoryPoint->Y * 1e4f - currentPosition->Y;
	//float diff_fi = trajectoryPoint->O - currentPosition->O; // unused?

	// buehuehuehue :D anyway, is float equivalent to float32_t???
	arm_sin_cos_f32(currentPosition->O, &s, &c);
	arm_sin_cos_f32(trajectoryPoint->O, &s_r, &c_r);

	//calculate error model
	e_x =  c * diff_x + s * diff_y;
	e_y = -s * diff_x + c * diff_y;
	e_s = s_r * c - c_r * s;
	e_c = c_r * c + s_r * s - 1.0f;

	//calculate feedback signal for n=2 and a=7 (see whitepaper)

	e_c_term = 1.0f + e_c / 7.0f; //a = 7;
	e_c_term = powf(e_c_term, 2.0f);

	v_b = globalTrajectoryControlGains.k_x * e_x;
	w_b = globalTrajectoryControlGains.k * trajectoryPoint->V * e_y * e_c_term + globalTrajectoryControlGains.k_s * e_s * powf(e_c_term, 2.0f); //n = 2

	//[m/s]
	v = trajectoryPoint->V * (e_c + 1.0f) + v_b;
	w = trajectoryPoint->W + w_b;

	// [rad/s]
	outputSpeeds->LeftSpeed  = (v - w * ROBOT_DIAM / (1000.0f * 2.0f)) / WHEEL_DIAM;
	outputSpeeds->RightSpeed = (v + w * ROBOT_DIAM / (1000.0f * 2.0f)) / WHEEL_DIAM;
}

/* ISR for WiFi DMA Rx */
void RawStreamDMAIncoming(void) {
	TBDMATransferCompletedSlot();
}

#endif /* FOLLOW_TRAJECTORY */
