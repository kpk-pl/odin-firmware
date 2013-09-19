#include <stm32f4xx.h>

#include "TaskMotorCtrl.h"
#include "main.h"
#include "compilation.h"
#include "hwinterface.h"

void TaskMotorCtrl(void * p) {
	portTickType wakeTime = xTaskGetTickCount();
	/* Speed is given in radians per second */
	MotorSpeed_Struct motorSpeed = {0.0f, 0.0f};

	//const float maxSpeedAllowed = 6300.0f * IMPS_TO_RAD; // 6300 is max ticks per second on both motors
	const float maxSpeedAllowed = 10.0f;  // 10 rad/s - It looks like this is the limit for motor controllers' parameters to hold
	const uint16_t delayMsPerPeriod = 10;

	float errorLeft, errorRight;
	float outLeft, outRight;
	float speedLeft, speedRight;
	int32_t prevPosLeft = getEncoderL(), prevPosRight = getEncoderR();
	int32_t posLeft, posRight;

	TelemetryUpdate_Struct telemetryUpdate = {.Source = TelemetryUpdate_Source_Odometry};
	TelemetryData_Struct telemetryData;

#ifdef USE_CUSTOM_MOTOR_CONTROLLER
	/*
	 * Custom controller
	 * input - speed [rad/s]
	 * output - PWM
	 */
#else
	/*
	 * Input to PID controller is error of rotational velocity in rad/sec
	 * Output is normalized speed scaled linearly to PWM
	 */
	arm_pid_init_f32(&globalPidLeft, 1);
	arm_pid_init_f32(&globalPidRight, 1);
#endif

	enableMotors(ENABLE);

	while(1) {
		/*
		 * Receive from the queue, if it is empty then nothing is saved nowhere
		 * If smth was really received it should be checked.
		 * Limit max absolute value preserving original sign
		 */
		if (xQueueReceive(motorCtrlQueue, &motorSpeed, 0) == pdTRUE) {
			float max = fmaxf(fabsf(motorSpeed.LeftSpeed), fabsf(motorSpeed.RightSpeed));
			if (max > maxSpeedAllowed) {
				motorSpeed.LeftSpeed  *= maxSpeedAllowed / max;
				motorSpeed.RightSpeed *= maxSpeedAllowed / max;
			}
			if (globalLogSpeed) safePrint(34, "Ordered speeds: L:%.2f R:%.2f\n", motorSpeed.LeftSpeed, motorSpeed.RightSpeed);
		}

		/* Read encoders and compute difference in readings. Speeds will be calculated after odometry part */
		posLeft = getEncoderL();
		posRight = getEncoderR();
		speedLeft = (float)(posLeft - prevPosLeft);
		speedRight = (float)(posRight - prevPosRight);

		/* Compute telemetry update */
		getTelemetry(&telemetryData);
		float deltaS = (speedRight + speedLeft) * IMPS_TO_MM_TRAVELED / 2.0f;
		telemetryUpdate.dO = (speedRight - speedLeft) * IMPS_TO_MM_TRAVELED / ROBOT_DIAM;
		telemetryUpdate.dX = deltaS * cosf(telemetryData.O);
		telemetryUpdate.dY = deltaS * sinf(telemetryData.O);
		/*if (speedLeft != speedRight) {
			float a = ROBOT_DIAM / 2.0f * (speedRight + speedLeft) / (speedRight - speedLeft);
			telemetryUpdate.dO = (speedRight - speedLeft) * IMPS_TO_MM_TRAVELED / ROBOT_DIAM;
			telemetryUpdate.dX = a*(sinf(telemetryUpdate.dO + telemetryData.O) - sinf(telemetryData.O));
			telemetryUpdate.dY = -a*(cosf(telemetryUpdate.dO + telemetryData.O) - cosf(telemetryData.O));
		}
		else {
			float deltaS = (speedRight + speedLeft) * IMPS_TO_MM_TRAVELED / 2.0f;
			telemetryUpdate.dX = deltaS * cosf(telemetryData.O);
			telemetryUpdate.dY = deltaS * sinf(telemetryData.O);
			telemetryUpdate.dO = 0.0f;
		}*/
		if (xQueueSendToBack(telemetryQueue, &telemetryUpdate, 0) == errQUEUE_FULL) {
			if (globalLogEvents) safePrint(25, "Telemetry queue full!\n");
		}

		/* Compute speeds in rad/s */
		speedLeft *= IMPS_TO_RAD*1000.0f/(float)(delayMsPerPeriod/portTICK_RATE_MS);
		speedRight *= IMPS_TO_RAD*1000.0f/(float)(delayMsPerPeriod/portTICK_RATE_MS);

		if (globalLogSpeed && globalLogSpeedCounter > 0) { // mod sth to allow changing log period
			safePrint(42, "Speeds: L:%.4frad/s R:%.4frad/s\n", speedLeft, speedRight);
			globalLogSpeedCounter--;
		}

		/* If regulator is on; critical section is to ensure that regulator is not switched after global... is checked */
		taskENTER_CRITICAL();
		{
			if (globalSpeedRegulatorOn) {
				/* Compute error by simple substraction */
				errorLeft = -(speedLeft - motorSpeed.LeftSpeed);
				errorRight = -(speedRight - motorSpeed.RightSpeed);

#ifdef USE_CUSTOM_MOTOR_CONTROLLER
				/* Use Ferdek's controllers */
				float voltage = 8.0f; //calculations were made for voltage normalized to 8V
				if(globalControllerVoltageCorrection) voltage = getAvgBatteryVoltage();
				outLeft = motorController(motorSpeed.LeftSpeed, errorLeft, voltage, &globalLeftMotorParams);
				outRight = motorController(motorSpeed.RightSpeed, errorRight, voltage, &globalRightMotorParams);
#else
				/* Invoke PID functions and compute output speed values, minus is necessary for PID */
				outLeft = arm_pid_f32(&globalPidLeft, errorLeft);
				outRight = arm_pid_f32(&globalPidRight, errorRight);
#endif

				/* Set motors speed; minus is necessary to drive in the right direction */
				if (motorSpeed.LeftSpeed == 0.0f && fabsf(errorLeft) < 0.001f) {
					setMotorLBrake();
#ifndef USE_CUSTOM_MOTOR_CONTROLLER
					arm_pid_reset_f32(&globalPidLeft);
#endif
				}
				else setMotorLSpeed(outLeft);

				//TODO: if motors' speeds are set to 0, change mode to servo-controller
				if (motorSpeed.RightSpeed == 0.0f && fabsf(errorRight) < 0.001f) {
					setMotorRBrake();
#ifndef USE_CUSTOM_MOTOR_CONTROLLER
					arm_pid_reset_f32(&globalPidRight);
#endif
				}
				else setMotorRSpeed(outRight);
			}
			else {
				// if regulator was turned off, keep target speed at 0 so that motors will break after regulator is on again
				motorSpeed.LeftSpeed = motorSpeed.RightSpeed = 0.0f;
			}
		}
		taskEXIT_CRITICAL();

		/* Update current encoders position for next loop pass */
		prevPosLeft = posLeft;
		prevPosRight = posRight;

		/* Wait for next sampling period */
		vTaskDelayUntil(&wakeTime, delayMsPerPeriod/portTICK_RATE_MS);
	}
}
