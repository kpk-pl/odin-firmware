#include <stm32f4xx.h>

#include "TaskMotorCtrl.h"
#include "main.h"
#include "priorities.h"
#include "stackSpace.h"
#include "compilation.h"
#include "hwinterface.h"

#include "TaskPrintfConsumer.h"
#include "TaskTelemetry.h"

/**
 * TODO: Operate on local copies of regulator enabled and pid params, reset pid instance if params changed, reset
 * pid instance if regulator was turned on
 */

xTaskHandle motorCtrlTask;				/*!< This task handle */
xQueueHandle motorCtrlQueue;			/*!< Queue with speeds for motor regulator. It should contain type (MotorSpeed_Struct) */
xSemaphoreHandle motorControllerMutex;	/*!< Mutex to allow many consecutive speeds to be ordered from trajectory regulator */

volatile uint32_t globalLogSpeedCounter = 0;				/*!< Counter to allow only a few logs from speed to be printed */
volatile FunctionalState globalSpeedRegulatorOn = ENABLE;	/*!< On/Off setting for regulator */
volatile MotorSpeed_Struct globalCurrentMotorSpeed;			/*!< Copy of current motors speeds in rad/s */

static MotorSpeed_Struct motorSpeed = {0.0f, 0.0f}; 		/*!< Last ordered speed in rads / sec */

volatile MotorControllerState_Struct globalLeftMotorParams = {		/*!< Left motors custom regulator parameters */
	.forward = {
		.K = 0.08177f,
		.B = 0.06878f
	},
	.backward = {
		.K = 0.07598f,
		.B = 0.05143f
	},
	.pid2 = {
		.forward = {
			.Kp = 0.09f,
			.Ki = 0.001f,
			.Kd = 0.0f
		},
		.backward = {
			.Kp = 0.09f,
			.Ki = 0.001f,
			.Kd = 0.0f
		}
	}
};
volatile MotorControllerState_Struct globalRightMotorParams = {		/*!< Right motors custom regulator parameters */
	.forward = {
		.K = 0.07629f,
		.B = 0.05824f
	},
	.backward = {
		.K = 0.07882f,
		.B = 0.05308f
	},
	.pid2 = {
		.forward = {
			.Kp = 0.09f,
			.Ki = 0.001f,
			.Kd = 0.0f
		},
		.backward = {
			.Kp = 0.09f,
			.Ki = 0.001f,
			.Kd = 0.0f
		}
	}
};

void TaskMotorCtrl(void * p) {
	portTickType wakeTime = xTaskGetTickCount();

	//const float maxSpeedAllowed = 6300.0f * IMPS_TO_RAD; // 6300 is max ticks per second on both motors
	const float maxSpeedAllowed = 10.0f;  // 10 rad/s - It looks like this is the limit for motor controllers' parameters to hold
	uint16_t delayMsPerPeriod = 10;

	float errorLeft, errorRight;
	float outLeft, outRight;
	MotorSpeed_Struct currSpeed, prevSpeed = {0.0f, 0.0f};
	int32_t prevPosLeft = getEncoderL(), prevPosRight = getEncoderR();
	int32_t posLeft, posRight;
	FunctionalState regulatorOn = globalSpeedRegulatorOn;

	TelemetryUpdate_Struct telemetryUpdate = {.Source = TelemetryUpdate_Source_Odometry};
	TelemetryData_Struct telemetryData;

	/*
	 * Custom controller
	 * input - speed [rad/s]
	 * output - PWM
	 */
	pid2_init(&globalLeftMotorParams.pid2);
	pid2_init(&globalRightMotorParams.pid2);

	enableMotors(ENABLE);

	while(1) {
		/* Wait for next sampling period */
		vTaskDelayUntil(&wakeTime, delayMsPerPeriod/portTICK_RATE_MS);

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
			safeLog(log_Type_SpeedOrdered, 21, "L: %.2f R: %.2f\n", motorSpeed.LeftSpeed, motorSpeed.RightSpeed);
		}

		/* Handle turning regulator on or off */
		if (regulatorOn != globalSpeedRegulatorOn) {
			regulatorOn = globalSpeedRegulatorOn;
			pid2_init(&globalLeftMotorParams.pid2);
			pid2_init(&globalRightMotorParams.pid2);
			setMotorLSpeed(0.0f);
			setMotorRSpeed(0.0f);
		}

		/* Read encoders and compute difference in readings. Speeds will be calculated after odometry part */
		posLeft = getEncoderL();
		posRight = getEncoderR();
		currSpeed.LeftSpeed = (float)(posLeft - prevPosLeft);
		currSpeed.RightSpeed = (float)(posRight - prevPosRight);

		/* Compute telemetry update */
		getTelemetry(&telemetryData, TelemetryStyle_Raw);
		float deltaS = (currSpeed.RightSpeed + currSpeed.LeftSpeed) * IMPS_TO_MM_TRAVELED / 2.0f;
		telemetryUpdate.dO = (currSpeed.RightSpeed - currSpeed.LeftSpeed) * IMPS_TO_MM_TRAVELED / ROBOT_DIAM;
		telemetryUpdate.dX = deltaS * cosf(telemetryData.O);
		telemetryUpdate.dY = deltaS * sinf(telemetryData.O);
		telemetryUpdate.Timestamp = xTaskGetTickCount();

		if (xQueueSendToBack(telemetryQueue, &telemetryUpdate, 0) == errQUEUE_FULL) {
			safeLog(Log_Type_Error, 25, "Telemetry queue full!\n");
		}

		/* Compute speeds in rad/s */
		currSpeed.LeftSpeed *= IMPS_TO_RAD*1000.0f/(float)(delayMsPerPeriod/portTICK_RATE_MS);
		currSpeed.RightSpeed *= IMPS_TO_RAD*1000.0f/(float)(delayMsPerPeriod/portTICK_RATE_MS);

		taskENTER_CRITICAL();
		{
			globalCurrentMotorSpeed = currSpeed;
		}
		taskEXIT_CRITICAL();

		if (fabsf(currSpeed.LeftSpeed-prevSpeed.LeftSpeed) > 0.01f ||
				fabsf(currSpeed.RightSpeed-prevSpeed.RightSpeed) > 0.01f)
		{
			safeLog(Log_Type_Speed, 21, "L: %.2f R: %.2f\n", currSpeed.LeftSpeed, currSpeed.RightSpeed);
		}

		prevSpeed = currSpeed;

		/* If regulator is on; critical section is to ensure that regulator is not switched after global... is checked */
		taskENTER_CRITICAL();
		{
			if (globalSpeedRegulatorOn) {
				/* Compute error by simple substraction */
				errorLeft = -(currSpeed.LeftSpeed - motorSpeed.LeftSpeed);
				errorRight = -(currSpeed.RightSpeed - motorSpeed.RightSpeed);

				/* Calculate control values */
				outLeft = motorController(motorSpeed.LeftSpeed, errorLeft, &globalLeftMotorParams);
				outRight = motorController(motorSpeed.RightSpeed, errorRight, &globalRightMotorParams);

				/* Set motors speed; minus is necessary to drive in the right direction */
				if (motorSpeed.LeftSpeed == 0.0f && fabsf(errorLeft) < 0.001f) {
					setMotorLBrake();
					pid2_init(&globalLeftMotorParams.pid2);
				}
				else setMotorLSpeed(outLeft);

				//TODO: if motors' speeds are set to 0, change mode to servo-controller
				if (motorSpeed.RightSpeed == 0.0f && fabsf(errorRight) < 0.001f) {
					setMotorRBrake();
					pid2_init(&globalRightMotorParams.pid2);
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
	}
}

void TaskMotorCtrlConstructor() {
	motorCtrlQueue = xQueueCreate(1, sizeof(MotorSpeed_Struct));
	xTaskCreate(TaskMotorCtrl, NULL, 300, NULL, PRIORITY_TASK_MOTORCTRL, &motorCtrlTask);
	motorControllerMutex = xSemaphoreCreateMutex();
}

void sendSpeeds(float left, float right) {
	MotorSpeed_Struct motorsSpeed = {
		.LeftSpeed = left,
		.RightSpeed = right
	};
	xQueueOverwrite(motorCtrlQueue, &motorsSpeed);
}

bool isCurrentlyDriving() {
	return (motorSpeed.LeftSpeed != 0.0f) || (motorSpeed.RightSpeed != 0.0f);
}
