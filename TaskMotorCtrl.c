#include <stm32f4xx.h>

#include "TaskMotorCtrl.h"
#include "main.h"
#include "priorities.h"
#include "stackSpace.h"
#include "compilation.h"
#include "hwinterface.h"

#include "TaskPrintfConsumer.h"
#include "TaskTelemetry.h"

xTaskHandle motorCtrlTask;				/*!< This task handle */
xQueueHandle motorCtrlQueue;			/*!< Queue with speeds for motor regulator. It should contain type (MotorSpeed_Struct) */
xSemaphoreHandle motorControllerMutex;	/*!< Mutex to allow many consecutive speeds to be ordered from trajectory regulator */

volatile uint32_t globalLogSpeedCounter = 0;				/*!< Counter to allow only a few logs from speed to be printed */
volatile FunctionalState globalSpeedRegulatorOn = ENABLE;	/*!< On/Off setting for regulator */
#ifdef USE_CUSTOM_MOTOR_CONTROLLER
	volatile FunctionalState globalControllerVoltageCorrection = DISABLE;	/*!< Voltage correction for custom regulator */
#endif

#ifdef USE_CUSTOM_MOTOR_CONTROLLER
	MotorControllerParameters_Struct globalLeftMotorParams = {		/*!< Left motors custom regulator parameters */
		.threshold = 1.7164f,
		.A  = 1.2885f,
		.B  = 0.9323f,
		.C  = 0.1427f,
		.KP = 0.05f, // TODO: value TBD experimentally
		.KI = 0.01f,
		.KD = 0.0f,
		.A_t  = 19.6755f,
		.B_t  = 14.2360f,
		.KP_t = 0.05f,//TODO: value TBD experimentally
		.KI_t = 0.01f,
		.KD_t = 0.0f
	};
	MotorControllerParameters_Struct globalRightMotorParams = {		/*!< Right motors custom regulator parameters */
		.threshold = 1.5510f,
		.A  = 1.3758f,
		.B  = 1.4037f,
		.C  = 0.1150f,
		.KP = 0.05f, // TODO: value TBD experimentally
		.KI = 0.01f,
		.KD = 0.0f,
		.A_t  = 17.2120f,
		.B_t  = 17.5611f,
		.KP_t = 0.05f, //TODO: value TBD experimentally
		.KI_t = 0.01f,
		.KD_t = 0.0f
	};
#else
	arm_pid_instance_f32 globalPidLeft = {	/*!< Left motors PID regulator parameters */
		.Kp = 0.08f,
		.Ki = 0.005f,
		.Kd = 0.0f
	};
	arm_pid_instance_f32 globalPidRight = {	/*!< Right motors PID regulator parameters */
		.Kp = 0.08f,
		.Ki = 0.005f,
		.Kd = 0.0f
	};
#endif

void TaskMotorCtrl(void * p) {
	safePrint(19, "Booting completed\n");
	portTickType wakeTime = xTaskGetTickCount();
	/* Speed is given in radians per second */
	MotorSpeed_Struct motorSpeed = {0.0f, 0.0f};

	//const float maxSpeedAllowed = 6300.0f * IMPS_TO_RAD; // 6300 is max ticks per second on both motors
	const float maxSpeedAllowed = 10.0f;  // 10 rad/s - It looks like this is the limit for motor controllers' parameters to hold
	uint16_t delayMsPerPeriod = 10;

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
	}
}

void TaskMotorCtrlConstructor() {
	motorCtrlQueue = xQueueCreate(1, sizeof(MotorSpeed_Struct));
	xTaskCreate(TaskMotorCtrl, NULL, 300, NULL, PRIORITY_TASK_MOTORCTRL, &motorCtrlTask);
	motorControllerMutex = xSemaphoreCreateMutex();
}

void sendSpeeds(float left, float right, unsigned portLONG delay) {
	MotorSpeed_Struct motorsSpeed = {
		.LeftSpeed = left,
		.RightSpeed = right
	};
	xQueueSendToBack(motorCtrlQueue, &motorsSpeed, delay);
}