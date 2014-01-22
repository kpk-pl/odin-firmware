#include "TaskDrive.h"
#include "main.h"
#include "hwinterface.h"
#include "priorities.h"
#include "stackSpace.h"
#include "TaskMotorCtrl.h"
#include "TaskTelemetry.h"
#include "TaskPrintfConsumer.h"
#include "TaskPenCtrl.h"

#define TASKDRIVE_BASEDELAY_MS 10		/*!< Base time period for motors regulators */

typedef enum {
	Smoothness_None = 0,
	Smoothness_Beginning = 1,
	Smoothness_End = 2,
	Smoothness_Both = 3
} Smoothness_Type;

/**
 * \brief Drives to target point. First turn in the direction of target point, then drive to it.
 * @param command Drive command pointer
 */
static void drivePoint(const DriveCommand_Struct* command);
/**
 * \brief Turns or drives over circular trajectory. Note that it cannot turn by more than 180 degrees ralative.
 * @param command Drive command pointer
 */
static void driveAngleArc(const DriveCommand_Struct* command);

/**
 * \brief Checks if there is more driving commands to perform
 * @return true if more commands awaits execution
 */
static bool isThereMoreDrivingCommands(void);

static void turnRads(float rads, MotorSpeed_Struct speed, float epsilon, Smoothness_Type smoothness);

xQueueHandle driveQueue;	/*!< Queue with drive commands. It should contain type (DriveCommand_Struct*) */
xTaskHandle driveTask;		/*!< This task handler */
static bool isDriving = false;

void TaskDrive(void * p) {
	DriveCommand_Struct * command;
	bool taken = false;

	while(1) {
		/* Take one command from queue or wait for the command to arrive */
		if (!isThereMoreDrivingCommands()) {		// free resources
			sendSpeeds(0.0f, 0.0f, portMAX_DELAY);			// stop motors as there is no command available
			if (taken) {
				xSemaphoreGive(motorControllerMutex);
				taken = false;
				xQueueSendToBack(penCommandQueue, &taken, portMAX_DELAY);	// set pen up, use 'taken' variable as it is false either way
			}
			isDriving = false;
		}

		xQueueReceive(driveQueue, &command, portMAX_DELAY);
		isDriving = true;

		if (!taken) {	// block resources
			xSemaphoreTake(motorControllerMutex, portMAX_DELAY);
			taken = true;
		}

		/* Check if speed is not less than zero */
		if (command->Speed >= 0.0f) {
			/* Handle pen */
			xQueueSendToBack(penCommandQueue, &command->UsePen, portMAX_DELAY);

			/* Recalculate speed from m/s to rad/s */
			command->Speed = command->Speed * 1000.0f / RAD_TO_MM_TRAVELED;

			if (command->Type == DriveCommand_Type_Line) {
				if (globalLogEvents) safePrint(20, "Driving %.0fmm\n", command->Param1);

				/* Change to driving to point */
				command->Type = DriveCommand_Type_Point;
				TelemetryData_Struct telemetry;
				getTelemetryScaled(&telemetry);
				command->Param2 = telemetry.Y + sinf(telemetry.O) * command->Param1;
				command->Param1 = telemetry.X + cosf(telemetry.O) * command->Param1;
				drivePoint(command);
			}
			else if (command->Type == DriveCommand_Type_Angle || command->Type == DriveCommand_Type_Arc) {
				if (globalLogEvents) {
					if (command->Type == DriveCommand_Type_Angle) {
						safePrint(30, "Turning by %.2f %s\n", command->Param2, (command->Param1 < 0.5f ? "relative" : "absolute"));
					}
					else {
						safePrint(58, "Turning with radius %.2fmm and %.1f degrees length\n", command->Param1, command->Param2);
					}
				}
				driveAngleArc(command);
			}
			else if (command->Type == DriveCommand_Type_Point) {
				if (globalLogEvents) safePrint(44, "Driving to point X:%.1fmm Y:%.1fmm\n", command->Param1, command->Param2);
				drivePoint(command);
			}
		}
		else { /* command->Speed < 0.0f */
			if (globalLogEvents) safePrint(34, "Speed cannot be less than zero!\n");
		}

		/* Free space where the command was held */
		vPortFree(command);
	}
}

void drivePoint(const DriveCommand_Struct* command) {
	if (command->Type != DriveCommand_Type_Point) return;

	portTickType wakeTime = xTaskGetTickCount();
	TelemetryData_Struct telemetryData;

	/* First of all, turn to target point with desired accuracy */
	/* Get starting point telemetry data */
	getTelemetryRawScaled(&telemetryData);

	/* Calculate angle to target */
	float angO = normalizeOrientation(atan2f(command->Param2 - telemetryData.Y, command->Param1 - telemetryData.X) - telemetryData.O);

	if (fabsf(angO) > 10.0 * DEGREES_TO_RAD) {
		float rightSpeed = copysignf(command->Speed, angO);
		turnRads(angO,
				(MotorSpeed_Struct)
					{.LeftSpeed = -rightSpeed,
					 .RightSpeed = rightSpeed},
				1.0f * DEGREES_TO_RAD,
				Smoothness_Beginning);
	}

	/* Start regulator - driving to point requires constant speeds updates */
	while(1) {
		/* Read current position */
		getTelemetryScaled(&telemetryData);

		/* Finish up if target is really close or robot's missing the target */
		float d = hypotf(telemetryData.X - command->Param1, telemetryData.Y - command->Param2);
		if (d < 1.0f || fabsf(normalizeOrientation(atan2f(command->Param2 - telemetryData.Y, command->Param1 - telemetryData.X) - telemetryData.O)) > 60.0f * DEGREES_TO_RAD)
			break;

		/*
		 * Compute regulator values based on Papers-RAS_Blazic_2011.pdf ignoring position regulation
		 * Applying slight modifications to 'v' and 'w', saturating
		 */
		const float k = 0.0003f;						/*<< Gain for turning */
		float dx = command->Param1 - telemetryData.X;	/*<< Distance to target perpendicular to global coordinate X */
		float dy = command->Param2 - telemetryData.Y;	/*<< Distance to target parallel to global coordinate Y */
		float cosfi = cosf(telemetryData.O);			/*<< Cos of robot orientation angle */
		float sinfi = sinf(telemetryData.O);			/*<< Sin of robot orientation angle */
		//float ex = cosfi * dx + sinfi * dy;				/*<< Distance to target perpendicular to robot wheel axis */
		float ey = -sinfi * dx + cosfi * dy;			/*<< Distance to target parallel to robot wheel axis */
		float v = command->Speed;						/*<< Linear speed - always maximum */
		float w = k * command->Speed * ey;				/*<< Rotational speed - depending on ey; the bigger turn to make the bigger the speed is */

		/* Begin to slow down if closer than 5cm from target */
		if (d < 50.0f) {
			v = v * (0.7f * d / 50.0f + 0.3f);
		}

		/* Limit maximal rotational speed to half of maximum speed */
		if (fabsf(w) > command->Speed / 2.0f) {
			w = copysignf(command->Speed / 2.0f, w);
		}

		/* Set wheels speeds */
		sendSpeeds(v - w * ROBOT_DIAM / 2.0f, v + w * ROBOT_DIAM / 2.0f, portMAX_DELAY);

		/* Wait a moment */
		vTaskDelayUntil(&wakeTime, TASKDRIVE_BASEDELAY_MS/portTICK_RATE_MS);
	}
}

void turnRads(float rads, MotorSpeed_Struct speed, float epsilon, Smoothness_Type smoothness) {
	portTickType wakeTime = xTaskGetTickCount();
	TelemetryData_Struct telemetryData;

	/* Read initial telemetry */
	getTelemetryRaw(&telemetryData);

	/* Compute target orientation */
	float targetO = telemetryData.O + rads;
	float error;

	while(1) {
		getTelemetryRaw(&telemetryData);
		error = targetO - telemetryData.O;
		if (fabsf(error) < epsilon || error*rads < 0.0f)
			break;

		// smoothness here

		sendSpeeds(speed.LeftSpeed, speed.RightSpeed, portMAX_DELAY);
		vTaskDelayUntil(&wakeTime, TASKDRIVE_BASEDELAY_MS/portTICK_RATE_MS);
	}
	sendSpeeds(0,0,portMAX_DELAY);
}

void driveAngleArc(const DriveCommand_Struct* command) {
	if (command->Type != DriveCommand_Type_Angle && command->Type != DriveCommand_Type_Arc) return;

	portTickType wakeTime = xTaskGetTickCount();

	TelemetryData_Struct telemetryData, begTelData;

	const float breakingAngle = 45.0f * DEGREES_TO_RAD;

	/* Get starting point telemetry data */
	getTelemetryScaled(&begTelData);

	/* Calculate target orientation */
	float targetO = command->Param2 * DEGREES_TO_RAD;
	if (command->Param1 < 0.5f || command->Type == DriveCommand_Type_Arc)
		targetO += begTelData.O;
	targetO = normalizeOrientation(targetO);

	/* Calculate direction; dir == 1 - turning left */
	int8_t dir = ((targetO > begTelData.O && fabsf(targetO - begTelData.O) < M_PI) || (targetO < begTelData.O && fabsf(begTelData.O - targetO) > M_PI) ? 1 : -1);

	/* Compute maximal speeds for both wheels */
	const float maxLeft = command->Speed * (command->Type == DriveCommand_Type_Angle ?
			(float)dir * -1.0f :
			1.0f - (float)dir*ROBOT_DIAM/(2.0f*command->Param1*globalPositionScale) );
	const float maxRight = command->Speed * (command->Type == DriveCommand_Type_Angle ?
			(float)dir :
			1.0f + (float)dir*ROBOT_DIAM/(2.0f*command->Param1*globalPositionScale) );

	/* Turn with maximum speed as long as turning angle is big */
	if (fabsf(normalizeOrientation(targetO - begTelData.O)) > breakingAngle) {
		/* Set maximum speed */
		sendSpeeds(maxLeft, maxRight, portMAX_DELAY);

		/* Wait for reaching close proximity of target angle */
		while(1) {
			getTelemetryScaled(&telemetryData);
			if (fabsf(normalizeOrientation(targetO - telemetryData.O)) < breakingAngle) break;
			vTaskDelayUntil(&wakeTime, 3*TASKDRIVE_BASEDELAY_MS/portTICK_RATE_MS);
		}
	}

	/* Regulate speed to allow gentle target angle approaching with desired accuracy */
	while(1) {
		/* Read current orientation */
		getTelemetryScaled(&telemetryData);
		/* Compute angle distance to target */
		float dist = fabsf(normalizeOrientation(telemetryData.O - targetO));

		/* End if distance is very small or direction changes */
		if (dist < 0.25f * DEGREES_TO_RAD ||
				((targetO > telemetryData.O && targetO - telemetryData.O < M_PI) ||
				 (targetO < telemetryData.O && telemetryData.O - targetO > M_PI) ? 1 : -1) != dir)
			break;

		/* Calculate speeds */
		float speedCoef = 0.9f * dist / breakingAngle + 0.1f;
		sendSpeeds(maxLeft * speedCoef, maxRight * speedCoef, portMAX_DELAY);

		/* Wait a little */
		vTaskDelayUntil(&wakeTime, TASKDRIVE_BASEDELAY_MS/portTICK_RATE_MS);
	}
}

bool isCurrentlyDriving() {
	return isDriving;
}

bool isThereMoreDrivingCommands(void) {
	return uxQueueMessagesWaiting(driveQueue);
}

void TaskDriveConstructor() {
	driveQueue = xQueueCreate(100, sizeof(DriveCommand_Struct*));		// holding pointers because there's a lot of big structures that are processed rather slowly. Memory is allocated dynamically
	xTaskCreate(TaskDrive, NULL, TASKDRIVE_STACKSPACE, NULL, PRIORITY_TASK_DRIVE, &driveTask);
}
