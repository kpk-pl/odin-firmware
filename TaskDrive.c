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

/**
 * \brief Drives to target point. First turn in the direction of target point, then drive to it.
 * @param command Drive command pointer
 * @param smooth if true then robot will slow down at the end
 */
static void drivePoint(const DriveCommand_Struct* command, const bool smooth);
/**
 * \brief Turns or drives over circular trajectory.
 * @param command Drive command pointer
 * @param smooth if true then robot will slow down at the end
 */
static void driveAngleArc(const DriveCommand_Struct* command, const bool smooth);

/**
 * \brief Sets wheels speeds and waits for the robot to turn specified angle
 * @param rads Angle that the robot should turn before stopping
 * @param speed Wheels' speeds
 * @param epsilon Error angle which is acceptable. Robot will stop if distance to target is less than epsilon
 * @param smooth if true then robot will slow down at the end
 */
static void turnRads(const float rads, const MotorSpeed_Struct speed, const float epsilon, const bool smooth);

/**
 * \brief Checks if there is more driving commands to perform
 * @return true if more commands awaits execution
 */
static bool isThereMoreDrivingCommands(void);

xQueueHandle driveQueue;	/*!< Queue with drive commands. It should contain type (DriveCommand_Struct*) */
xTaskHandle driveTask;		/*!< This task handler */

void TaskDrive(void * p) {
	DriveCommand_Struct * command;
	bool taken = false;

	while(1) {
		/* Take one command from queue or wait for the command to arrive */
		if (!isThereMoreDrivingCommands()) {				// free resources
			sendSpeeds(0.0f, 0.0f);							// stop motors as there is no command available
			if (taken) {
				xSemaphoreGive(motorControllerMutex);
				taken = false;
				xQueueOverwrite(penCommandQueue, &taken);	// set pen up, use 'taken' variable as it is false either way
			}
		}

		xQueueReceive(driveQueue, &command, portMAX_DELAY);

		if (!taken) {	// block resources
			xSemaphoreTake(motorControllerMutex, portMAX_DELAY);
			taken = true;
		}

		/* Check if speed is not less than zero */
		if (command->Speed >= 0.0f) {
			/* Handle pen */
			xQueueOverwrite(penCommandQueue, &command->UsePen);

			/* Recalculate speed from m/s to rad/s */
			command->Speed = command->Speed * 1000.0f / RAD_TO_MM_TRAVELED;

			/* If no more commands, then slow down at the end */
			bool smooth = !isThereMoreDrivingCommands() || command->Smooth;

			if (command->Type == DriveCommand_Type_Line) {
				safeLog(Log_Type_Drive, 20, "Driving %.0fmm\n", command->Param1);

				/* Change to driving to point */
				command->Type = DriveCommand_Type_Point;
				TelemetryData_Struct telemetry;
				getTelemetry(&telemetry, TelemetryStyle_Common);
				command->Param2 = telemetry.Y + sinf(telemetry.O) * command->Param1;
				command->Param1 = telemetry.X + cosf(telemetry.O) * command->Param1;
				drivePoint(command, smooth);
			}
			else if (command->Type == DriveCommand_Type_Angle || command->Type == DriveCommand_Type_Arc) {
				if (command->Type == DriveCommand_Type_Angle) {
					safeLog(Log_Type_Drive, 30, "Turning by %.2f %s\n", command->Param2, (command->Param1 < 0.5f ? "relative" : "absolute"));
				}
				else {
					safeLog(Log_Type_Drive, 58, "Turning with radius %.2fmm and %.1f degrees length\n", command->Param1, command->Param2);
				}
				driveAngleArc(command, smooth);
			}
			else if (command->Type == DriveCommand_Type_Point) {
				safeLog(Log_Type_Drive, 44, "Driving to point X:%.1fmm Y:%.1fmm\n", command->Param1, command->Param2);
				drivePoint(command, smooth);
			}
		}
		else { /* command->Speed < 0.0f */
			safeLog(Log_Type_Error, 34, "Speed cannot be less than zero!\n");
		}

		/* Free space where the command was held */
		vPortFree(command);
	}
}

void drivePoint(const DriveCommand_Struct* command, const bool smooth) {
	if (command->Type != DriveCommand_Type_Point) return;

	portTickType wakeTime = xTaskGetTickCount();
	TelemetryData_Struct telemetryData;
	float smoothDistance = 50.0f / globalPositionScale;

	/* First of all, turn to target point with desired accuracy */
	/* Get starting point telemetry data */
	getTelemetry(&telemetryData, TelemetryStyle_Scaled);

	/* Calculate angle to target */
	float angO = normalizeOrientation(atan2f(command->Param2 - telemetryData.Y, command->Param1 - telemetryData.X) - telemetryData.O);

	if (fabsf(angO) > 10.0 * DEGREES_TO_RAD) {
		float rightSpeed = copysignf(command->Speed, angO);
		turnRads(angO,
				(MotorSpeed_Struct)
					{.LeftSpeed = -rightSpeed,
					 .RightSpeed = rightSpeed},
				1.0f * DEGREES_TO_RAD,
				false);
	}

	/* Start regulator - driving to point requires constant speeds updates */
	while(1) {
		/* Read current position */
		getTelemetry(&telemetryData, TelemetryStyle_Common);

		/* Finish up if target is really close or robot's missing the target */
		float d = hypotf(telemetryData.X - command->Param1, telemetryData.Y - command->Param2);
		if (d < 1.0f / globalPositionScale || fabsf(normalizeOrientation(atan2f(command->Param2 - telemetryData.Y, command->Param1 - telemetryData.X) - telemetryData.O)) > 60.0f * DEGREES_TO_RAD)
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

		/* Slow down based on smoothness */
		if (smooth && d < smoothDistance) { // smooth ending - more important than beginning
			v = v * (0.7f * d / smoothDistance + 0.3f);
		}

		/* Limit maximal rotational speed to half of maximum speed */
		if (fabsf(w) > command->Speed / 2.0f) {
			w = copysignf(command->Speed / 2.0f, w);
		}

		/* Set wheels speeds */
		sendSpeeds(v - w * ROBOT_DIAM / 2.0f, v + w * ROBOT_DIAM / 2.0f);

		/* Wait a moment */
		vTaskDelayUntil(&wakeTime, TASKDRIVE_BASEDELAY_MS/portTICK_RATE_MS);
	}
}

void driveAngleArc(const DriveCommand_Struct* command, const bool smooth) {
	if (command->Type != DriveCommand_Type_Angle && command->Type != DriveCommand_Type_Arc) return;

	TelemetryData_Struct telemetryData;

	/* Get starting point telemetry data */
	getTelemetry(&telemetryData, TelemetryStyle_Common);

	/* Calculate angle to turn */
	float angle = command->Param2 * DEGREES_TO_RAD;
	if (command->Param1 > 0.5f && command->Type == DriveCommand_Type_Angle)
		angle = normalizeOrientation(angle - telemetryData.O); // shortest path to target angle

	float direction = copysignf(1.0f, angle);

	MotorSpeed_Struct speeds;
	if (command->Type == DriveCommand_Type_Angle) {
		speeds.RightSpeed = direction*command->Speed;
		speeds.LeftSpeed = -speeds.RightSpeed;
	}
	else {
		/* Calculate speeds as if there was no maximum speed */
		float adjustment = direction*ROBOT_DIAM/(2.0f*command->Param1*globalPositionScale);
		speeds.LeftSpeed = (1.0f - adjustment);
		speeds.RightSpeed = (1.0f + adjustment);
		/* Correct both speeds so that maximum speed is not exceeded */
		adjustment = command->Speed / fmaxf(speeds.LeftSpeed, speeds.RightSpeed);
		speeds.LeftSpeed *= adjustment ;
		speeds.RightSpeed *= adjustment;
	}

	turnRads(angle, speeds, 0.1f * DEGREES_TO_RAD, smooth);
}

void turnRads(const float rads, const MotorSpeed_Struct speed, const float epsilon, const bool smooth) {
	portTickType wakeTime = xTaskGetTickCount();
	TelemetryData_Struct telemetryData;
	float smoothAngle = 10.0f * DEGREES_TO_RAD;

	/* Read initial telemetry */
	getTelemetry(&telemetryData, TelemetryStyle_Raw);

	/* Compute target orientation */
	float targetO = telemetryData.O + rads;
	float error, abserror;

	while(1) {
		getTelemetry(&telemetryData, TelemetryStyle_Raw);
		error = targetO - telemetryData.O;
		abserror = fabsf(error);
		if (abserror < epsilon || error*rads < 0.0f)
			break;

		MotorSpeed_Struct adjspeed = speed;

		if (smooth && abserror < smoothAngle) {
			float adjustment = (0.7f * abserror / smoothAngle + 0.3f);
			adjspeed.LeftSpeed = speed.LeftSpeed * adjustment;
			adjspeed.RightSpeed = speed.RightSpeed * adjustment;
		}

		sendSpeeds(adjspeed.LeftSpeed, adjspeed.RightSpeed);
		vTaskDelayUntil(&wakeTime, TASKDRIVE_BASEDELAY_MS/portTICK_RATE_MS);
	}
}

bool isThereMoreDrivingCommands(void) {
	return uxQueueMessagesWaiting(driveQueue);
}

void TaskDriveConstructor() {
	driveQueue = xQueueCreate(100, sizeof(DriveCommand_Struct*));		// holding pointers because there's a lot of big structures that are processed rather slowly. Memory is allocated dynamically
	xTaskCreate(TaskDrive, NULL, TASKDRIVE_STACKSPACE, NULL, PRIORITY_TASK_DRIVE, &driveTask);
}
