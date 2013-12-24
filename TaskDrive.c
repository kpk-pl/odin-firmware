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
 * \brief Drives over a straight line to target point.
 * @param command Drive command pointer
 */
static void driveLine(const DriveCommand_Struct* command);
/**
 * \brief Drives to target point. First turn in the direction of target point, then drive to it.
 * @param command Drive command pointer
 * @param print Set to true if it is the primary function called, set to false if other drive function calls in
 */
static void drivePoint(const DriveCommand_Struct* command, bool print, bool smooth);
/**
 * \brief Turns or drives over circular trajectory using relative angles. It may turn the robot by any number of degrees (more than 180)
 * @param command Drive command pointer
 */
static void driveAngleArc(const DriveCommand_Struct* command);
/**
 * \brief Turns the robot so that is has the normalized orientation angle equal to the one specified in command.
 * It chooses the shortest path to turn
 * @param command Drive command pointer
 */
static void driveAngle(const DriveCommand_Struct* command);

static void turnAngleRelative(float angle, float speed, float eps, bool smooth);

/**
 * \brief Checks if there is more driving commands to perform
 * @return true if more commands awaits execution
 */
static bool isThereMoreDrivingCommands(void);

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

			if (command->Type == DriveCommand_Type_Line)
				driveLine(command);
			else if (command->Type == DriveCommand_Type_Angle)
				driveAngle(command);
			else if (command->Type == DriveCommand_Type_Arc)
				driveAngleArc(command);
			else if (command->Type == DriveCommand_Type_Point)
				drivePoint(command, true, true);
		}
		else { /* command->Speed < 0.0f */
			if (globalLogEvents) safePrint(34, "Speed cannot be less than zero!\n");
		}

		/* Free space where the command was held */
		vPortFree(command);
	}
}

void driveLine(const DriveCommand_Struct* command) {
	if (command->Type != DriveCommand_Type_Line) return;

	if (globalLogEvents) safePrint(20, "Driving %.0fmm\n", command->Param1);

	/* Get starting point telemetry data */
	TelemetryData_Struct telemetryData;
	getTelemetryRawScaled(&telemetryData);

	/* Create new driving command based on target point */
	DriveCommand_Struct newCommand;
	newCommand.Type = DriveCommand_Type_Point;
	newCommand.Param1 = telemetryData.X + command->Param1 * cosf(telemetryData.O);
	newCommand.Param2 = telemetryData.Y + command->Param1 * sinf(telemetryData.O);
	newCommand.Speed = command->Speed;
	newCommand.UsePen = command->UsePen;

	/* Issue driving to point with new command */
	drivePoint(&newCommand, false, true);
}

void drivePoint(const DriveCommand_Struct* command, bool print, bool smooth) {
	if (command->Type != DriveCommand_Type_Point) return;

	portTickType wakeTime = xTaskGetTickCount();
	TelemetryData_Struct telemetryData;

	if (globalLogEvents && print) safePrint(44, "Driving to point X:%.1fmm Y:%.1fmm\n", command->Param1, command->Param2);

	/* First of all, turn to target point with desired accuracy */
	/* Get starting point telemetry data */
	getTelemetryScaled(&telemetryData);

	/* Calculate relative target orientation */
	float targetO = normalizeOrientation(atan2f(command->Param2 - telemetryData.Y, command->Param1 - telemetryData.X) - telemetryData.O);

	/* Start turning if targetO is bigger than threshold */
	if (fabsf(targetO) > 50.0f * DEGREES_TO_RAD) {
		turnAngleRelative(targetO, command->Speed, DEGREES_TO_RAD * 2.0f, false);
	}

	/* Start regulator - driving to point requires constant speeds updates */
	while(1) {
		/* Read current position */
		getTelemetryScaled(&telemetryData);

		/* Finish up if target is really close or robot's missing the target */
		float d = hypotf(telemetryData.X - command->Param1, telemetryData.Y - command->Param2);
		if (d < 1.0f || fabsf(normalizeOrientation(atan2f(command->Param2 - telemetryData.Y, command->Param1 - telemetryData.X) - telemetryData.O)) > 90.0f * DEGREES_TO_RAD)
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
		float ey = -sinfi * dx + cosfi * dy;			/*<< Distance to target parallel to robot wheel axis */
		float v = command->Speed;						/*<< Linear speed - always maximum */
		float w = k * command->Speed * ey;				/*<< Rotational speed - depending on ey; the bigger turn to make the bigger the speed is */

		/* Begin to slow down if closer than 5cm from target and no more commands are available at the moment */
		if (smooth && !isThereMoreDrivingCommands()) {
			if (d < 50.0f) {
				v = v * (0.7f * d / 50.0f + 0.3f);
			}
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

void driveAngleArc(const DriveCommand_Struct* command) {
	// return if absolute angle was given
	if (command->Type != DriveCommand_Type_Arc) return;
	if (command->Param1 <= 0.0f) return; // disallow negative radius

	portTickType wakeTime = xTaskGetTickCount();
	TelemetryData_Struct telemetryData;

	if (globalLogEvents) {
		safePrint(58, "Turning with radius %.2fmm and %.1f degrees length\n", command->Param1, command->Param2);
	}

	const float breakingAngle = 15.0f * DEGREES_TO_RAD;

	/* Get starting point telemetry data */
	getTelemetryRaw(&telemetryData);

	/* Calculate target orientation */
	float targetO = command->Param2 * DEGREES_TO_RAD + telemetryData.O;

	/* Calculate direction; dir == 1 - turning left */
	int8_t dir = (command->Param2 > 0 ? 1 : -1);

	/* Compute maximal speeds for both wheels */
	const float maxLeft = command->Speed * (1.0f - (float)dir*ROBOT_DIAM/(2.0f*command->Param1*globalPositionScale) );
	const float maxRight = command->Speed * (1.0f + (float)dir*ROBOT_DIAM/(2.0f*command->Param1*globalPositionScale) );

	/* Turn with maximum speed as long as turning angle is big */
	if (fabsf(targetO - telemetryData.O) > breakingAngle) {
		/* Set maximum speed */
		sendSpeeds(maxLeft, maxRight, portMAX_DELAY);

		/* Wait for reaching close proximity of target angle */
		while(1) {
			getTelemetryRaw(&telemetryData);
			if (fabsf(targetO - telemetryData.O) < breakingAngle) break;
			vTaskDelayUntil(&wakeTime, TASKDRIVE_BASEDELAY_MS/portTICK_RATE_MS);
		}
	}

	/* Regulate speed to allow gentle target angle approaching with desired accuracy */
	while(1) {
		/* Read current orientation */
		getTelemetryRaw(&telemetryData);
		/* Compute angle distance to target */
		float dist = fabsf(telemetryData.O - targetO);

		/* End if distance is very small or direction changes */
		if (dist < 0.25f * DEGREES_TO_RAD || (telemetryData.O - targetO)*dir > 0.0f)
			break;

		/* Calculate speeds */
		if (!isThereMoreDrivingCommands()) {
			float speedCoef = 0.9f * dist / breakingAngle + 0.1f;
			sendSpeeds(maxLeft * speedCoef, maxRight * speedCoef, portMAX_DELAY);
		}

		/* Wait a little */
		vTaskDelayUntil(&wakeTime, TASKDRIVE_BASEDELAY_MS/portTICK_RATE_MS);
	}
}

void driveAngle(const DriveCommand_Struct* command) {
	// return if not turning by absolute angle angle was given
	if (command->Type != DriveCommand_Type_Angle) return;

	if (globalLogEvents) {
		if (command->Param1 > 0.5f)
			safePrint(34, "Turning to %.2f deg absolute\n", command->Param2);
		else
			safePrint(34, "Turning by %.2f deg relative\n", command->Param2);
	}

	float ang = command->Param2 * DEGREES_TO_RAD;
	if (command->Param1 > 0.5f) {
		TelemetryData_Struct telemetryData;
		getTelemetryScaled(&telemetryData);
		ang = normalizeOrientation(ang-telemetryData.O);
	}

	turnAngleRelative(ang, command->Speed, DEGREES_TO_RAD*0.25f, true);
}

void turnAngleRelative(float angle, float speed, float eps, bool smooth) {
	assert_param(eps >= 0.0f);
	if (fabsf(angle) < eps) return;

	portTickType wakeTime = xTaskGetTickCount();
	TelemetryData_Struct telemetryData;

	getTelemetryRaw(&telemetryData);
	float targetO = telemetryData.O + angle;
	float prevDist = fabsf(angle);
	const float speedRight = copysign(speed, angle);

	/* Regulate speed to allow gentle target angle approaching with desired accuracy */
	while(1) {
		/* Read current orientation */
		getTelemetryRaw(&telemetryData);
		/* Compute angle distance to target */
		float dist = fabsf(telemetryData.O - targetO);

		/* End if distance is very small or direction changes */
		if (dist < eps || prevDist - dist < 0.0f)
			break;

		/* Calculate speeds */
		if (smooth && !isThereMoreDrivingCommands() && (dist < 10.0f * DEGREES_TO_RAD)) {
			float coef = 0.9f * dist / (10.0f * DEGREES_TO_RAD) + 0.1f;
			sendSpeeds(-coef*speedRight, coef*speedRight, portMAX_DELAY);
		}
		else {
			sendSpeeds(-speedRight, speedRight, portMAX_DELAY);
		}

		prevDist = dist;

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
