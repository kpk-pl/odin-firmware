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
 * @param wakeTime Wake time from calling function, allowing to synchronize time between calls
 */
static void driveLine(const DriveCommand_Struct* command, portTickType* wakeTime);
/**
 * \brief Drives to target point. First turn in the direction of target point, then drive to it.
 * @param command Drive command pointer
 * @param wakeTime Wake time from calling function, allowing to synchronize time between calls
 */
static void drivePoint(const DriveCommand_Struct* command, portTickType* wakeTime);
/**
 * \brief Turns or drives over circular trajectory. Note that it cannot turn by more than 180 degrees ralative.
 * @param command Drive command pointer
 * @param wakeTime Wake time from calling function, allowing to synchronize time between calls
 */
static void driveAngleArc(const DriveCommand_Struct* command, portTickType* wakeTime);

xQueueHandle driveQueue;	/*!< Queue with drive commands. It should contain type (DriveCommand_Struct*) */
xTaskHandle driveTask;		/*!< This task handler */

void TaskDrive(void * p) {
	portTickType wakeTime = xTaskGetTickCount();
	DriveCommand_Struct * command;
	bool taken = false;

	while(1) {
		/* Take one command from queue or wait for the command to arrive */
		if (uxQueueMessagesWaiting(driveQueue) == 0) {		// free resources
			sendSpeeds(0.0f, 0.0f, portMAX_DELAY);			// stop motors as there is no command available
			if (taken) {
				xSemaphoreGive(motorControllerMutex);
				taken = false;
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
			xQueueSendToBack(penCommandQueue, &command->UsePen, portMAX_DELAY);

			/* Recalculate speed from m/s to rad/s */
			command->Speed = command->Speed * 1000.0f / RAD_TO_MM_TRAVELED;

			if (command->Type == DriveCommand_Type_Line)
				driveLine(command, &wakeTime);
			else if (command->Type == DriveCommand_Type_Angle || command->Type == DriveCommand_Type_Arc)
				driveAngleArc(command, &wakeTime);
			else if (command->Type == DriveCommand_Type_Point)
				drivePoint(command, &wakeTime);
		}
		else { /* command->Speed < 0.0f */
			if (globalLogEvents) safePrint(34, "Speed cannot be less than zero!\n");
		}

		/* Free space where the command was held */
		vPortFree(command);
	}
}

void driveLine(const DriveCommand_Struct* command, portTickType* wakeTime) {
	if (command->Type != DriveCommand_Type_Line) return;

	TelemetryData_Struct telemetryData, begTelData;

	if (globalLogEvents) safePrint(20, "Driving %.0fmm\n", command->Param1);

	const float breakingDistance = 100.0f;
	const float dist = fabsf(command->Param1);
	const float maxSpeed = copysignf(1.0f, (command->Param1)) * command->Speed;

	/* Get starting point telemetry data */
	getTelemetry(&begTelData);

	/* Start driving at max speed if far away from target */
	if (dist > breakingDistance) {
		/* Set motors speed allowing negative speed */
		sendSpeeds(maxSpeed, maxSpeed, portMAX_DELAY);

		/* Wait in periods until position is too close to target position */
		while(1) {
			getTelemetry(&telemetryData);
			if (hypotf(telemetryData.X - begTelData.X, telemetryData.Y - begTelData.Y) >= dist - breakingDistance)
				break;
			vTaskDelayUntil(wakeTime, 3*TASKDRIVE_BASEDELAY_MS/portTICK_RATE_MS);
		}
	}

	/* Regulate speed to gently approach target with desired accuracy */
	while(1) {
		/* Read current position */
		getTelemetry(&telemetryData);

		/* Calculate remaining distance */
		float rem = dist - hypotf(telemetryData.X - begTelData.X, telemetryData.Y - begTelData.Y);

		/* Finish if distance is very very small */
		if (rem < 0.5f) break;

		/* Set motors speed constantly as robot approaches target distance */
		float speed = maxSpeed * (0.8f * rem / breakingDistance + 0.2f);
		sendSpeeds(speed, speed, portMAX_DELAY);
	}
	/* Wait for a bit */
	vTaskDelayUntil(wakeTime, TASKDRIVE_BASEDELAY_MS/portTICK_RATE_MS);
}

void drivePoint(const DriveCommand_Struct* command, portTickType* wakeTime) {
	if (command->Type != DriveCommand_Type_Point) return;

	TelemetryData_Struct telemetryData, begTelData;

	if (globalLogEvents) safePrint(44, "Driving to point X:%.1fmm Y:%.1fmm\n", command->Param1, command->Param2);

	/* First of all, turn to target point with desired accuracy */
	/* Get starting point telemetry data */
	getTelemetry(&begTelData);

	/* Calculate target orientation */
	float targetO = normalizeOrientation(atan2f(command->Param2 - begTelData.Y, command->Param1 - begTelData.X));
	/* Calculate direction; dir == 1 - turning left */
	int8_t dir = ((targetO > begTelData.O && fabsf(targetO - begTelData.O) < M_PI) || (targetO < begTelData.O && fabsf(begTelData.O - targetO) > M_PI) ? 1 : -1);

	/* Start turning if targetO is bigger than threshold */
	if (fabsf(targetO) > 20.0f * DEGREES_TO_RAD) {
		/* Compute speeds and send them to queue */
		float leftspeed = (float)dir * (-1.0f) * command->Speed;
		sendSpeeds(leftspeed, -leftspeed, portMAX_DELAY);

		/* Wait for angle distance to become small enough */
		while(1) {
			/* Read current telemetry data */
			getTelemetry(&telemetryData);

			/* End turning if close enough to target angle */
			if (fabsf(normalizeOrientation(atan2f(command->Param2 - telemetryData.Y, command->Param1 - telemetryData.X) - telemetryData.O)) < 20.0f * DEGREES_TO_RAD)
				break;

			/* Wait for a while */
			vTaskDelayUntil(wakeTime, 3*TASKDRIVE_BASEDELAY_MS/portTICK_RATE_MS);
		}
	}

	/* Start regulator - driving to point requires constant speeds updates */
	while(1) {
		/* Read current position */
		getTelemetry(&telemetryData);

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
		vTaskDelayUntil(wakeTime, TASKDRIVE_BASEDELAY_MS/portTICK_RATE_MS);
	}
}

void driveAngleArc(const DriveCommand_Struct* command, portTickType* wakeTime) {
	if (command->Type != DriveCommand_Type_Angle && command->Type != DriveCommand_Type_Arc) return;

	TelemetryData_Struct telemetryData, begTelData;

	if (globalLogEvents) {
		if (command->Type == DriveCommand_Type_Angle) {
			safePrint(30, "Turning by %.2f %s\n", command->Param2, (command->Param1 < 0.5f ? "relative" : "absolute"));
		}
		else {
			safePrint(58, "Turning with radius %.2fmm and %.1f degrees length\n", command->Param1, command->Param2);
		}
	}

	const float breakingAngle = 45.0f * DEGREES_TO_RAD;

	/* Get starting point telemetry data */
	getTelemetry(&begTelData);

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
			1.0f - (float)dir*ROBOT_DIAM/(2.0f*command->Param1) );
	const float maxRight = command->Speed * (command->Type == DriveCommand_Type_Angle ?
			(float)dir :
			1.0f + (float)dir*ROBOT_DIAM/(2.0f*command->Param1) );

	/* Turn with maximum speed as long as turning angle is big */
	if (fabsf(normalizeOrientation(targetO - begTelData.O)) > breakingAngle) {
		/* Set maximum speed */
		sendSpeeds(maxLeft, maxRight, portMAX_DELAY);

		/* Wait for reaching close proximity of target angle */
		while(1) {
			getTelemetry(&telemetryData);
			if (fabsf(normalizeOrientation(targetO - telemetryData.O)) < breakingAngle) break;
			vTaskDelayUntil(wakeTime, 3*TASKDRIVE_BASEDELAY_MS/portTICK_RATE_MS);
		}
	}

	/* Regulate speed to allow gentle target angle approaching with desired accuracy */
	while(1) {
		/* Read current orientation */
		getTelemetry(&telemetryData);
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
		vTaskDelayUntil(wakeTime, TASKDRIVE_BASEDELAY_MS/portTICK_RATE_MS);
	}
}

void TaskDriveConstructor() {
	driveQueue = xQueueCreate(100, sizeof(DriveCommand_Struct*));		// holding pointers because there's a lot of big structures that are processed rather slowly. Memory is allocated dynamically
	xTaskCreate(TaskDrive, NULL, TASKDRIVE_STACKSPACE, NULL, PRIORITY_TASK_DRIVE, &driveTask);
}
