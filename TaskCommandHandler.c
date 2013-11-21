#include <string.h>
#include <stdlib.h>

#include "priorities.h"
#include "stackSpace.h"

#include "TaskCommandHandler.h"
#include "main.h"
#include "compilation.h"
#include "commands.h"
#include "pointsBuffer.h"	// for starting points download
#include "hwinterface.h"

#include "TaskDrive.h"		// for typedefs
#include "TaskMotorCtrl.h"
#include "TaskPrintfConsumer.h"
#include "TaskTelemetry.h"
#include "TaskPenCtrl.h"
#ifdef FOLLOW_TRAJECTORY
#include "TaskTrajectory.h"
#endif

xQueueHandle commandQueue;	/*!< Queue with pointers to messages. It should contain type (char*) */
xTaskHandle commandHandlerTask;

/*
 * \brief Handles various commands from various places.
 * @param command Pointer to the beginning of a string containing command
 * @return None
 */
static void COMHandle(const char * command);

void TaskCommandHandler(void * p) {
	char * msg;
	while(1) {
		/* Block till message is available */
		xQueueReceive(commandQueue, &msg, portMAX_DELAY);
		/* Handle message */
		COMHandle(msg);
		/* Free allocated resources */
		vPortFree(msg);
	}
}

void COMHandle(const char * command) {
#ifdef DRIVE_COMMANDS
	DriveCommand_Struct *dc;
#endif
	char *last;
	TelemetryData_Struct td;
	float temp_float;

	const char wrongComm[] = "Incorrect command\n";

	bool commandCheck(const bool p) {
		if (p != true) {
			if (globalLogEvents) safePrint(20, "%s", wrongComm);
		}
		return p;
	}
#ifdef DRIVE_COMMANDS
	void waitForDrivingEnd() {
		while (isCurrentlyDriving()) {
			vTaskDelay(10/portTICK_RATE_MS);
		}
	}
#endif

	switch(command[0]) {
	case LOW_LEVEL_AUA:
		GPIO_ToggleBits(LEDS_GPIO_1, LEDS_GPIO_1_PIN);
		break;
	case HIGH_LEVEL_AUA:
		safePrint(8, "Hello!\n");
		break;
	case AVAILABLE_MEMORY:
		safePrint(32, "Available memory: %dkB\n", xPortGetFreeHeapSize());
		break;
#ifdef FOLLOW_TRAJECTORY
	case IMPORT_TRAJECTORY_POINTS:
		if (commandCheck( strlen(command) >= 3) ) {
			safePrint(36, "<#Please send only %d points#>\n",
				TBloadNewPoints(strtol((char*)&command[2], NULL, 10)));
		}
		break;
#endif
	case BATTERY_VOLTAGE:
		safePrint(26, "Battery voltage: %.2fV\n", getBatteryVoltage());
		break;
	case PEN_DOWN:
		setPenDown();
		if (globalLogEvents) safePrint(10, "Pen down\n");
		break;
	case PEN_UP:
		setPenUp();
		if (globalLogEvents) safePrint(8, "Pen up\n");
		break;
	case MOTOR_LEFT_SPEED:
		if (commandCheck( strlen(command) >= 3 )) {
			globalSpeedRegulatorOn = DISABLE;
			setMotorLSpeed(strtof((char*)&command[2], NULL)/100.0f);
		}
		break;
	case MOTOR_RIGHT_SPEED:
		if (commandCheck( strlen(command) >= 3 )) {
			globalSpeedRegulatorOn = DISABLE;
			setMotorRSpeed(strtof((char*)&command[2], NULL)/100.0f);
		}
		break;
	case MOTOR_LEFT_ENCODER:
		safePrint(22, "Encoder left: %ld\n", getEncoderL());
		break;
	case MOTOR_RIGHT_ENCODER:
		safePrint(23, "Encoder right: %ld\n", getEncoderR());
		break;
	case MOTORS_ENABLE:
		if (commandCheck( strlen(command) >= 3 )) {
			if (command[2] == '0') {
				globalSpeedRegulatorOn = DISABLE;
				enableMotors(DISABLE);
				if (globalLogEvents) safePrint(19, "Motors disabled\n");
			}
			else {
				enableMotors(ENABLE);
				globalSpeedRegulatorOn = ENABLE;
				if (globalLogEvents) safePrint(18, "Motors enabled\n");
			}
		}
		break;
	case MOTORS_SET_SPEEDS:
		if (commandCheck( strlen(command) >= 3 )) {
			globalSpeedRegulatorOn = ENABLE;
			temp_float = strtof((char*)&command[2], &last);
			sendSpeeds(temp_float, strtof(last+1, NULL), 0);
		}
		break;
	case TELEMETRY_PRINT:
		getTelemetry(&td);
		safePrint(40, "X:%.2f Y:%.2f O:%.1f\n", td.X, td.Y, td.O / DEGREES_TO_RAD);
		break;
	case CPU_RESET:
		IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
		IWDG_SetReload(1);
		break;
	case WIFI_MODE:
		if (commandCheck( strlen(command) >= 3 )) {
			setWiFiMode(command[2] == 'c' ? WiFiMode_Command : WiFiMode_Data);
			if (globalLogEvents) safePrint(20, "WiFi mode set to %d\n", getWiFiMode());
		}
		break;
	case WIFI_RESET:
		if (commandCheck( strlen(command) >= 3 )) {
			setWiFiReset(command[2] == '1' ? ENABLE : DISABLE);
			if (globalLogEvents) safePrint(24, "WiFi reset changed to %d\n", getWiFiReset());
		}
		break;
#ifdef USE_IMU_TELEMETRY
	case USE_IMU_UPDATES:
		if (commandCheck( strlen(command) >= 3 )) {
			globalUseIMUUpdates = command[2] == '1';
			if (globalLogEvents) safePrint(26, "IMU updates changed to %d\n", globalUseIMUUpdates);
		}
		break;
#endif
	case LANTERN_ENABLE:
		if (commandCheck( strlen(command) >= 3 )) {
			enableLantern(command[2] == '1' ? ENABLE : DISABLE);
			if (globalLogEvents) safePrint(28, "Lantern changed state to %d\n", getLanternState());
		}
		break;
	case CPU_USAGE:
		safePrint(19, "CPU Usage: %.1f%%\n", globalCPUUsage*100.0f);
		break;
	case STACK_USAGE:
		reportStackUsage();
		break;
	case SET_PEN_LINE:
		if (commandCheck( strlen(command) >= 3 && IS_PENLINE_TYPE(command[2] - '0') )) {
			setPenLineType(command[2] - '0');
		}
		break;
	case SPEED_REGULATOR_ENABLE:
		if (commandCheck( strlen(command) >= 3 )) {
			globalSpeedRegulatorOn = (command[2] != '0');
			if (globalLogEvents) safePrint(36, "Speed regulator state changed to %d\n", globalSpeedRegulatorOn);
		}
		break;
	case MOTORS_CHARACTERISTIC:
		if (commandCheck( strlen(command) >= 7 && (command[2] == 'l' || command[2] == 'r') )) {
			taskENTER_CRITICAL();
			{
				globalSpeedRegulatorOn = DISABLE;
				globalLogSpeed = ENABLE;
				temp_float = strtof((char*)&command[4], &last) / 100.0f;
				globalLogSpeedCounter = strtol(last+1, NULL, 10);
				command[2] == 'l' ? setMotorLSpeed(temp_float) : setMotorRSpeed(temp_float);
			}
			taskEXIT_CRITICAL();
		}
		break;
	case MOTORS_CHARACTERISTIC2:
		if (commandCheck( strlen(command) >= 7 )) {
			taskENTER_CRITICAL();
			{
				globalSpeedRegulatorOn = DISABLE;
				globalLogSpeed = ENABLE;
				setMotorLSpeed(strtof((char*)&command[4], &last) / 100.0f);
				setMotorRSpeed(strtof(last+1, &last) / 100.0f);
				globalLogSpeedCounter = strtol(last+1, NULL, 10);
			}
			taskEXIT_CRITICAL();
		}
		break;
	case DELAY_COMMANDS:
		if (commandCheck( strlen(command) >= 3) ) {
			if (globalLogEvents) safePrint(9, "Pausing\n");
			vTaskDelay(strtol((char*)&command[2], NULL, 10) / portTICK_RATE_MS);
			if (globalLogEvents) safePrint(14, "Done waiting\n");
		}
		break;
	case SET_POSITION_SCALE:
		if (commandCheck( strlen(command) >= 3) ) {
			globalPositionScale = strtof((char*)&command[2], NULL);
			if (globalLogEvents) safePrint(31, "Position scale set to %.3f\n", globalPositionScale);
		}
		break;
#ifdef DRIVE_COMMANDS
	case WAIT_FOR_DRIVE_COMPLETE:
		if (globalLogEvents) safePrint(31, "Waiting for driving to finish\n");
		waitForDrivingEnd();
		break;
#endif
	case ODOMETRY_CORRECTION:
		if (commandCheck( strlen(command) >= 3 )) {
			temp_float = strtof((char*)&command[2], NULL);
			if (temp_float > 0.0f) {
				globalOdometryCorrectionGain = temp_float;
				if (globalLogEvents) safePrint(29, "Correction set to %.6f\n", globalOdometryCorrectionGain);
			}
		}
		break;
	case LOGGING_COMMAND:
		if (commandCheck( strlen(command) >= 5 )) {
			switch(command[2]) {
			case Logging_Type_Telemetry:
				globalLogTelemetry = (command[4] == '1');
				if (globalLogEvents) safePrint(28, "Logging telemetry set to %d\n", globalLogTelemetry);
				break;
			case Logging_Type_Speed:
				if (command[4] == '1') {
					taskENTER_CRITICAL();
					{
						globalLogSpeed = ENABLE;
						globalLogSpeedCounter = 1<<31;
					}
					taskEXIT_CRITICAL();
				}
				else {
					globalLogSpeed = DISABLE;
				}
				if (globalLogEvents) safePrint(24, "Logging speed set to %d\n", globalLogSpeed);
				break;
			case Logging_Type_Events:
				globalLogEvents = (command[4] == '1');
				if (globalLogEvents) safePrint(25, "Logging events set to %d\n", globalLogEvents);
				break;
			case Logging_Type_IMU:
				globalLogIMU = (command[4] == '1');
				if (globalLogEvents) safePrint(23, "Logging IMU set to %d\n", globalLogIMU);
				break;
			default:
				safePrint(17, wrongComm);
			}
		}
		break;
#ifndef USE_CUSTOM_MOTOR_CONTROLLER
	case SPEED_REGULATOR_PID_PARAMS:
		if(commandCheck (strlen(command) >= 7) ) {
			taskENTER_CRITICAL();
			{
				globalMotorPidKp = strtof((char*)&command[2], &last);
				globalMotorPidKi = strtof(last+1, &last);
				globalMotorPidKd = strtof(last+1, NULL);
			}
			taskEXIT_CRITICAL();
			if (globalLogEvents) safePrint(26, "Regulator params changed\n");
		}
		break;
#endif
#ifdef FOLLOW_TRAJECTORY
	case TRAJECTORY_REGULATOR_PARAMS:
		if(commandCheck (strlen(command) >= 7) ) {
			taskENTER_CRITICAL();
			{
				globalTrajectoryControlGains.k_x = strtof((char*)&command[2], &last);
				globalTrajectoryControlGains.k = strtof(last+1, &last);
				globalTrajectoryControlGains.k_s = strtof(last+1, NULL);
			}
			taskEXIT_CRITICAL();
			if (globalLogEvents) safePrint(26, "Regulator params changed\n");
		}
		break;
#endif
#ifdef DRIVE_COMMANDS
	case DRIVE_COMMAND:
		if (commandCheck( strlen(command) >= 11) ) {
			dc = (DriveCommand_Struct*)pvPortMalloc(sizeof(DriveCommand_Struct));
			dc->Type = command[2];
			dc->UsePen = (command[4] == '0' ? 0 : 1);
			dc->Speed = strtof((char*)&command[6], &last);
			dc->Param1 = strtof(last+1, &last);
			/* Only 'line' has one parameter */
			if (dc->Type != DriveCommand_Type_Line) {
				dc->Param2 = strtof(last+1, NULL);
			}
			xQueueSendToBack(driveQueue, &dc, portMAX_DELAY);
		}
		break;
#endif
	default:
		if (globalLogEvents) safePrint(18, "No such command\n");
		break;
	}
}

void TaskCommandHandlerConstructor() {
	xTaskCreate(TaskCommandHandler, NULL, TASKCOMMANDHANDLER_STACKSPACE, NULL, PRIOTITY_TASK_COMMANDHANDLER, &commandHandlerTask);
	commandQueue = xQueueCreate(50, sizeof(char*));
}
