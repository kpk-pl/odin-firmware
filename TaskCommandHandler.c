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
#ifndef FOLLOW_TRAJECTORY
	DriveCommand_Struct *dc;
#endif
	char *last;
	MotorSpeed_Struct ms;
	TelemetryData_Struct td;
	float temp_float;
#ifdef USE_CUSTOM_MOTOR_CONTROLLER
	MotorControllerParameters_Struct* ptrParams;
#endif

	const char wrongComm[] = "Incorrect command\n";

	bool commandCheck(const bool p) {
		if (p != true) {
			if (globalLogEvents) safePrint(20, "%s", wrongComm);
		}
		return p;
	}

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
		break;
	case PEN_UP:
		setPenUp();
		break;
#ifndef FOLLOW_TRAJECTORY
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
				safePrint(19, "Motors disabled\n");
			}
			else {
				enableMotors(ENABLE);
				globalSpeedRegulatorOn = ENABLE;
				safePrint(18, "Motors enabled\n");
			}
		}
		break;
	case MOTORS_SET_SPEEDS:
		if (commandCheck( strlen(command) >= 3 )) {
			globalSpeedRegulatorOn = ENABLE;
			ms.LeftSpeed = strtof((char*)&command[2], &last);
			ms.RightSpeed = strtof(last+1, NULL);
			xQueueSendToBack(motorCtrlQueue, &ms, portMAX_DELAY);
		}
		break;
	case TELEMETRY_PRINT:
		getTelemetry(&td);
		safePrint(40, "X:%.2f Y:%.2f O:%.1f\n", td.X, td.Y, td.O / DEGREES_TO_RAD);
		break;
	case LOGGING_COMMAND:
		if (commandCheck( strlen(command) >= 5 )) {
			switch(command[2]) {
			case Logging_Type_Telemetry:
				globalLogTelemetry = (command[4] == '1');
				break;
			case Logging_Type_Speed:
				if (command[4] == '1') {
					taskENTER_CRITICAL();
					globalLogSpeed = ENABLE;
					globalLogSpeedCounter = 1<<31;
					taskEXIT_CRITICAL();
				}
				else {
					globalLogSpeed = DISABLE;
				}
				break;
			case Logging_Type_Events:
				globalLogEvents = (command[4] == '1');
				break;
			default:
				safePrint(17, wrongComm);
			}
		}
		break;
	case CPU_RESET:
		IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
		IWDG_SetReload(1);
		break;
	case WIFI_MODE:
		if (commandCheck( strlen(command) >= 3 )) {
			setWiFiMode(command[2] == 'c' ? WiFiMode_Command : WiFiMode_Data);
		}
		break;
	case WIFI_RESET:
		if (commandCheck( strlen(command) >= 3 )) {
			setWiFiReset(command[2] == '1' ? ENABLE : DISABLE);
		}
		break;
	case LANTERN_ENABLE:
		if (commandCheck( strlen(command) >= 3 )) {
			enableLantern(command[2] == '1' ? ENABLE : DISABLE);
		}
		break;
	case SPEED_REGULATOR_ENABLE:
		if (commandCheck( strlen(command) >= 3 )) {
			taskENTER_CRITICAL();
			{
				globalSpeedRegulatorOn = (command[2] != '0');
				setMotorLSpeed(0.0f);
				setMotorRSpeed(0.0f);
			}
			taskEXIT_CRITICAL();
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
			vTaskDelay(strtol((char*)&command[2], NULL, 10) / portTICK_RATE_MS);
		}
		break;
	case CPU_USAGE:
		safePrint(19, "CPU Usage: %.1f%%\n", globalCPUUsage*100.0f);
		break;
	case STACK_USAGE:
		reportStackUsage();
		break;
	case '$':
		temp_float = strtof((char*)&command[2], NULL);
		safePrint(60, "Float %f: %x %x %x %x\n", temp_float, *((uint8_t*)(&temp_float)), *(((uint8_t*)(&temp_float)+1)), *(((uint8_t*)(&temp_float)+2)), *(((uint8_t*)(&temp_float))+3));
		break;
#ifdef USE_CUSTOM_MOTOR_CONTROLLER
	case SPEER_REGULATOR_VOLTAGE_CORRECTION:
		if (commandCheck( strlen(command) >= 3 )) {
			globalControllerVoltageCorrection = (command[2] != '0');
		}
		break;
	case SPEED_REGULATOR_CUSTOM_PARAMS:
		if (commandCheck (strlen(command) >= 19 && (command[2] == 'l' || command[2] == 'r') )) {
			if (command[2] == 'l') ptrParams = &globalLeftMotorParams;
			else ptrParams = &globalRightMotorParams;
			taskENTER_CRITICAL();
			{
				ptrParams->threshold = strtof((char*)&command[4], &last);
				ptrParams->A = strtof(last+1, &last);
				ptrParams->B = strtof(last+1, &last);
				ptrParams->C = strtof(last+1, &last);
				ptrParams->KP = strtof(last+1, &last);
				ptrParams->A_t = strtof(last+1, &last);
				ptrParams->B_t = strtof(last+1, &last);
				ptrParams->KP_t = strtof(last+1, &last);
			}
			taskEXIT_CRITICAL();
		}
		break;
#else
	case SPEED_REGULATOR_PID_PARAMS:
		if(commandCheck (strlen(command) >= 7) ) {
			taskENTER_CRITICAL();
			{
				globalPidLeft.Kp = globalPidRight.Kp = strtof((char*)&command[2], &last);
				globalPidLeft.Ki = globalPidRight.Ki = strtof(last+1, &last);
				globalPidLeft.Kd = globalPidRight.Kd = strtof(last+1, &last);
			}
			taskEXIT_CRITICAL();
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
	commandQueue = xQueueCreate(15, sizeof(char*));
}
