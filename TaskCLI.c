#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS_CLI.h"

#include "priorities.h"
#include "stackSpace.h"
#include "compilation.h"
#include "hwinterface.h"
#include "main.h"
#include "pointsBuffer.h"

#include "TaskCLI.h"
#include "TaskPrintfConsumer.h"
#include "TaskMotorCtrl.h"
#include "TaskPenCtrl.h"
#include "TaskTelemetry.h"

#ifdef FOLLOW_TRAJECTORY
#include "TaskTrajectory.h"
#endif
#ifdef DRIVE_COMMANDS
#include "TaskDrive.h"
#endif

xTaskHandle CLITask;
xQueueHandle CLIInputQueue;

static const char* const welcomeMessage = "FreeRTOS command server.\r\nType \"help\" to view a list of registered commands.\n";
static const char* const promptMessage = "\nodin>";
static const char* const incorrectMessage = "Incorrect command parameter(s).  Enter \"help\" to view a list of available commands.\n";

/**
 * Checks if given command matches
 * @param command Full command name
 * @param imput Command to check
 * @param Number of characters that must be present in command to be recognized
 * @result 0 if Command matches, 1 otherwise
 */
static size_t cmatch(const char *command, const char *input, const size_t shortest);
/**
 * Slices command into small pieces separated by '\0'. Every piece is saved in params table
 * @param command Command to be sliced. '\0' Will be written after each separate word
 * @param params Array of n pointers to the slices created
 * @param n Maximum number of parameters to retrieve
 * @retval Number of characters really retrieved
 */
static size_t sliceCommand(char *command, char **params, const size_t n);
static void registerAllCommands();

void TaskCLI(void *p) {
	portBASE_TYPE moreDataComing;
	char * msg;
	char * outputString = (char*)FreeRTOS_CLIGetOutputBuffer();

	registerAllCommands();

	vTaskDelay(500/portTICK_RATE_MS);
	while (xQueueReceive(CLIInputQueue, &msg, 0) == pdTRUE);

	safePrint(strlen(welcomeMessage)+1, "%s", welcomeMessage);

    while(1) {
    	safePrint(strlen(promptMessage)+1, "%s", promptMessage);

		/* Block till message is available */
		xQueueReceive(CLIInputQueue, &msg, portMAX_DELAY);

		if (strlen(msg) == 0)
			continue;

		/* Process command and print as many lines as necessary */
		do {
			moreDataComing = FreeRTOS_CLIProcessCommand((int8_t*)msg, (int8_t*)outputString, configCOMMAND_INT_MAX_OUTPUT_SIZE);
			safePrint(strlen(outputString)+1, "%s", outputString);
		} while(moreDataComing != pdFALSE);

		/* Free allocated resources */
		vPortFree(msg);
    }
}

/////////////////////////////// COMMAND HANDLERS ///////////////////////////////////////

static portBASE_TYPE systemCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command);
static portBASE_TYPE lanternCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command);
static portBASE_TYPE delayCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command);
static portBASE_TYPE penCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command);
static portBASE_TYPE telemetryCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command);
static portBASE_TYPE motorCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command);
static portBASE_TYPE wifiCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command);
static portBASE_TYPE logCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command);
#ifdef FOLLOW_TRAJECTORY
static portBASE_TYPE trajectoryCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command);
#endif
#ifdef DRIVE_COMMANDS
static portBASE_TYPE driveCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command);
#endif

static const CLI_Command_Definition_t systemComDef =
{
    (int8_t*)"system",
    (int8_t*)"system <reset|battery|cpu|stack|memory|aua>\n",
    systemCommand,
    1
};
static const CLI_Command_Definition_t lanternComDef =
{
    (int8_t*)"lantern",
    (int8_t*)"lantern <enable|disable>\n",
    lanternCommand,
    1
};
static const CLI_Command_Definition_t delayComDef =
{
    (int8_t*)"delay",
    (int8_t*)"delay #milliseconds\n",
    delayCommand,
    1
};
static const CLI_Command_Definition_t penComDef =
{
    (int8_t*)"pen",
    (int8_t*)"pen <up|down|line [solid|dotted|dashed|ldashed|dotdashed]>\n",
    penCommand,
    -1
};
static const CLI_Command_Definition_t telemetryComDef =
{
    (int8_t*)"telemetry",
    (int8_t*)"telemetry [raw|<scaled [raw]>]\n"
    		 "\todometry correction [#param]\n"
    		 "\timu <enable|disable>\n",
    telemetryCommand,
    -1
};
static const CLI_Command_Definition_t motorComDef =
{
    (int8_t*)"motor",
    (int8_t*)"motor ...\n"
    		 "\tspeed <<left #val>|<right #val>|<#valL #valR>>\n"
    		 "\tpwm <<left #val>|<right #val>|<#valL #valR)>>\n"
#ifndef USE_CUSTOM_MOTOR_CONTROLLER
    		 "\tregulator [enable|disable|<params [#P #I #D]>]\n"
#endif
    		 "\tencoder [left|right]\n"
    		 "\tenable|disable\n"
    		 "\tbrake\n",
    motorCommand,
    -1
};
static const CLI_Command_Definition_t wifiComDef =
{
    (int8_t*)"wifi",
    (int8_t*)"wifi <reset|<set <command|data>>>\n",
    wifiCommand,
    -1
};
static const CLI_Command_Definition_t logComDef =
{
    (int8_t*)"log",
    (int8_t*)"log\n"
    		 "\toff\n"
    		 "\tall\n"
    		 "\t <events|telemetry|speed|imu> [off]\n"
    		 "\t motor something here\n",
    logCommand,
    -1
};
#ifdef FOLLOW_TRAJECTORY
static const CLI_Command_Definition_t trajectoryComDef =
{
    (int8_t*)"trajectory",
    (int8_t*)"trajectory ...\n"
    		 "\tcontroller params [iles paramsow]\n"
    		 "\tregulator params [iles paramsow]\n"
    		 "\timport <(Npoints)>\n",
    trajectoryCommand,
    -1
};
#endif
#ifdef DRIVE_COMMANDS
static const CLI_Command_Definition_t driveComDef =
{
    (int8_t*)"drive",
    (int8_t*)"drive ...\n"
    		 "\tscale [#value]\n"
    		 "\twait finish\n"
    		 "\tline dist #mm speed #speed [pen]\n"
    		 "\tturn <relative|absolute> angle #deg speed #speed [pen]\n"
    		 "\tarc angle #deg radius #mm speed #speed [pen]\n"
    		 "\tpoint x #x y #y speed #speed [pen]\n",
    driveCommand,
    -1
};
#endif

portBASE_TYPE systemCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command) {
	char *param;
	portBASE_TYPE paramLen;

	param = (char*)FreeRTOS_CLIGetParameter(command, 1, &paramLen);
	param[paramLen] = '\0';

	if (cmatch("aua", param, 1)) { // a
		strncpy((char*)outBuffer, "I am alive!", outBufferLen);
	}
	else if (cmatch("memory", param, 1)) { // m
		snprintf((char*)outBuffer, outBufferLen, "Available memory: %dkB\n", xPortGetFreeHeapSize());
	}
	else if (cmatch("reset", param, 1)) { // r
		IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
		IWDG_SetReload(1);
		while(1);
	}
	else if (cmatch("battery", param, 1)) { // b
		snprintf((char*)outBuffer, outBufferLen, "Battery voltage: %.2fV\n", getBatteryVoltage());
	}
	else if (cmatch("cpu", param, 1)) { // c
		snprintf((char*)outBuffer, outBufferLen, "CPU Usage: %.1f%%\n", globalCPUUsage*100.0f);
	}
	else if (cmatch("stack", param, 1)) { // s
		reportStackUsage();
		strncpy((char*)outBuffer, "\n", outBufferLen);
	}
	else {
		strncpy((char*)outBuffer, incorrectMessage, outBufferLen);
	}

	return pdFALSE;
}

portBASE_TYPE lanternCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command) {
	char* param;
	portBASE_TYPE paramLen;

	param = (char*)FreeRTOS_CLIGetParameter(command, 1, &paramLen);
	param[paramLen] = '\0';

	if (cmatch("enable", param, 1)) { // e
		enableLantern(ENABLE);
		strncpy((char*)outBuffer, "Lantern enabled\n", outBufferLen);
	}
	else if (cmatch("disable", param, 1)) { // d
		enableLantern(DISABLE);
		strncpy((char*)outBuffer, "Lantern disabled\n", outBufferLen);
	}
	else {
		strncpy((char*)outBuffer, incorrectMessage, outBufferLen);
	}

	return pdFALSE;
}

portBASE_TYPE delayCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command) {
	char* param;
	portBASE_TYPE paramLen;

	param = (char*)FreeRTOS_CLIGetParameter(command, 1, &paramLen);
	param[paramLen] = '\0';

	long del = strtol(param, NULL, 10);
	vTaskDelay(del / portTICK_RATE_MS);

	snprintf((char*)outBuffer, outBufferLen, "Delayed %ld milliseconds\n", del);
	return pdFALSE;
}

portBASE_TYPE penCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command) {
	char *param[3];
	bool ok = false;

	size_t nOfParams = sliceCommand((char*)command, param, 3);

	if (nOfParams > 0) {
		if (cmatch("up", param[0], 1)) { // u
			if (nOfParams == 1) {
				bool usePen = false;
				xQueueSendToBack(penCommandQueue, &usePen, portMAX_DELAY);
				strncpy((char*)outBuffer, "Pen is up\n", outBufferLen);
				ok = true;
			}
		}
		else if (cmatch("down", param[0], 1)) { // d
			if (nOfParams == 1) {
				bool usePen = true;
				xQueueSendToBack(penCommandQueue, &usePen, portMAX_DELAY);
				strncpy((char*)outBuffer, "Pen is down\n", outBufferLen);
				ok = true;
			}
		}
		else if (cmatch("line", param[0], 1)) { // l
			if (nOfParams == 2) {
				if (cmatch("solid", param[1], 1)) { // s
					setPenLineType(PenLine_Continuous);
					strncpy((char*)outBuffer, "Continuous line set\n", outBufferLen);
					ok = true;
				}
				else if (cmatch("dotted", param[1], 4)) { // dott
					setPenLineType(PenLine_Dotted);
					strncpy((char*)outBuffer, "Dotted line set\n", outBufferLen);
					ok = true;
				}
				else if (cmatch("dashed", param[1], 2)) { // da
					setPenLineType(PenLine_DashedShort);
					strncpy((char*)outBuffer, "Dashed line set\n", outBufferLen);
					ok = true;
				}
				else if (cmatch("ldashed", param[1], 1)) { // l
					setPenLineType(PenLine_DashedLong);
					strncpy((char*)outBuffer, "Long dashed line set\n", outBufferLen);
					ok = true;
				}
				else if (cmatch("dotdashed", param[1], 4)) { // dotd
					setPenLineType(PenLine_DotDash);
					strncpy((char*)outBuffer, "Dot-dash line set\n", outBufferLen);
					ok = true;
				}
			}
			else { // p2 == NULL
				strncpy((char*)outBuffer, "Not implemented\n", outBufferLen);
				ok = true;
			}
		}
	}

	if (!ok) {
		strncpy((char*)outBuffer, incorrectMessage, outBufferLen);
	}

	return pdFALSE;
}

portBASE_TYPE telemetryCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command) {
	char *param[4];
	bool ok = false;

	size_t nOfParams = sliceCommand((char*)command, param, 4);

	if (nOfParams > 0) {
		if (cmatch("odometry", param[0], 1)) { // o
			if (nOfParams > 1) {
				if (cmatch("correction", param[1], 1)) { // c
					if (nOfParams == 3) {
						globalOdometryCorrectionGain = strtof(param[2], NULL);
						snprintf((char*)outBuffer, outBufferLen, "Odometry correction param: %.6f\n", globalOdometryCorrectionGain);
						ok = true;
					}
					else {
						snprintf((char*)outBuffer, outBufferLen, "Odometry correction param: %.6f\n", globalOdometryCorrectionGain);
						ok = true;
					}
				}
			}
		}
		else if (cmatch("imu", param[0], 1)) { // i
			if (nOfParams == 2) {
				if (cmatch("enable", param[1], 1)) { // e
					globalUseIMUUpdates = true;
					strncpy((char*)outBuffer, "IMU enabled\n", outBufferLen);
					ok = true;
				}
				else if (cmatch("disable", param[1], 1)) { // d
					globalUseIMUUpdates = false;
					strncpy((char*)outBuffer, "IMU disabled\n", outBufferLen);
					ok = true;
				}
			}
		}
		else if (cmatch("raw", param[0], 1)) { // r
			if (nOfParams == 1) {
				TelemetryData_Struct tl;
				getTelemetryRaw(&tl);
				snprintf((char*)outBuffer, outBufferLen, "X: %.2f\nY: %.2f\nO: %.2f\n", tl.X, tl.Y, tl.O);
				ok = true;
			}
		}
		else if (cmatch("scaled", param[0], 1)) { // s
			if (nOfParams == 2) {
				if (cmatch("raw", param[1], 1)) { // r
					TelemetryData_Struct tl;
					getTelemetryRawScaled(&tl);
					snprintf((char*)outBuffer, outBufferLen, "X: %.2f\nY: %.2f\nO: %.2f\n", tl.X, tl.Y, tl.O);
					ok = true;
				}
			}
			else {
				TelemetryData_Struct tl;
				getTelemetryScaled(&tl);
				snprintf((char*)outBuffer, outBufferLen, "X: %.2f\nY: %.2f\nO: %.2f\n", tl.X, tl.Y, tl.O);
				ok = true;
			}
		}
	}
	else {
		TelemetryData_Struct tl;
		getTelemetry(&tl);
		snprintf((char*)outBuffer, outBufferLen, "X: %.2f\nY: %.2f\nO: %.2f\n", tl.X, tl.Y, tl.O);
		ok = true;
	}

	if (!ok) {
		strncpy((char*)outBuffer, incorrectMessage, outBufferLen);
	}
	return pdFALSE;
}

portBASE_TYPE motorCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command) {
	char *p1, *p2, *p3, *p4, *p5, *p6;
	portBASE_TYPE p1Len, p2Len, p3Len, p4Len, p5Len, p6Len;
	bool ok = false;

	p1 = (char*)FreeRTOS_CLIGetParameter(command, 1, &p1Len);
	p2 = (char*)FreeRTOS_CLIGetParameter(command, 2, &p2Len);
	p3 = (char*)FreeRTOS_CLIGetParameter(command, 3, &p3Len);
	p4 = (char*)FreeRTOS_CLIGetParameter(command, 4, &p4Len);
	p5 = (char*)FreeRTOS_CLIGetParameter(command, 5, &p5Len);
	p6 = (char*)FreeRTOS_CLIGetParameter(command, 6, &p6Len);

	if (p1 != NULL) {
		p1[p1Len] = '\0';
		if (cmatch("speed", p1, 1)) { // s
			if (p2 != NULL && p3 != NULL && p4 == NULL) {
				p2[p2Len] = '\0';
				p3[p3Len] = '\0';
				float left = 0.0f, right = 0.0f;
				if (cmatch("left", p2, 1)) { // l
					left = strtof(p3, NULL);
				}
				else if (cmatch("right", p2, 1)) { // r
					right = strtof(p3, NULL);
				}
				else {
					left = strtof(p2, NULL);
					right = strtof(p3, NULL);
				}
				snprintf((char*)outBuffer, outBufferLen, "Speeds set to %.2f left and %.2f right\n", left, right);
				sendSpeeds(left, right, 0);
				ok = true;
			}
		}
		else if (cmatch("pwm", p1, 1)) { // p
			if (p2 != NULL && p3 != NULL && p4 == NULL) {
				p2[p2Len] = '\0';
				p3[p3Len] = '\0';
				if (cmatch("left", p2, 1)) { // l
					float left = strtof(p3, NULL);
					setMotorLSpeed(left/100.0f);
					snprintf((char*)outBuffer, outBufferLen, "Left PWM set to %.1f%%\n", left);
					ok = true;
				}
				else if (cmatch("right", p2, 1)) { // r
					float right = strtof(p3, NULL);
					setMotorRSpeed(right/100.0f);
					snprintf((char*)outBuffer, outBufferLen, "Right PWM set to %.1f%%\n", right);
					ok = true;
				}
				else {
					float left = strtof(p2, NULL);
					float right = strtof(p3, NULL);
					setMotorLSpeed(left/100.0f);
					setMotorRSpeed(right/100.0f);
					snprintf((char*)outBuffer, outBufferLen, "Left PWM set to %.1f%%\nRight PWM set to %.1f%%\n", left, right);
					ok = true;
				}
			}
		}
		else if (cmatch("regulator", p1, 1)) { // r
			if (p2 != NULL) {
				p2[p2Len] = '\0';
				if (cmatch("enable", p2, 1)) { // e
					if (p3 == NULL) {
						globalSpeedRegulatorOn = ENABLE;
						strncpy((char*)outBuffer, "Regulator enabled\n", outBufferLen);
						ok = true;
					}
				}
				else if (cmatch("disable", p2, 1)) { // d
					if (p3 == NULL) {
						globalSpeedRegulatorOn = DISABLE;
						strncpy((char*)outBuffer, "Regulator disabled\n", outBufferLen);
						ok = true;
					}
				}
				else if (strcmp(p2, "params")) {
#ifndef USE_CUSTOM_MOTOR_CONTROLLER
					if (p3 != NULL && p4 != NULL && p5 != NULL && p6 == NULL) {
						p3[p3Len] = '\0';
						p4[p4Len] = '\0';
						p5[p5Len] = '\0';
						taskENTER_CRITICAL();
						{
							globalMotorPidKp = strtof(p2, NULL);
							globalMotorPidKi = strtof(p3, NULL);
							globalMotorPidKd = strtof(p4, NULL);
						}
						taskEXIT_CRITICAL();
						strncpy((char*)outBuffer, "New params set\n", outBufferLen);
						ok = true;
					}
					else if (p3 == NULL) {
						snprintf((char*)outBuffer, outBufferLen, "P %.3f\nI %.3f\nD %.3f\n", globalMotorPidKp, globalMotorPidKi, globalMotorPidKd);
						ok = true;
					}
#endif
				}
			}
			else {
				if (globalSpeedRegulatorOn == ENABLE) {
					strncpy((char*)outBuffer, "Regulator enabled\n", outBufferLen);
				}
				else {
					strncpy((char*)outBuffer, "Regulator disabled\n", outBufferLen);
				}
				ok = true;
			}
		}
		else if (cmatch("encoder", p1, 3)) { // enc
			if (p2 != NULL) {
				p2[p2Len] = '\0';
				if (cmatch("left", p2, 1)) { // l
					if (p3 == NULL) {
						snprintf((char*)outBuffer, outBufferLen, "Encoder left: %ld\n", getEncoderL());
						ok = true;
					}
				}
				else if (cmatch("right", p2, 1)) { // r
					if (p3 == NULL) {
						snprintf((char*)outBuffer, outBufferLen, "Encoder right: %ld\n", getEncoderR());
						ok = true;
					}
				}
			}
			else { // p2 == NULL
				snprintf((char*)outBuffer, outBufferLen, "Encoder left: %ld\nEncoder right: %ld\n", getEncoderL(), getEncoderR());
				ok = true;
			}
		}
		else if (cmatch("disable", p1, 1)) { // d
			if (p2 == NULL) {
				globalSpeedRegulatorOn = DISABLE;
				enableMotors(DISABLE);
				strncpy((char*)outBuffer, "Motors disabled\n", outBufferLen);
				ok = true;
			}
		}
		else if (cmatch("enable", p1, 3)) { // ena
			if (p2 == NULL) {
				globalSpeedRegulatorOn = ENABLE;
				enableMotors(ENABLE);
				strncpy((char*)outBuffer, "Motors enabled\n", outBufferLen);
				ok = true;
			}
		}
		else if (cmatch("brake", p1, 1)) { // b
			if (p2 == NULL) {
				sendSpeeds(0.0f, 0.0f, 0);
				setMotorLBrake();
				setMotorRBrake();
				strncpy((char*)outBuffer, "Motors braking\n", outBufferLen);
				ok = true;
			}
		}
	}

	if (!ok) {
		strncpy((char*)outBuffer, incorrectMessage, outBufferLen);
	}
	return pdFALSE;
}

portBASE_TYPE wifiCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command) {
	char *p1, *p2, *p3;
	portBASE_TYPE p1Len, p2Len, p3Len;
	bool ok = false;

	p1 = (char*)FreeRTOS_CLIGetParameter(command, 1, &p1Len);
	p2 = (char*)FreeRTOS_CLIGetParameter(command, 2, &p2Len);
	p3 = (char*)FreeRTOS_CLIGetParameter(command, 3, &p3Len);

	if (p1 != NULL) {
		p1[p1Len] = '\0';
		if (cmatch("set", p1, 1)) { // s
			if (false) {} else // set command is dangerous because there is loop in printing commands and responses
			if (p2 != NULL && p3 == NULL) {
				p2[p2Len] = '\0';
				if (cmatch("command", p2, 1)) { // c
					setWiFiMode(WiFiMode_Command);
					strncpy((char*)outBuffer, "Command mode set\nCommunication terminated\n", outBufferLen);
					ok = true;
				}
				else if (cmatch("data", p2, 1)) { // d
					setWiFiMode(WiFiMode_Data);
					strncpy((char*)outBuffer, "Data mode set\n", outBufferLen);
					ok = true;
				}
			}
		}
		else if (cmatch("reset", p1, 1)) { // r
			if (p2 == NULL) {
				setWiFiReset(ENABLE);
				vTaskDelay(200/portTICK_RATE_MS);
				setWiFiReset(DISABLE);
				strncpy((char*)outBuffer, "WiFi reset\n", outBufferLen);
				ok = true;
			}
		}
	}

	if (!ok) {
		strncpy((char*)outBuffer, incorrectMessage, outBufferLen);
	}

	return pdFALSE;
}

portBASE_TYPE logCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command) {
	char *p1, *p2, *p3;
	portBASE_TYPE p1Len, p2Len, p3Len;
	bool ok = false;

	p1 = (char*)FreeRTOS_CLIGetParameter(command, 1, &p1Len);
	p2 = (char*)FreeRTOS_CLIGetParameter(command, 2, &p2Len);
	p3 = (char*)FreeRTOS_CLIGetParameter(command, 3, &p3Len);

	if (p1 != NULL) {
		p1[p1Len] = '\0';
		if (cmatch("all", p1, 1)) { // a
			if (p2 == NULL) {
				globalLogEvents = globalLogTelemetry = globalLogSpeed = globalLogIMU = ENABLE;
				strncpy((char*)outBuffer, "Logging all\n", outBufferLen);
				ok = true;
			}
		}
		else if (cmatch("off", p1, 1)) { // o
			if (p2 == NULL) {
				globalLogEvents = globalLogTelemetry = globalLogSpeed = globalLogIMU = DISABLE;
				strncpy((char*)outBuffer, "Logging off\n", outBufferLen);
				ok = true;
			}
		}
		else if (cmatch("motor", p1, 1)) { // m
			strncpy((char*)outBuffer, "Not implemented\n", outBufferLen);
			ok = true;
		}
		else { // events, speed, telemetry, imu
			FunctionalState state = ENABLE;
			bool error = false;
			volatile FunctionalState *log;
			if (p2 != NULL) {
				p2[p2Len] = '\0';
				if (cmatch("off", p2, 1) && p3 == NULL) { // must be equal 'off' and nothing follows
					state = DISABLE;
				}
				else {
					error = true;
				}
			}
			if (cmatch("events", p1, 1)) { // e
				log = &globalLogEvents;
			}
			else if (cmatch("speed", p1, 1)) { // s
				log = &globalLogSpeed;
			}
			else if (cmatch("telemetry", p1, 1)) { // t
				log = &globalLogTelemetry;
			}
			else if (cmatch("imu", p1, 1)) { // i
				log = &globalLogIMU;
			}
			else {
				error = true;
			}
			if (!error) {
				*log = state;
				const char* c = (state == ENABLE ? "enabled" : "disabled");
				snprintf((char*)outBuffer, outBufferLen, "Logging %s %s\n", p1, c);
				ok = true;
			}
		}
	}
	else {
		snprintf((char*)outBuffer, outBufferLen, "Logging settings:\n"
				"\tEvents: %d\n\tTelemetry: %d\n"
				"\tSpeed: %d\n\tIMU: %d\n",
				globalLogEvents == ENABLE, globalLogTelemetry == ENABLE, globalLogSpeed == ENABLE, globalLogIMU = ENABLE);
		ok = true;
	}

	if (!ok) {
		strncpy((char*)outBuffer, incorrectMessage, outBufferLen);
	}

	return pdFALSE;
}

#ifdef FOLLOW_TRAJECTORY
portBASE_TYPE trajectoryCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command) {
	char *p1, *p2, *p3;
	portBASE_TYPE p1Len, p2Len, p3Len;
	bool ok = false;

	p1 = (char*)FreeRTOS_CLIGetParameter(command, 1, &p1Len);
	p2 = (char*)FreeRTOS_CLIGetParameter(command, 2, &p2Len);
	p3 = (char*)FreeRTOS_CLIGetParameter(command, 3, &p3Len);

	if (p1 != NULL) {
		p1[p1Len] = '\0';
		if (cmatch("import", p1, 1)) { // i
			if (p2 != NULL && p3 == NULL) {
				p2[p2Len] = '\0';
				snprintf((char*)outBuffer, outBufferLen, "<#Please send only %d points#>\n", TBloadNewPoints(strtol(p2, NULL, 10)));
				ok = true;
			}
		}
		else if (cmatch("regulator", p1, 1)) { // r
			if (p2 != NULL) {
				p2[p2Len] = '\0';
				if (cmatch("params", p2, 1)) { // p
					strncpy((char*)outBuffer, "Not implemented\n", outBufferLen);
					ok = true;
				}
			}
		}
		else if (cmatch("controller", p1, 1)) { // c
			if (p2 != NULL) {
				p2[p2Len] = '\0';
				if (cmatch("params", p1, 1)) { // p
					strncpy((char*)outBuffer, "Not implemented\n", outBufferLen);\
					ok = true;
				}
			}
		}
	}

	if (!ok) {
		strncpy((char*)outBuffer, incorrectMessage, outBufferLen);
	}
	return pdFALSE;
}
#endif

#ifdef DRIVE_COMMANDS
portBASE_TYPE driveCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command) {
	char *p1, *p2, *p3, *p4, *p5, *p6, *p7, *p8, *p9;
	portBASE_TYPE p1Len, p2Len, p3Len, p4Len, p5Len, p6Len, p7Len, p8Len, p9Len;
	bool ok = false;

	p1 = (char*)FreeRTOS_CLIGetParameter(command, 1, &p1Len);
	p2 = (char*)FreeRTOS_CLIGetParameter(command, 2, &p2Len);
	p3 = (char*)FreeRTOS_CLIGetParameter(command, 3, &p3Len);
	p4 = (char*)FreeRTOS_CLIGetParameter(command, 4, &p4Len);
	p5 = (char*)FreeRTOS_CLIGetParameter(command, 5, &p5Len);
	p6 = (char*)FreeRTOS_CLIGetParameter(command, 6, &p6Len);
	p7 = (char*)FreeRTOS_CLIGetParameter(command, 7, &p7Len);
	p8 = (char*)FreeRTOS_CLIGetParameter(command, 8, &p8Len);
	p9 = (char*)FreeRTOS_CLIGetParameter(command, 9, &p9Len);

	if (p1 != NULL) {
		p1[p1Len] = '\0';
		if (cmatch("scale", p1, 1)) { // s
			if (p2 != NULL) {
				if (p3 == NULL) {
					p2[p2Len] = '\0';
					float s = strtof(p2, NULL);
					if (s > 0.0f) {
						globalPositionScale = s;
						snprintf((char*)outBuffer, outBufferLen, "Scale set to %.4f\n", globalPositionScale);
						ok = true;
					}
				}
			}
			else { // p2 == NULL
				snprintf((char*)outBuffer, outBufferLen, "Scale: %.4f\n", globalPositionScale);
				ok = true;
			}
		}
		else if (cmatch("wait", p1, 1)) { // w
			if (p2 != NULL && p3 == NULL) {
				p2[p2Len] = '\0';
				if (cmatch("finish", p2, 1)) { // f
					while (isCurrentlyDriving()) {
						vTaskDelay(10/portTICK_RATE_MS);
					}
					strncpy((char*)outBuffer, "Driving finished\n", outBufferLen);
					ok = true;
				}
			}
		}
		else if (cmatch("line", p1, 1)) { // l // p2 == 'dist', p3 == mm, p4 == 'speed', p5 == 'sp', p6 optional 'pen'
			bool pen = false;
			bool error = false;
			if (p6 != NULL) {
				p6[p6Len] = '\0';
				if (cmatch("pen", p6, 1)) { // p
					pen = true;
				}
				else {
					error = true;
				}
			}
			if (p5 == NULL || p7 != NULL) {
				error = true;
			}
			if (!error) {
				p2[p2Len] = p3[p3Len] = p4[p4Len] = p5[p5Len] = '\0';
				if (!cmatch("dist", p2, 1) || !cmatch("speed", p4, 1)) {
					error = true;
				}
			}
			if (!error) {
				float dist = strtof(p3, NULL);
				float speed = strtof(p5, NULL);
				DriveCommand_Struct cmd = {
					.Type = DriveCommand_Type_Line,
					.UsePen = pen,
					.Speed = speed,
					.Param1 = dist
				};
				xQueueSendToBack(driveQueue, &cmd, portMAX_DELAY);
				strncpy((char*)outBuffer, "Drive command accepted\n", outBufferLen);
				ok = true;
			}
		}
		else if (cmatch("turn", p1, 1)) { // t // p2 == relative or absolute, p3 == 'angle', p4 == deg, p5 == 'speed', p6 == sp, p7 == optional pen
			bool pen = false;
			bool error = false;
			bool relative;
			if (p7 != NULL) {
				p7[p7Len] = '\0';
				if (cmatch("pen", p7, 1)) { // p
					pen = true;
				}
				else {
					error = true;
				}
			}
			if (p6 == NULL || p8 != NULL) {
				error = true;
			}
			if (!error) {
				p2[p2Len] = p3[p3Len] = p4[p4Len] = p5[p5Len] = p6[p6Len] = '\0';
				if (!cmatch("angle", p3, 1) || !cmatch("speed", p5, 1)) {
					error = true;
				}
				if (cmatch("relative", p2, 1)) { // r
					relative = true;
				}
				else if (cmatch("absolute", p2, 1)) { // a
					relative = false;
				}
				else {
					error = true;
				}
			}
			if (!error) {
				float ang = strtof(p4, NULL);
				float speed = strtof(p6, NULL);
				DriveCommand_Struct cmd = {
					.Type = DriveCommand_Type_Angle,
					.UsePen = pen,
					.Speed = speed,
					.Param1 = (relative ? 0.0f : 1.0f),
					.Param2 = ang
				};
				xQueueSendToBack(driveQueue, &cmd, portMAX_DELAY);
				strncpy((char*)outBuffer, "Drive command accepted\n", outBufferLen);
				ok = true;
			}
		}
		else if (cmatch("arc", p1, 1)) { // a  // p2 == 'angle', p3 == ang, p4 == 'radius', p5 == 'mm', p6 == 'speed', p7 == sp, p8 == optional 'pen'
			bool pen = false;
			bool error = false;
			if (p8 != NULL) {
				p8[p8Len] = '\0';
				if (cmatch("pen", p8, 1)) { // p
					pen = true;
				}
				else {
					error = true;
				}
			}
			if (p7 == NULL || p9 != NULL) {
				error = true;
			}
			if (!error) {
				p2[p2Len] = p3[p3Len] = p4[p4Len] = p5[p5Len] = p6[p6Len] = p7[p7Len] = '\0';
				if (!cmatch("angle", p2, 1) || !cmatch("radius", p4, 1) || !cmatch("speed", p6, 1)) {
					error = true;
				}
			}
			if (!error) {
				float angle = strtof(p3, NULL);
				float radius = strtof(p5, NULL);
				float speed = strtof(p7, NULL);
				DriveCommand_Struct cmd = {
					.Type = DriveCommand_Type_Arc,
					.UsePen = pen,
					.Speed = speed,
					.Param1 = radius,
					.Param2 = angle
				};
				xQueueSendToBack(driveQueue, &cmd, portMAX_DELAY);
				strncpy((char*)outBuffer, "Drive command accepted\n", outBufferLen);
				ok = true;
			}
		}
		else if (cmatch("point", p1, 1)) { // p // p2 == 'x', p3 = x, p4 = 'y', p5 = y, p6 == 'speed', p7 == sp, p8 optional 'pen'
			bool pen = false;
			bool error = false;
			if (p8 != NULL) {
				p8[p8Len] = '\0';
				if (cmatch("pen", p8, 1)) { // p
					pen = true;
				}
				else {
					error = true;
				}
			}
			if (p7 == NULL || p9 != NULL) {
				error = true;
			}
			if (!error) {
				p2[p2Len] = p3[p3Len] = p4[p4Len] = p5[p5Len] = p6[p6Len] = p7[p7Len] = '\0';
				if (strcmp(p2, "x") != 0 || strcmp(p4, "y") != 0 || !cmatch("speed", p6, 1) != 0) {
					error = true;
				}
			}
			if (!error) {
				float x = strtof(p3, NULL);
				float y = strtof(p5, NULL);
				float speed = strtof(p7, NULL);
				DriveCommand_Struct cmd = {
					.Type = DriveCommand_Type_Arc,
					.UsePen = pen,
					.Speed = speed,
					.Param1 = x,
					.Param2 = y
				};
				xQueueSendToBack(driveQueue, &cmd, portMAX_DELAY);
				strncpy((char*)outBuffer, "Drive command accepted\n", outBufferLen);
				ok = true;
			}
		}
	}

	if (!ok) {
		strncpy((char*)outBuffer, incorrectMessage, outBufferLen);
	}
	return pdFALSE;
}
#endif

///////////////////////////////////// END COMMAND HANDLERS ///////////////////////////////////////

void registerAllCommands() {
	FreeRTOS_CLIRegisterCommand(&systemComDef);
	FreeRTOS_CLIRegisterCommand(&lanternComDef);
	FreeRTOS_CLIRegisterCommand(&delayComDef);
	FreeRTOS_CLIRegisterCommand(&penComDef);
	FreeRTOS_CLIRegisterCommand(&telemetryComDef);
	FreeRTOS_CLIRegisterCommand(&motorComDef);
	FreeRTOS_CLIRegisterCommand(&wifiComDef);
	FreeRTOS_CLIRegisterCommand(&logComDef);
#ifdef FOLLOW_TRAJECTORY
	FreeRTOS_CLIRegisterCommand(&trajectoryComDef);
#endif
#ifdef DRIVE_COMMANDS
	FreeRTOS_CLIRegisterCommand(&driveComDef);
#endif
}

size_t cmatch(const char *command, const char *input, const size_t shortest) {
	if (strncmp(command, input, shortest) != 0) return 0; // beginning does not match
	command += shortest;
	input += shortest;
	while (*input && *command && *input == *command) {
		input++;
		command++;
	}
	if (*input == '\0') return 1;
	return 0;
}

static size_t sliceCommand(char *command, char **params, const size_t n) {
	size_t nFound = 0;

	/* Ignore first word - it is a command name itself */
	while (*command && *command != ' ')
		command++;

	while (nFound < n) {
		/* Ignore leading spaces */
		while (*command && *command == ' ')
			command++;

		/* If end of command found then return */
		if (*command == '\0')
			break;

		/* Save the beginning of the parameter on the list */
		params[nFound++] = command;

		/* Move to the end of the parameter */
		while (*command && *command != ' ')
			command++;

		/* Replace the first space with '\0' to indicate end of param */
		if (*command == ' ') {
			*command = '\0';
			command++;
		}
	}

	/* Make the rest of the params be NULL */
	for (size_t i = nFound; i<n; ++i)
		params[i] = NULL;

	return nFound;
}

void TaskCLIConstructor() {
	xTaskCreate(TaskCLI, NULL, TASKCLI_STACKSPACE, NULL, PRIORITY_TASK_CLI, &CLITask);
	CLIInputQueue = xQueueCreate(configCOMMAND_INT_MAX_OUTPUT_SIZE, sizeof(char*));
}
