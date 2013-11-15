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

static void registerAllCommands();
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

portBASE_TYPE systemCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command) {
	char *param;
	portBASE_TYPE paramLen;

	param = (char*)FreeRTOS_CLIGetParameter(command, 1, &paramLen);
	param[paramLen] = '\0';

	if (strcmp(param, "aua") == 0) {
		strncpy((char*)outBuffer, "I am alive!", outBufferLen);
	}
	else if (strcmp(param, "memory") == 0) {
		snprintf((char*)outBuffer, outBufferLen, "Available memory: %dkB\n", xPortGetFreeHeapSize());
	}
	else if (strcmp(param, "reset") == 0) {
		IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
		IWDG_SetReload(1);
		while(1);
	}
	else if (strcmp(param, "battery") == 0) {
		snprintf((char*)outBuffer, outBufferLen, "Battery voltage: %.2fV\n", getBatteryVoltage());
	}
	else if (strcmp(param, "cpu") == 0) {
		snprintf((char*)outBuffer, outBufferLen, "CPU Usage: %.1f%%\n", globalCPUUsage*100.0f);
	}
	else if (strcmp(param, "stack") == 0) {
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

	if (strcmp(param, "enable") == 0) {
		enableLantern(ENABLE);
		strncpy((char*)outBuffer, "Lantern enabled\n", outBufferLen);
	}
	else if (strcmp(param, "disable") == 0) {
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
	char *p1, *p2, *p3;
	portBASE_TYPE p1Len, p2Len, p3Len;
	bool ok = false;

	p1 = (char*)FreeRTOS_CLIGetParameter(command, 1, &p1Len);
	p2 = (char*)FreeRTOS_CLIGetParameter(command, 2, &p2Len);
	p3 = (char*)FreeRTOS_CLIGetParameter(command, 3, &p3Len);

	if (p1 != NULL) {
		p1[p1Len] = '\0';
		if (strcmp(p1, "up") == 0) {
			if (p2 == NULL) {
				bool usePen = false;
				xQueueSendToBack(penCommandQueue, &usePen, portMAX_DELAY);
				strncpy((char*)outBuffer, "Pen is up\n", outBufferLen);
				ok = true;
			}
		}
		else if (strcmp(p1, "down") == 0) {
			if (p2 == NULL) {
				bool usePen = true;
				xQueueSendToBack(penCommandQueue, &usePen, portMAX_DELAY);
				strncpy((char*)outBuffer, "Pen is down\n", outBufferLen);
				ok = true;
			}
		}
		else if (strcmp(p1, "line") == 0) {
			if (p2 != NULL) {
				if (p3 == NULL) {
					p2[p2Len] = '\0';
					if (strcmp(p2, "solid") == 0) {
						setPenLineType(PenLine_Continuous);
						strncpy((char*)outBuffer, "Continuous line set\n", outBufferLen);
						ok = true;
					}
					else if (strcmp(p2, "dotted") == 0) {
						setPenLineType(PenLine_Dotted);
						strncpy((char*)outBuffer, "Dotted line set\n", outBufferLen);
						ok = true;
					}
					else if (strcmp(p2, "dashed") == 0) {
						setPenLineType(PenLine_DashedShort);
						strncpy((char*)outBuffer, "Dashed line set\n", outBufferLen);
						ok = true;
					}
					else if (strcmp(p2, "ldashed") == 0) {
						setPenLineType(PenLine_DashedLong);
						strncpy((char*)outBuffer, "Long dashed line set\n", outBufferLen);
						ok = true;
					}
					else if (strcmp(p2, "dotdashed") == 0) {
						setPenLineType(PenLine_DotDash);
						strncpy((char*)outBuffer, "Dot-dash line set\n", outBufferLen);
						ok = true;
					}
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
	char *p1, *p2, *p3, *p4;
	portBASE_TYPE p1Len, p2Len, p3Len, p4Len;
	bool ok = false;

	p1 = (char*)FreeRTOS_CLIGetParameter(command, 1, &p1Len);
	p2 = (char*)FreeRTOS_CLIGetParameter(command, 2, &p2Len);
	p3 = (char*)FreeRTOS_CLIGetParameter(command, 3, &p3Len);
	p4 = (char*)FreeRTOS_CLIGetParameter(command, 4, &p4Len);

	if (p1 != NULL) {
		p1[p1Len] = '\0';
		if (strcmp(p1, "odometry") == 0) {
			if (p2 != NULL) {
				p2[p2Len] = '\0';
				if (strcmp(p2, "correction") == 0) {
					if (p3 != NULL) {
						if (p4 == NULL) {
							p3[p3Len] = '\0';
							globalOdometryCorrectionGain = strtof(p3, NULL);
							snprintf((char*)outBuffer, outBufferLen, "Odometry correction param: %.6f\n", globalOdometryCorrectionGain);
							ok = true;
						}
					}
					else { // p3 == NULL
						snprintf((char*)outBuffer, outBufferLen, "Odometry correction param: %.6f\n", globalOdometryCorrectionGain);
						ok = true;
					}
				}
			}
		}
		else if (strcmp(p1, "imu") == 0) {
			if (p2 != NULL) {
				if (strcmp(p2, "enable") == 0) {
					if (p3 == NULL) {
						globalUseIMUUpdates = true;
						strncpy((char*)outBuffer, "IMU enabled\n", outBufferLen);
						ok = true;
					}
				}
				else if (strcmp(p2, "disable") == 0) {
					if (p3 == NULL) {
						globalUseIMUUpdates = false;
						strncpy((char*)outBuffer, "IMU disabled\n", outBufferLen);
						ok = true;
					}
				}
			}
		}
		else if (strcmp(p1, "raw") == 0) {
			if (p2 == NULL) {
				TelemetryData_Struct tl;
				getTelemetryRaw(&tl);
				snprintf((char*)outBuffer, outBufferLen, "X: %.2f\nY: %.2f\nO: %.2f\n", tl.X, tl.Y, tl.O);
				ok = true;
			}
		}
		else if (strcmp(p1, "scaled") == 0) {
			if (p2 != NULL) {
				if (p3 == NULL) {
					p2[p2Len] = '\0';
					if (strcmp(p2, "raw") == 0) {
						TelemetryData_Struct tl;
						getTelemetryRawScaled(&tl);
						snprintf((char*)outBuffer, outBufferLen, "X: %.2f\nY: %.2f\nO: %.2f\n", tl.X, tl.Y, tl.O);
						ok = true;
					}
				}
			}
			else { // p2 == NULL
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
		if (strcmp(p1, "speed") == 0) {
			if (p2 != NULL && p3 != NULL && p4 == NULL) {
				p2[p2Len] = '\0';
				p3[p3Len] = '\0';
				float left = 0.0f, right = 0.0f;
				if (strcmp(p2, "left") == 0) {
					left = strtof(p3, NULL);
				}
				else if (strcmp(p2, "right") == 0) {
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
		else if (strcmp(p1, "pwm") == 0) {
			if (p2 != NULL && p3 != NULL && p4 == NULL) {
				p2[p2Len] = '\0';
				p3[p3Len] = '\0';
				if (strcmp(p2, "left") == 0) {
					float left = strtof(p3, NULL);
					setMotorLSpeed(left/100.0f);
					snprintf((char*)outBuffer, outBufferLen, "Left PWM set to %.1f%%\n", left);
					ok = true;
				}
				else if (strcmp(p2, "right") == 0) {
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
		else if (strcmp(p1, "regulator") == 0) {
			if (p2 != NULL) {
				p2[p2Len] = '\0';
				if (strcmp(p2, "enable") == 0) {
					if (p3 == NULL) {
						globalSpeedRegulatorOn = ENABLE;
						strncpy((char*)outBuffer, "Regulator enabled\n", outBufferLen);
						ok = true;
					}
				}
				else if (strcmp(p2, "disable") == 0) {
					if (p3 == NULL) {
						globalSpeedRegulatorOn = DISABLE;
						strncpy((char*)outBuffer, "Regulator disabled\n", outBufferLen);
						ok = true;
					}
				}
				else if (strcmp(p2, "params") == 0) {
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
		else if (strcmp(p1, "encoder") == 0) {
			if (p2 != NULL) {
				p2[p2Len] = '\0';
				if (strcmp(p2, "left") == 0) {
					if (p3 == NULL) {
						snprintf((char*)outBuffer, outBufferLen, "Encoder left: %ld\n", getEncoderL());
						ok = true;
					}
				}
				else if (strcmp(p2, "right") == 0) {
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
		else if (strcmp(p1, "disable") == 0) {
			if (p2 == NULL) {
				globalSpeedRegulatorOn = DISABLE;
				enableMotors(DISABLE);
				strncpy((char*)outBuffer, "Motors disabled\n", outBufferLen);
				ok = true;
			}
		}
		else if (strcmp(p1, "enable") == 0) {
			if (p2 == NULL) {
				globalSpeedRegulatorOn = ENABLE;
				enableMotors(ENABLE);
				strncpy((char*)outBuffer, "Motors enabled\n", outBufferLen);
				ok = true;
			}
		}
		else if (strcmp(p1, "brake") == 0) {
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
		if (strcmp(p1, "set") == 0) {
			if (false) {} else // set command is dangerous because there is loop in printing commands and responses
			if (p2 != NULL && p3 == NULL) {
				p2[p2Len] = '\0';
				if (strcmp(p2, "command") == 0) {
					setWiFiMode(WiFiMode_Command);
					strncpy((char*)outBuffer, "Command mode set\nCommunication terminated\n", outBufferLen);
					ok = true;
				}
				else if (strcmp(p2, "data") == 0) {
					setWiFiMode(WiFiMode_Data);
					strncpy((char*)outBuffer, "Data mode set\n", outBufferLen);
					ok = true;
				}
			}
		}
		else if (strcmp(p1, "reset") == 0) {
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
		if (strcmp(p1, "all") == 0) {
			if (p2 == NULL) {
				globalLogEvents = globalLogTelemetry = globalLogSpeed = globalLogIMU = ENABLE;
				strncpy((char*)outBuffer, "Logging all\n", outBufferLen);
				ok = true;
			}
		}
		else if (strcmp(p1, "off") == 0) {
			if (p2 == NULL) {
				globalLogEvents = globalLogTelemetry = globalLogSpeed = globalLogIMU = DISABLE;
				strncpy((char*)outBuffer, "Logging off\n", outBufferLen);
				ok = true;
			}
		}
		else if (strcmp(p1, "motor") == 0) {
			strncpy((char*)outBuffer, "Not implemented\n", outBufferLen);
			ok = true;
		}
		else { // events, speed, telemetry, imu
			FunctionalState state = ENABLE;
			bool error = false;
			volatile FunctionalState *log;
			if (p2 != NULL) {
				p2[p2Len] = '\0';
				if (strcmp(p2, "off") == 0 && p3 == NULL) { // must be equal 'off' and nothing follows
					state = DISABLE;
				}
				else {
					error = true;
				}
			}
			if (strcmp(p1, "events") == 0) {
				log = &globalLogEvents;
			}
			else if (strcmp(p1, "speed") == 0) {
				log = &globalLogSpeed;
			}
			else if (strcmp(p1, "telemetry") == 0) {
				log = &globalLogTelemetry;
			}
			else if (strcmp(p1, "imu") == 0) {
				log = &globalLogIMU;
			}
			else {
				error = true;
			}
			if (!error) {
				*log = state;
				char* c = (state == ENABLE ? "enabled" : "disabled");
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
		if (strcmp(p1, "import") == 0) {
			if (p2 != NULL && p3 == NULL) {
				p2[p2Len] = '\0';
				snprintf((char*)outBuffer, outBufferLen, "<#Please send only %d points#>\n", TBloadNewPoints(strtol(p2, NULL, 10)));
				ok = true;
			}
		}
		else if (strcmp(p1, "regulator") == 0) {
			if (p2 != NULL) {
				p2[p2Len] = '\0';
				if (strcmp(p2, "params") == 0) {
					strncpy((char*)outBuffer, "Not implemented\n", outBufferLen);
					ok = true;
				}
			}
		}
		else if (strcmp(p1, "controller") == 0) {
			if (p2 != NULL) {
				p2[p2Len] = '\0';
				if (strcmp(p2, "params") == 0) {
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
		if (strcmp(p1, "scale") == 0) {
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
		else if (strcmp(p1, "wait") == 0) {
			if (p2 != NULL && p3 == NULL) {
				p2[p2Len] = '\0';
				if (strcmp(p2, "finish") == 0) {
					while (isCurrentlyDriving()) {
						vTaskDelay(10/portTICK_RATE_MS);
					}
					strncpy((char*)outBuffer, "Driving finished\n", outBufferLen);
					ok = true;
				}
			}
		}
		else if (strcmp(p1, "line") == 0) { // p2 == 'dist', p3 == mm, p4 == 'speed', p5 == 'sp', p6 optional 'pen'
			bool pen = false;
			bool error = false;
			if (p6 != NULL) {
				p6[p6Len] = '\0';
				if (strcmp(p6, "pen") == 0) {
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
				if (strcmp(p2, "dist") != 0 || strcmp(p4, "speed") != 0) {
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
		else if (strcmp(p1, "turn") == 0) { // p2 == relative or absolute, p3 == 'angle', p4 == deg, p5 == 'speed', p6 == sp, p7 == optional pen
			bool pen = false;
			bool error = false;
			bool relative;
			if (p7 != NULL) {
				p7[p7Len] = '\0';
				if (strcmp(p7, "pen") == 0) {
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
				if (strcmp(p3, "angle") != 0 || strcmp(p5, "speed") != 0) {
					error = true;
				}
				if (strcmp(p2, "relative") == 0) {
					relative = true;
				}
				else if (strcmp(p2, "absolute") == 0) {
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
		else if (strcmp(p1, "arc") == 0) { // p2 == 'angle', p3 == ang, p4 == 'radius', p5 == 'mm', p6 == 'speed', p7 == sp, p8 == optional 'pen'
			bool pen = false;
			bool error = false;
			if (p8 != NULL) {
				p8[p8Len] = '\0';
				if (strcmp(p8, "pen") == 0) {
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
				if (strcmp(p2, "angle") != 0 || strcmp(p4, "radius") != 0 || strcmp(p6, "speed") != 0) {
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
		else if (strcmp(p1, "point") == 0) { // p2 == 'x', p3 = x, p4 = 'y', p5 = y, p6 == 'speed', p7 == sp, p8 optional 'pen'
			bool pen = false;
			bool error = false;
			if (p8 != NULL) {
				p8[p8Len] = '\0';
				if (strcmp(p8, "pen") == 0) {
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
				if (strcmp(p2, "x") != 0 || strcmp(p4, "y") != 0 || strcmp(p6, "speed") != 0) {
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

void TaskCLIConstructor() {
	xTaskCreate(TaskCLI, NULL, TASKCLI_STACKSPACE, NULL, PRIORITY_TASK_CLI, &CLITask);
	CLIInputQueue = xQueueCreate(configCOMMAND_INT_MAX_OUTPUT_SIZE, sizeof(char*));
}
