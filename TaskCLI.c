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

static portBASE_TYPE wifiErrorCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command);
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

static const CLI_Command_Definition_t wifiErrorComDef = {
	(int8_t*)"[ERROR]",
	(int8_t*)"",
	wifiErrorCommand,
	0
};
static const CLI_Command_Definition_t systemComDef = {
    (int8_t*)"system",
    (int8_t*)"system <reset|battery|cpu|stack|memory|aua>\n",
    systemCommand,
    1
};
static const CLI_Command_Definition_t lanternComDef = {
    (int8_t*)"lantern",
    (int8_t*)"lantern <enable|disable>\n",
    lanternCommand,
    1
};
static const CLI_Command_Definition_t delayComDef = {
    (int8_t*)"delay",
    (int8_t*)"delay #milliseconds\n",
    delayCommand,
    1
};
static const CLI_Command_Definition_t penComDef = {
    (int8_t*)"pen",
    (int8_t*)"pen <up|down|line [solid|dotted|dashed|ldashed|dotdashed]>\n",
    penCommand,
    -1
};
static const CLI_Command_Definition_t telemetryComDef = {
    (int8_t*)"telemetry",
    (int8_t*)"telemetry [raw|<scaled [raw]>]\n"
    		 "\todometry correction [#param]\n"
    		 "\timu <enable|disable>\n",
    telemetryCommand,
    -1
};
static const CLI_Command_Definition_t motorComDef = {
    (int8_t*)"motor",
    (int8_t*)"motor ...\n"
    		 "\tspeed <<left #val>|<right #val>|<#valL #valR>>\n"
    		 "\tpwm <<left #val>|<right #val>|<#valL #valR)>>\n"
#ifndef USE_CUSTOM_MOTOR_CONTROLLER
    		 "\tregulator [enable|disable|<params [#P #I #D]>]\n"
#else
    		 "\tregulator [enable|disable|<params left|right forward|backward [<#K #B #Kp #Ki #Kd>|<K|B|Kp|Ki|Kd #val>]>]\n"
#endif
    		 "\tencoder [left|right]\n"
    		 "\tenable|disable\n"
    		 "\tbrake\n",
    motorCommand,
    -1
};
static const CLI_Command_Definition_t wifiComDef = {
    (int8_t*)"wifi",
    (int8_t*)"wifi <reset|<set <command|data>>>\n",
    wifiCommand,
    -1
};
static const CLI_Command_Definition_t logComDef = {
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
static const CLI_Command_Definition_t trajectoryComDef = {
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
static const CLI_Command_Definition_t driveComDef = {
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
			if (nOfParams == 1){
				strncpy((char*)outBuffer, "Not implemented\n", outBufferLen);
				ok = true;
			}
			else if (nOfParams == 2) {
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
					if (nOfParams == 2) {
						snprintf((char*)outBuffer, outBufferLen, "Odometry correction param: %.6f\n", globalOdometryCorrectionGain);
						ok = true;
					}
					else if (nOfParams == 3) {
						globalOdometryCorrectionGain = strtof(param[2], NULL);
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
			if (nOfParams == 1) {
				TelemetryData_Struct tl;
				getTelemetryScaled(&tl);
				snprintf((char*)outBuffer, outBufferLen, "X: %.2f\nY: %.2f\nO: %.2f\n", tl.X, tl.Y, tl.O);
				ok = true;
			}
			else if (nOfParams == 2) {
				if (cmatch("raw", param[1], 1)) { // r
					TelemetryData_Struct tl;
					getTelemetryRawScaled(&tl);
					snprintf((char*)outBuffer, outBufferLen, "X: %.2f\nY: %.2f\nO: %.2f\n", tl.X, tl.Y, tl.O);
					ok = true;
				}
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
	char *param[10];
	bool ok = false;

	size_t nOfParams = sliceCommand((char*)command, param, 10);

	if (nOfParams > 0) {
		if (cmatch("speed", param[0], 1)) { // s
			if (nOfParams == 3) {
				float left = 0.0f, right = 0.0f;
				if (cmatch("left", param[1], 1)) { // l
					left = strtof(param[2], NULL);
				}
				else if (cmatch("right", param[1], 1)) { // r
					right = strtof(param[2], NULL);
				}
				else {
					left = strtof(param[1], NULL);
					right = strtof(param[2], NULL);
				}
				snprintf((char*)outBuffer, outBufferLen, "Speeds set to %.2f left and %.2f right\n", left, right);
				sendSpeeds(left, right, 0);
				ok = true;
			}
		}
		else if (cmatch("pwm", param[0], 1)) { // p
			if (nOfParams == 3) {
				if (cmatch("left", param[1], 1)) { // l
					float left = strtof(param[2], NULL);
					setMotorLSpeed(left/100.0f);
					snprintf((char*)outBuffer, outBufferLen, "Left PWM set to %.1f%%\n", left);
					ok = true;
				}
				else if (cmatch("right", param[1], 1)) { // r
					float right = strtof(param[2], NULL);
					setMotorRSpeed(right/100.0f);
					snprintf((char*)outBuffer, outBufferLen, "Right PWM set to %.1f%%\n", right);
					ok = true;
				}
				else {
					float left = strtof(param[1], NULL);
					float right = strtof(param[2], NULL);
					setMotorLSpeed(left/100.0f);
					setMotorRSpeed(right/100.0f);
					snprintf((char*)outBuffer, outBufferLen, "Left PWM set to %.1f%%\nRight PWM set to %.1f%%\n", left, right);
					ok = true;
				}
			}
		}
		else if (cmatch("regulator", param[0], 1)) { // r
			if (nOfParams > 1) {
				if (cmatch("enable", param[1], 1)) { // e
					if (nOfParams == 2) {
						globalSpeedRegulatorOn = ENABLE;
						strncpy((char*)outBuffer, "Regulator enabled\n", outBufferLen);
						ok = true;
					}
				}
				else if (cmatch("disable", param[1], 1)) { // d
					if (nOfParams == 2) {
						globalSpeedRegulatorOn = DISABLE;
						strncpy((char*)outBuffer, "Regulator disabled\n", outBufferLen);
						ok = true;
					}
				}
				else if (cmatch("params", param[1], 1)) { // p
#ifndef USE_CUSTOM_MOTOR_CONTROLLER
					if (nOfParams == 2) {
						snprintf((char*)outBuffer, outBufferLen, "P %.3f\nI %.3f\nD %.3f\n", globalMotorPidKp, globalMotorPidKi, globalMotorPidKd);
						ok = true;
					}
					else if (nOfParams == 5) {
						taskENTER_CRITICAL();
						{
							globalMotorPidKp = strtof(param[2], NULL);
							globalMotorPidKi = strtof(param[3], NULL);
							globalMotorPidKd = strtof(param[4], NULL);
						}
						taskEXIT_CRITICAL();
						strncpy((char*)outBuffer, "New params set\n", outBufferLen);
						ok = true;
					}
#else
					if (nOfParams == 4 || nOfParams == 6 || nOfParams == 9) {
						bool dirleft, dirfwd, error = false;
						if (cmatch("left", param[2], 1))
							dirleft = true;
						else if (cmatch("right", param[2], 1))
							dirleft = false;
						else
							error = true;

						if (cmatch("forward", param[3], 1))
							dirfwd = true;
						else if (cmatch("backward", param[3], 1))
							dirfwd = false;
						else
							error = true;

						if (!error) {
							volatile MotorControllerState_Struct *mcss = dirleft ? &globalLeftMotorParams : &globalRightMotorParams;
							volatile MotorControllerPredictionParams *mcpp = dirfwd ? &mcss->forward : &mcss->backward;
							volatile PID_Params *pidp = dirfwd ? &mcss->pid2.forward : &mcss->pid2.backward;
							if (nOfParams == 4) {
								snprintf((char*)outBuffer, outBufferLen,
										"K: %.5f\nB: %.5f\nKp: %.3f\nKi: %.3f\nKd: %.3f\n",
										mcpp->K, mcpp->B, pidp->Kp, pidp->Ki, pidp->Kd);
								ok = true;
							}
							else if (nOfParams == 6) {
								volatile float* par = NULL;
								if (strcmp(param[4], "K") == 0)
									par = &mcpp->K;
								else if (strcmp(param[4], "B") == 0)
									par = &mcpp->B;
								else if (strcmp(param[4], "Kp") == 0)
									par = &pidp->Kp;
								else if (strcmp(param[4], "Ki") == 0)
									par = &pidp->Ki;
								else if (strcmp(param[4], "Kd") == 0)
									par = &pidp->Kd;

								if (par != NULL) {
									*par = strtof(param[5], NULL);
									strncpy((char*)outBuffer, "New params set\n", outBufferLen);
									ok = true;
								}
							}
							else if (nOfParams == 9) {
								mcpp->K = strtof(param[4], NULL);
								mcpp->B = strtof(param[5], NULL);
								pidp->Kp = strtof(param[6], NULL);
								pidp->Ki = strtof(param[7], NULL);
								pidp->Kd = strtof(param[8], NULL);
								strncpy((char*)outBuffer, "New params set\n", outBufferLen);
								ok = true;
							}
						}
					}
#endif
				}
			}
			else { // nOfParams == 1
				if (globalSpeedRegulatorOn == ENABLE) {
					strncpy((char*)outBuffer, "Regulator enabled\n", outBufferLen);
				}
				else {
					strncpy((char*)outBuffer, "Regulator disabled\n", outBufferLen);
				}
				ok = true;
			}
		}
		else if (cmatch("encoder", param[0], 3)) { // enc
			if (nOfParams == 1) {
				snprintf((char*)outBuffer, outBufferLen, "Encoder left: %ld\nEncoder right: %ld\n", getEncoderL(), getEncoderR());
				ok = true;
			}
			else if (nOfParams == 2) {
				if (cmatch("left", param[1], 1)) { // l
					snprintf((char*)outBuffer, outBufferLen, "Encoder left: %ld\n", getEncoderL());
					ok = true;
				}
				else if (cmatch("right", param[1], 1)) { // r
					snprintf((char*)outBuffer, outBufferLen, "Encoder right: %ld\n", getEncoderR());
					ok = true;
				}
			}
		}
		else if (cmatch("disable", param[0], 1)) { // d
			if (nOfParams == 1) {
				globalSpeedRegulatorOn = DISABLE;
				enableMotors(DISABLE);
				strncpy((char*)outBuffer, "Motors disabled\n", outBufferLen);
				ok = true;
			}
		}
		else if (cmatch("enable", param[0], 3)) { // ena
			if (nOfParams == 1) {
				globalSpeedRegulatorOn = ENABLE;
				enableMotors(ENABLE);
				strncpy((char*)outBuffer, "Motors enabled\n", outBufferLen);
				ok = true;
			}
		}
		else if (cmatch("brake", param[0], 1)) { // b
			if (nOfParams == 1) {
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
	char *param[3];
	bool ok = false;

	size_t nOfParams = sliceCommand((char*)command, param, 3);

	if (nOfParams > 0) {
		if (cmatch("set", param[0], 1)) { // s
			if (false) {} else // set command is dangerous because there is loop in printing commands and responses
			if (nOfParams == 2) {
				if (cmatch("command", param[1], 1)) { // c
					setWiFiMode(WiFiMode_Command);
					strncpy((char*)outBuffer, "Command mode set\nCommunication terminated\n", outBufferLen);
					ok = true;
				}
				else if (cmatch("data", param[1], 1)) { // d
					setWiFiMode(WiFiMode_Data);
					strncpy((char*)outBuffer, "Data mode set\n", outBufferLen);
					ok = true;
				}
			}
		}
		else if (cmatch("reset", param[0], 1)) { // r
			if (nOfParams == 1) {
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
	char *param[3];
	bool ok = false;

	size_t nOfParams = sliceCommand((char*)command, param, 3);

	if (nOfParams > 0) {
		if (cmatch("all", param[0], 1)) { // a
			if (nOfParams == 1) {
				globalLogEvents = globalLogTelemetry = globalLogSpeed = globalLogIMU = ENABLE;
				strncpy((char*)outBuffer, "Logging all\n", outBufferLen);
				ok = true;
			}
		}
		else if (cmatch("off", param[0], 1)) { // o
			if (nOfParams == 1) {
				globalLogEvents = globalLogTelemetry = globalLogSpeed = globalLogIMU = DISABLE;
				strncpy((char*)outBuffer, "Logging off\n", outBufferLen);
				ok = true;
			}
		}
		else if (cmatch("motor", param[0], 1)) { // m
			strncpy((char*)outBuffer, "Not implemented\n", outBufferLen);
			ok = true;
		}
		else { // events, speed, telemetry, imu
			if (nOfParams <= 2) {
				FunctionalState state = ENABLE;
				bool error = false;
				if (nOfParams == 2) {
					if (cmatch("off", param[1], 1)) { // must be equal 'off' and nothing follows
						state = DISABLE;
					}
					else {
						error = true;
					}
				}
				if (!error) {
					if (cmatch("events", param[0], 1)) { // e
						globalLogEvents = state;
						snprintf((char*)outBuffer, outBufferLen, "Logging %s: %s\n", "events", (state == ENABLE ? "enabled" : "disabled"));
						ok = true;
					}
					else if (cmatch("speed", param[0], 1)) { // s
						globalLogSpeed = state;
						snprintf((char*)outBuffer, outBufferLen, "Logging %s: %s\n", "speed", (state == ENABLE ? "enabled" : "disabled"));
						ok = true;
					}
					else if (cmatch("telemetry", param[0], 1)) { // t
						globalLogTelemetry = state;
						snprintf((char*)outBuffer, outBufferLen, "Logging %s: %s\n", "telemetry", (state == ENABLE ? "enabled" : "disabled"));
						ok = true;
					}
					else if (cmatch("imu", param[0], 1)) { // i
						globalLogIMU = state;
						snprintf((char*)outBuffer, outBufferLen, "Logging %s: %s\n", "imu", (state == ENABLE ? "enabled" : "disabled"));
						ok = true;
					}
				}
			}
		}
	}
	else { // nOfParams == 0
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
	char *param[3];
	bool ok = false;

	size_t nOfParams = sliceCommand((char*)command, param, 3);

	if (nOfParams > 0) {
		if (cmatch("import", param[0], 1)) { // i
			if (nOfParams == 2) {
				snprintf((char*)outBuffer, outBufferLen, "<#Please send only %d points#>\n", TBloadNewPoints(strtol(param[1], NULL, 10)));
				ok = true;
			}
		}
		else if (cmatch("regulator", param[0], 1)) { // r
			if (nOfParams == 2) {
				if (cmatch("params", param[1], 1)) { // p
					strncpy((char*)outBuffer, "Not implemented\n", outBufferLen);
					ok = true;
				}
			}
		}
		else if (cmatch("controller", param[0], 1)) { // c
			if (nOfParams == 2) {
				if (cmatch("params", param[1], 1)) { // p
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
	char *param[9];
	bool ok = false;

	size_t nOfParams = sliceCommand((char*)command, param, 9);

	if (nOfParams > 0) {
		if (cmatch("scale", param[0], 1)) { // s
			if (nOfParams == 2) {
				float s = strtof(param[1], NULL);
				if (s > 0.0f) {
					globalPositionScale = s;
					snprintf((char*)outBuffer, outBufferLen, "Scale set to %.4f\n", globalPositionScale);
				}
				else {
					strncpy((char*)outBuffer, "Scale can be only positive!\n", outBufferLen);
				}
				ok = true;
			}
			else if (nOfParams == 1) {
				snprintf((char*)outBuffer, outBufferLen, "Scale: %.4f\n", globalPositionScale);
				ok = true;
			}
		}
		else if (cmatch("wait", param[0], 1)) { // w
			if (nOfParams == 2) {
				if (cmatch("finish", param[1], 1)) { // f
					safePrint(30, "Waiting for driving to finish\n");
					while (isCurrentlyDriving()) {
						vTaskDelay(10/portTICK_RATE_MS);
					}
					strncpy((char*)outBuffer, "Driving finished\n", outBufferLen);
					ok = true;
				}
			}
		}
		else if (cmatch("line", param[0], 1)) { // l // p2 == 'dist', p3 == mm, p4 == 'speed', p5 == 'sp', p6 optional 'pen'
			if ((nOfParams == 6 || nOfParams == 5) && cmatch("dist", param[1], 1) && cmatch("speed", param[3], 1)) {
				bool pen = false;
				bool error = false;
				if (nOfParams == 6) {
					if (cmatch("pen", param[5], 1)) { // p
						pen = true;
					}
					else {
						error = true;
					}
				}
				if (!error) {
					float dist = strtof(param[2], NULL);
					float speed = strtof(param[4], NULL);
					DriveCommand_Struct *cmd = (DriveCommand_Struct*)pvPortMalloc(sizeof(DriveCommand_Struct));
					cmd->Type = DriveCommand_Type_Line;
					cmd->UsePen = pen;
					cmd->Speed = speed;
					cmd->Param1 = dist;
					xQueueSendToBack(driveQueue, &cmd, portMAX_DELAY);
					strncpy((char*)outBuffer, "Drive command accepted\n", outBufferLen);
					ok = true;
				}
			}
		}
		else if (cmatch("turn", param[0], 1)) { // t // p2 == relative or absolute, p3 == 'angle', p4 == deg, p5 == 'speed', p6 == sp, p7 == optional pen
			if ((nOfParams == 7 || nOfParams == 6) && cmatch("angle", param[2], 1) && cmatch("speed", param[4], 1)) {
				bool pen = false;
				bool error = false;
				bool relative;
				if (nOfParams == 7) {
					if (cmatch("pen", param[6], 1)) { // p
						pen = true;
					}
					else {
						error = true;
					}
				}

				if (cmatch("relative", param[1], 1)) { // r
					relative = true;
				}
				else if (cmatch("absolute", param[1], 1)) { // a
					relative = false;
				}
				else {
					error = true;
				}

				if (!error) {
					float ang = strtof(param[3], NULL);
					float speed = strtof(param[5], NULL);
					DriveCommand_Struct *cmd = (DriveCommand_Struct*)pvPortMalloc(sizeof(DriveCommand_Struct));
					cmd->Type = DriveCommand_Type_Angle;
					cmd->UsePen = pen;
					cmd->Speed = speed;
					cmd->Param1 = (relative ? 0.0f : 1.0f);
					cmd->Param2 = ang;
					xQueueSendToBack(driveQueue, &cmd, portMAX_DELAY);
					strncpy((char*)outBuffer, "Drive command accepted\n", outBufferLen);
					ok = true;
				}
			}
		}
		else if (cmatch("arc", param[0], 1)) { // a  // p2 == 'angle', p3 == ang, p4 == 'radius', p5 == 'mm', p6 == 'speed', p7 == sp, p8 == optional 'pen'
			if ((nOfParams == 8 || nOfParams == 7) && cmatch("angle", param[1], 1) && cmatch("radius", param[3], 1) && cmatch("speed", param[5], 1)) {
				bool pen = false;
				bool error = false;
				if (nOfParams == 8) {
					if (cmatch("pen", param[7], 1)) { // p
						pen = true;
					}
					else {
						error = true;
					}
				}

				if (!error) {
					float angle = strtof(param[2], NULL);
					float radius = strtof(param[4], NULL);
					float speed = strtof(param[6], NULL);
					DriveCommand_Struct *cmd = (DriveCommand_Struct*)pvPortMalloc(sizeof(DriveCommand_Struct));
					cmd->Type = DriveCommand_Type_Arc;
					cmd->UsePen = pen;
					cmd->Speed = speed;
					cmd->Param1 = radius;
					cmd->Param2 = angle;
					xQueueSendToBack(driveQueue, &cmd, portMAX_DELAY);
					strncpy((char*)outBuffer, "Drive command accepted\n", outBufferLen);
					ok = true;
				}
			}
		}
		else if (cmatch("point", param[0], 1)) { // p // p2 == 'x', p3 = x, p4 = 'y', p5 = y, p6 == 'speed', p7 == sp, p8 optional 'pen'
			if ((nOfParams == 8 || nOfParams == 7) && strcmp("x", param[1]) == 0 && strcmp("y", param[3]) == 0 && cmatch("speed", param[5], 1)) {
				bool pen = false;
				bool error = false;
				if (nOfParams == 8) {
					if (cmatch("pen", param[7], 1)) { // p
						pen = true;
					}
					else {
						error = true;
					}
				}

				if (!error) {
					float x = strtof(param[2], NULL);
					float y = strtof(param[4], NULL);
					float speed = strtof(param[6], NULL);
					DriveCommand_Struct *cmd = (DriveCommand_Struct*)pvPortMalloc(sizeof(DriveCommand_Struct));
					cmd->Type = DriveCommand_Type_Arc;
					cmd->UsePen = pen;
					cmd->Speed = speed;
					cmd->Param1 = x;
					cmd->Param2 = y;
					xQueueSendToBack(driveQueue, &cmd, portMAX_DELAY);
					strncpy((char*)outBuffer, "Drive command accepted\n", outBufferLen);
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

static portBASE_TYPE wifiErrorCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command) {
	if (getWiFiStatus() == ON) {
		enableWiFi(DISABLE);
		enableUSB(ENABLE);
		strncpy((char*)outBuffer, "WiFi error detected, check wireless connection\n", outBufferLen);
		lightLED(1, ON);
	}
	return pdFALSE;
}

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
	FreeRTOS_CLIRegisterCommand(&wifiErrorComDef);
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
