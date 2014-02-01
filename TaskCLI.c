#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS_CLI.h"

#include "ff.h"

#include "priorities.h"
#include "stackSpace.h"
#include "compilation.h"
#include "hwinterface.h"
#include "main.h"
#include "pointsBuffer.h"
#include "memory.h"
#include "streaming.h"

#include "TaskCLI.h"
#include "TaskPrintfConsumer.h"
#include "TaskMotorCtrl.h"
#include "TaskPenCtrl.h"
#include "TaskTelemetry.h"
#include "TaskWiFiMngr.h"
#include "TaskAsyncCallHandler.h"

#ifdef FOLLOW_TRAJECTORY
#include "TaskTrajectory.h"
#endif
#ifdef DRIVE_COMMANDS
#include "TaskDrive.h"
#endif

typedef struct {
	FIL *file;
	char *buffer;
	uint32_t bufferLen;
} FileTransfer_Struct;

xTaskHandle CLITask;
xQueueHandle CLIInputQueue;
xSemaphoreHandle CLILoadCompleteSemaphore;

static FIL* CLISourceFile;

static const char* const welcomeMessage = "FreeRTOS command server.\nType \"help\" to view a list of registered commands.\n";
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

/**
 * Registers commands in CLI interpreter
 */
static void registerAllCommands();

void TaskCLI(void *p) {
	portBASE_TYPE moreDataComing;
	char *msg;
	char *outputString = (char*)FreeRTOS_CLIGetOutputBuffer();
	char msgBuffer[150];

	xSemaphoreTake(CLILoadCompleteSemaphore, 0); 						/* initial take */
	registerAllCommands();

	vTaskDelay(500/portTICK_RATE_MS);
	while (xQueueReceive(CLIInputQueue, &msg, 0) == pdTRUE);

	safePrint(strlen(welcomeMessage)+1, "%s", welcomeMessage);

    while(1) {
    	if (CLISourceFile == NULL) { 									/* no file stream is open, wait for user input */
    		safePrint(strlen(promptMessage)+1, "%s", promptMessage);
    		xQueueReceive(CLIInputQueue, &msg, portMAX_DELAY); 			/* block till message is available */
    		strcpy(msgBuffer, msg);
    		vPortFree(msg);												/* free allocated resources */
    	}
    	else { 															/* stream instructions from file */
    		if (f_gets(msgBuffer, 150, CLISourceFile) != msgBuffer) { 	/* error while reading or EOF */
    			f_close(CLISourceFile);
    			vPortFree(CLISourceFile);								/* free allocated file */
    			CLISourceFile = NULL;
    			continue; 												/* start from the top */
    		}
    		size_t end = strlen(msgBuffer);
    		if (msgBuffer[end-1] == '\n')
    			msgBuffer[end-1] = '\0';
    	}

		if (strlen(msgBuffer) == 0)
			continue;

		if (strncmp(msgBuffer, "[ERROR: INVALID INPUT]", 21) == 0) {
			if (getWiFiStatus() == ON) {
				enableWiFi(DISABLE);
				if (!TaskWiFiMngrConstructor(WiFiMngr_Command_Reconnect))
					safePrint(36, "[CLI] WiFi error: Reconnect failed\n");
			}
			continue;
		}

		/* Process command and print as many lines as necessary */
		do {
			moreDataComing = FreeRTOS_CLIProcessCommand((int8_t*)msgBuffer, (int8_t*)outputString, configCOMMAND_INT_MAX_OUTPUT_SIZE);
			safePrint(strlen(outputString)+1, outputString);
		} while(moreDataComing != pdFALSE);
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
static portBASE_TYPE loadCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command);
static portBASE_TYPE catCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command);
static portBASE_TYPE lsCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command);
static portBASE_TYPE execCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command);

static const CLI_Command_Definition_t systemComDef = {
    (const int8_t*)"system",
    (const int8_t*)"system <reset|battery|cpu|stack|memory|aua|save|restore|sdcard>\n",
    systemCommand,
    1
};
static const CLI_Command_Definition_t lanternComDef = {
    (const int8_t*)"lantern",
    (const int8_t*)"lantern <enable|disable>\n",
    lanternCommand,
    1
};
static const CLI_Command_Definition_t delayComDef = {
    (const int8_t*)"delay",
    (const int8_t*)"delay #milliseconds\n",
    delayCommand,
    1
};
static const CLI_Command_Definition_t penComDef = {
    (const int8_t*)"pen",
    (const int8_t*)"pen <up|down|enable|disable|line [solid|dotted|dashed|ldashed|dotdashed]>\n",
    penCommand,
    -1
};
static const CLI_Command_Definition_t telemetryComDef = {
    (const int8_t*)"telemetry",
    (const int8_t*)"telemetry [raw|<scaled [raw]>]\n"
    		 "\todometry <<scale #turns>|<correction [#param]>>\n"
    		 "\timu <enable|disable>\n"
    		 "\tscale [#value]\n",
    telemetryCommand,
    -1
};
static const CLI_Command_Definition_t motorComDef = {
    (const int8_t*)"motor",
    (const int8_t*)"motor ...\n"
    		 "\tspeed <<left #val>|<right #val>|<#valL #valR>>\n"
    		 "\tpwm <<left #val>|<right #val>|<#valL #valR)>>\n"
#ifndef USE_CUSTOM_MOTOR_CONTROLLER
    		 "\tregulator [enable|disable|<params [#P #I #D]>]\n"
#else
    		 "\tregulator [enable|disable|<params all|[left|right forward|backward] [<#K #B #Kp #Ki #Kd>|<K|B|Kp|Ki|Kd #val>]>]\n"
#endif
    		 "\tencoder [left|right]\n"
    		 "\tenable|disable\n"
    		 "\tbrake\n",
    motorCommand,
    -1
};
static const CLI_Command_Definition_t wifiComDef = {
    (const int8_t*)"wifi",
    (const int8_t*)"wifi <reset|<set <command|data>>|reconnect>\n",
    wifiCommand,
    -1
};
static const CLI_Command_Definition_t logComDef = {
    (const int8_t*)"log",
    (const int8_t*)"log\n"
    		 "\toff\n"
    		 "\tall\n"
    		 "\t <events|telemetry|speed|imu> [off]\n"
    		 "\t motor something here\n",
    logCommand,
    -1
};
#ifdef FOLLOW_TRAJECTORY
static const CLI_Command_Definition_t trajectoryComDef = {
    (const int8_t*)"trajectory",
    (const int8_t*)"trajectory ...\n"
    		 "\tregulator params [#k_x #k #k_s]\n"
    		 "\timport <(Npoints)>\n"
    		 "\tfollow <streaming|<file #filename>|stop|reset>\n",
    trajectoryCommand,
    -1
};
#endif
#ifdef DRIVE_COMMANDS
static const CLI_Command_Definition_t driveComDef = {
    (const int8_t*)"drive",
    (const int8_t*)"drive ...\n"
    		 "\twait finish\n"
    		 "\tline dist #mm speed #speed [pen]\n"
    		 "\tturn <relative|absolute> angle #deg speed #speed [pen]\n"
    		 "\tarc angle #deg radius #mm speed #speed [pen]\n"
    		 "\tpoint x #x y #y speed #speed [pen]\n",
    driveCommand,
    -1
};
#endif
static const CLI_Command_Definition_t loadComDef = {
	(const int8_t*)"load",
	(const int8_t*)"load filename bytes [append]\n",
	loadCommand,
	-1
};
static const CLI_Command_Definition_t catComDef = {
	(const int8_t*)"cat",
	(const int8_t*)"cat filename\n",
	catCommand,
	1
};
static const CLI_Command_Definition_t lsComDef = {
	(const int8_t*)"ls",
	(const int8_t*)"ls filename\n",
	lsCommand,
	1
};
static const CLI_Command_Definition_t execComDef = {
	(const int8_t*)"exec",
	(const int8_t*)"exec filename\n",
	execCommand,
	1
};

portBASE_TYPE systemCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command) {
	char *param;
	portBASE_TYPE paramLen;

	param = (char*)FreeRTOS_CLIGetParameter(command, 1, &paramLen);
	param[paramLen] = '\0';

	if (cmatch("aua", param, 1)) { // a
		strncpy((char*)outBuffer, "I am alive!\n", outBufferLen);
	}
	else if (cmatch("memory", param, 1)) { // m
		snprintf((char*)outBuffer, outBufferLen, "Available memory: %dkB\n", xPortGetFreeHeapSize());
	}
	else if (cmatch("reset", param, 4)) { // rese
		systemReset();
	}
	else if (cmatch("battery", param, 1)) { // b
		snprintf((char*)outBuffer, outBufferLen, "Battery voltage: %.2fV\n", getBatteryVoltage());
	}
	else if (cmatch("cpu", param, 1)) { // c
		snprintf((char*)outBuffer, outBufferLen, "CPU usage: %.1f%%\n", globalCPUUsage*100.0f);
	}
	else if (cmatch("stack", param, 2)) { // st
		reportStackUsage();
		outBuffer[0] = '\0';
	}
	else if (cmatch("save", param, 2)) { // sa
		bool ok = saveConfig(InitTarget_All);
		snprintf((char*)outBuffer, outBufferLen, "System state saved %s\n", ok ? "successfully" : " but errors present");
	}
	else if (cmatch("restore", param, 4)) { // rest
		bool ok = readInit(InitTarget_All);
		snprintf((char*)outBuffer, outBufferLen, "System state restored %s\n", ok ? "successfully" : " but errors present");
	}
	else if (cmatch("sdcard", param, 2)) {
		snprintf((char*)outBuffer, outBufferLen, "SD card state: %s\n", globalSDMounted ? "mounted" : "error");
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
		else if (cmatch("down", param[0], 2)) { // do
			if (nOfParams == 1) {
				bool usePen = true;
				xQueueSendToBack(penCommandQueue, &usePen, portMAX_DELAY);
				strncpy((char*)outBuffer, "Pen is down\n", outBufferLen);
				ok = true;
			}
		}
		else if (cmatch("enable", param[0], 1)) { // e
			if (nOfParams == 1) {
				enablePen(ENABLE);
				strncpy((char*)outBuffer, "Pen enabled\n", outBufferLen);
				ok = true;
			}
		}
		else if (cmatch("disable", param[0], 2)) { // di
			if (nOfParams == 1) {
				enablePen(DISABLE);
				strncpy((char*)outBuffer, "Pen disabled\n", outBufferLen);
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
						snprintf((char*)outBuffer, outBufferLen, "Odometry correction param: %.6g\n", globalOdometryCorrectionGain);
						ok = true;
					}
					else if (nOfParams == 3) {
						globalOdometryCorrectionGain = strtof(param[2], NULL);
						snprintf((char*)outBuffer, outBufferLen, "Odometry correction param: %.6g\n", globalOdometryCorrectionGain);
						ok = true;
					}
				}
				else if (cmatch("scale", param[1], 1)) { // s
					if (nOfParams == 3) {
						int turns = atoi(param[2]);
						if (turns > 0) {
							if (!isCurrentlyDriving()) {
								AsyncCall_Type call = {
									.Type = AsyncCallProc_Int,
									.CallInt = scaleOdometryCorrectionParam,
									.IntParam = turns

								};
								xQueueSend(AsyncCallHandlerQueue, &call, portMAX_DELAY);
								strncpy((char*)outBuffer, "Scaling started\n", outBufferLen);
							}
							else {
								strncpy((char*)outBuffer, "Robot cannot drive while scaling\n", outBufferLen);
							}
						}
						else {
							strncpy((char*)outBuffer, "Only positive number of turns allowed\n", outBufferLen);
						}
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
		if (cmatch("scale", param[0], 5)) { // scale
			if (nOfParams == 2) {
				float s = strtof(param[1], NULL);
				if (s > 0.0f) {
					globalPositionScale = s;
					snprintf((char*)outBuffer, outBufferLen, "Scale set to %.4g\n", globalPositionScale);
				}
				else {
					strncpy((char*)outBuffer, "Scale can be only positive!\n", outBufferLen);
				}
				ok = true;
			}
			else if (nOfParams == 1) {
				snprintf((char*)outBuffer, outBufferLen, "Scale: %.4g\n", globalPositionScale);
				ok = true;
			}
		}
		else if (cmatch("raw", param[0], 1)) { // r
			if (nOfParams == 1) {
				TelemetryData_Struct tl;
				getTelemetry(&tl, TelemetryStyle_Raw);
				snprintf((char*)outBuffer, outBufferLen, "X: %.2f\nY: %.2f\nO: %.2f\n", tl.X, tl.Y, tl.O/DEGREES_TO_RAD);
				ok = true;
			}
		}
		else if (cmatch("scaled", param[0], 6)) { // scaled
			if (nOfParams == 1) {
				TelemetryData_Struct tl;
				getTelemetry(&tl, TelemetryStyle_Common);
				snprintf((char*)outBuffer, outBufferLen, "X: %.2f\nY: %.2f\nO: %.2f\n", tl.X, tl.Y, tl.O/DEGREES_TO_RAD);
				ok = true;
			}
			else if (nOfParams == 2) {
				if (cmatch("raw", param[1], 1)) { // r
					TelemetryData_Struct tl;
					getTelemetry(&tl, TelemetryStyle_Scaled);
					snprintf((char*)outBuffer, outBufferLen, "X: %.2f\nY: %.2f\nO: %.2f\n", tl.X, tl.Y, tl.O/DEGREES_TO_RAD);
					ok = true;
				}
			}
		}
	}
	else {
		TelemetryData_Struct tl;
		getTelemetry(&tl, TelemetryStyle_Normalized);
		snprintf((char*)outBuffer, outBufferLen, "X: %.2f\nY: %.2f\nO: %.2f\n", tl.X, tl.Y, tl.O/DEGREES_TO_RAD);
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
				snprintf((char*)outBuffer, outBufferLen, "Speeds set to %.3g left and %.3g right\n", left, right);
				sendSpeeds(left, right, 0);
				ok = true;
			}
		}
		else if (cmatch("pwm", param[0], 1)) { // p
			if (nOfParams == 3) {
				if (cmatch("left", param[1], 1)) { // l
					float left = strtof(param[2], NULL);
					setMotorLSpeed(left/100.0f);
					snprintf((char*)outBuffer, outBufferLen, "Left PWM set to %.1g%%\n", left);
					ok = true;
				}
				else if (cmatch("right", param[1], 1)) { // r
					float right = strtof(param[2], NULL);
					setMotorRSpeed(right/100.0f);
					snprintf((char*)outBuffer, outBufferLen, "Right PWM set to %.1g%%\n", right);
					ok = true;
				}
				else {
					float left = strtof(param[1], NULL);
					float right = strtof(param[2], NULL);
					setMotorLSpeed(left/100.0f);
					setMotorRSpeed(right/100.0f);
					snprintf((char*)outBuffer, outBufferLen, "Left PWM set to %.1g%%\nRight PWM set to %.1g%%\n", left, right);
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
					if (nOfParams >= 3) {
						bool dirleft, dirfwd, error = false, all = false;
						uint8_t curParr = 2;

						if (cmatch("left", param[2], 1)) {
							dirleft = true;
							curParr++;
						}
						else if (cmatch("right", param[2], 1)) {
							dirleft = false;
							curParr++;
						}
						else if (cmatch("all", param[2], 1)) {
							all = true;
							curParr++;
						}
						else {
							error = true;
						}

						if (!all && !error) {
							if (cmatch("forward", param[3], 1)) {
								dirfwd = true;
								curParr++;
							}
							else if (cmatch("backward", param[3], 1)) {
								dirfwd = false;
								curParr++;
							}
							else {
								error = true;
							}
						}

						if (!error) {
							volatile MotorControllerState_Struct *mcss = dirleft ? &globalLeftMotorParams : &globalRightMotorParams;
							volatile MotorControllerPredictionParams *mcpp = dirfwd ? &mcss->forward : &mcss->backward;
							volatile PID_Params *pidp = dirfwd ? &mcss->pid2.forward : &mcss->pid2.backward;

							if (nOfParams == curParr) {
								if (all) {
									snprintf((char*)outBuffer, outBufferLen,
										"Left forward:\n"
										"\tK: %.5g\n\tB: %.5g\n\tKp: %.3g\n\tKi: %.3g\n\tKd: %.3g\n"
										"Left backward:\n"
										"\tK: %.5g\n\tB: %.5g\n\tKp: %.3g\n\tKi: %.3g\n\tKd: %.3g\n"
										"Right forward:\n"
										"\tK: %.5g\n\tB: %.5g\n\tKp: %.3g\n\tKi: %.3g\n\tKd: %.3g\n"
										"Right backward:\n"
										"\tK: %.5g\n\tB: %.5g\n\tKp: %.3g\n\tKi: %.3g\n\tKd: %.3g\n",
										globalLeftMotorParams.forward.K,
										globalLeftMotorParams.forward.B,
										globalLeftMotorParams.pid2.forward.Kp,
										globalLeftMotorParams.pid2.forward.Ki,
										globalLeftMotorParams.pid2.forward.Kd,
										globalLeftMotorParams.backward.K,
										globalLeftMotorParams.backward.B,
										globalLeftMotorParams.pid2.backward.Kp,
										globalLeftMotorParams.pid2.backward.Ki,
										globalLeftMotorParams.pid2.backward.Kd,
										globalRightMotorParams.forward.K,
										globalRightMotorParams.forward.B,
										globalRightMotorParams.pid2.forward.Kp,
										globalRightMotorParams.pid2.forward.Ki,
										globalRightMotorParams.pid2.forward.Kd,
										globalRightMotorParams.backward.K,
										globalRightMotorParams.backward.B,
										globalRightMotorParams.pid2.backward.Kp,
										globalRightMotorParams.pid2.backward.Ki,
										globalRightMotorParams.pid2.backward.Kd
									);
								}
								else {
									snprintf((char*)outBuffer, outBufferLen,
										"K: %.5g\nB: %.5g\nKp: %.3g\nKi: %.3g\nKd: %.3g\n",
										mcpp->K, mcpp->B, pidp->Kp, pidp->Ki, pidp->Kd
									);

								}
								ok = true;
							}
							else if (nOfParams == curParr+2) {
								if (all) {
									float newVal = strtof(param[4], NULL);
									if (strcmp(param[3], "K") == 0) {
										globalLeftMotorParams.forward.K = newVal;
										globalLeftMotorParams.backward.K = newVal;
										globalRightMotorParams.forward.K = newVal;
										globalRightMotorParams.backward.K = newVal;
										ok = true;
									}
									else if (strcmp(param[3], "B") == 0) {
										globalLeftMotorParams.forward.B = newVal;
										globalLeftMotorParams.backward.B = newVal;
										globalRightMotorParams.forward.B = newVal;
										globalRightMotorParams.backward.B = newVal;
										ok = true;
									}
									else if (strcmp(param[3], "Kp") == 0) {
										globalLeftMotorParams.pid2.forward.Kp = newVal;
										globalLeftMotorParams.pid2.backward.Kp = newVal;
										globalRightMotorParams.pid2.forward.Kp = newVal;
										globalRightMotorParams.pid2.backward.Kp = newVal;
										ok = true;
									}
									else if (strcmp(param[3], "Ki") == 0) {
										globalLeftMotorParams.pid2.forward.Ki = newVal;
										globalLeftMotorParams.pid2.backward.Ki = newVal;
										globalRightMotorParams.pid2.forward.Ki = newVal;
										globalRightMotorParams.pid2.backward.Ki = newVal;
										ok = true;
									}
									else if (strcmp(param[3], "Kd") == 0) {
										globalLeftMotorParams.pid2.forward.Kd = newVal;
										globalLeftMotorParams.pid2.backward.Kd = newVal;
										globalRightMotorParams.pid2.forward.Kd = newVal;
										globalRightMotorParams.pid2.backward.Kd = newVal;
										ok = true;
									}

									if (ok)
										strncpy((char*)outBuffer, "New params set\n", outBufferLen);
								}
								else {
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
							}
							else if (nOfParams == curParr+5) {
								if (!all) {
									mcpp->K = strtof(param[4], NULL);
									mcpp->B = strtof(param[5], NULL);
									pidp->Kp = strtof(param[6], NULL);
									pidp->Ki = strtof(param[7], NULL);
									pidp->Kd = strtof(param[8], NULL);
									strncpy((char*)outBuffer, "New params set\n", outBufferLen);
									ok = true;
								}
								else {
									strncpy((char*)outBuffer, "Cannot change all params that way!\n", outBufferLen);
									ok = true;
								}
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
			if (false) {} else // TODO: set command is dangerous because there is loop in printing commands and responses
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
		else if (cmatch("reset", param[0], 3)) { // res
			if (nOfParams == 1) {
				setWiFiReset(ENABLE);
				vTaskDelay(200/portTICK_RATE_MS);
				setWiFiReset(DISABLE);
				strncpy((char*)outBuffer, "WiFi reset\n", outBufferLen);
				ok = true;
			}
		}
		else if (cmatch("reconnect", param[0], 3)) { // rec
			if (nOfParams == 1) {
				if (!TaskWiFiMngrConstructor(WiFiMngr_Command_Reconnect))
					strncpy((char*)outBuffer, "Reconnect failed\n", outBufferLen);
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
	char *param[6];
	bool ok = false;
/* TODO:
 *     (const int8_t*)"trajectory ...\n"
    		 "\tregulator params [iles paramsow]\n"
    		 "\timport <(Npoints)>\n"
    		 "\tfollow <streaming|<file #filename>|stop|reset>\n",
 */
	size_t nOfParams = sliceCommand((char*)command, param, 6);

	if (nOfParams > 0) {
		if (cmatch("import", param[0], 1)) { // i
			if (nOfParams == 2) {
				snprintf((char*)outBuffer, outBufferLen, "Trajectory: send %d points\n", TBloadNewPoints(strtol(param[1], NULL, 10)));
				ok = true;
			}
		}
		else if (cmatch("regulator", param[0], 1)) { // r
			if (nOfParams > 1) {
				if (cmatch("params", param[1], 1)) { // p
					if (nOfParams == 2) {
						snprintf((char*)outBuffer, outBufferLen, "k_x: %.2g\nk: %.2g\nk_s: %.2g\n",
								globalTrajectoryControlGains.k_x,
								globalTrajectoryControlGains.k,
								globalTrajectoryControlGains.k_s);
						ok = true;
					}
					else if (nOfParams == 5) {
						globalTrajectoryControlGains.k_x = strtof(param[2], NULL);
						globalTrajectoryControlGains.k = strtof(param[3], NULL);
						globalTrajectoryControlGains.k_s = strtof(param[4], NULL);
						strncpy((char*)outBuffer, "Parameters changed\n", outBufferLen);
						ok = true;
					}
				}
			}
		}
		else if (cmatch("follow", param[0], 1)) { // f
			if (nOfParams > 1) {
				if (cmatch("streaming", param[1], 1)) { // str
					TrajectoryRequest_Struct request;
					request.source = TrajectorySource_Streaming;
					xQueueSendToBack(trajectoryRequestQueue, &request, portMAX_DELAY);
					strncpy((char*)outBuffer, "Request sent\n", outBufferLen);
					ok = true;
				}
				else if (cmatch("stop", param[1], 1)) { // sto
					xSemaphoreGive(trajectoryStopSemaphore);
					strncpy((char*)outBuffer, "Request sent\n", outBufferLen);
					ok = true;
				}
				else if (cmatch("reset", param[1], 1)) { // r
					xSemaphoreGive(trajectoryStopSemaphore);
					TBresetBuffer();
					strncpy((char*)outBuffer, "Reset OK\n", outBufferLen);
					ok = true;
				}
				else if (cmatch("file", param[1], 1)) { // f
					if (nOfParams == 3) {
						if (!globalSDMounted) {
							strncpy((char*)outBuffer, "SD card not mounted\n", outBufferLen);
							ok = true;
						}
						else {
// TODO: do not open it here!!!
							TrajectoryRequest_Struct request;
							request.source = TrajectorySource_File;
							FIL *file = pvPortMalloc(sizeof(FIL));
							FRESULT res = f_open(file, param[2], FA_READ | FA_OPEN_EXISTING);
							if (res == FR_OK) {
								request.file_ptr = file;
								xQueueSendToBack(trajectoryRequestQueue, &request, portMAX_DELAY);
								strncpy((char*)outBuffer, "Request sent\n", outBufferLen);
								ok = true;
							}
							else {
								vPortFree(file);
								strncpy((char*)outBuffer, "Could not open file or file does not exist\n", outBufferLen);
								ok = true;
							}
						}
					}
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
		if (cmatch("wait", param[0], 1)) { // w
			if (nOfParams == 2) {
				if (cmatch("finish", param[1], 1)) { // f
					safePrint(31, "Waiting for driving to finish\n");
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
					cmd->Smooth = false;
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
					cmd->Smooth = false;
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
					cmd->Smooth = false;
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
					cmd->Type = DriveCommand_Type_Point;
					cmd->UsePen = pen;
					cmd->Speed = speed;
					cmd->Param1 = x;
					cmd->Param2 = y;
					cmd->Smooth = false;
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

static FIL* createFileHandle(const TCHAR* filename, BYTE mode, char* outBuffer, size_t outBufferLen) {
	if (!globalSDMounted) {
		strncpy((char*)outBuffer, "Error - SD card not present\n", outBufferLen);
		return NULL;
	}

	FIL* file = pvPortMalloc(sizeof(FIL));
	FRESULT res = f_open(file, filename, mode);

	if (res == FR_DENIED) {
		strncpy((char*)outBuffer, "Access to file denied\n", outBufferLen);
	}
	else if (res == FR_INVALID_NAME) {
		strncpy((char*)outBuffer, "Invalid filename specified\n", outBufferLen);
	}
	else if (res == FR_NO_FILE || res == FR_NO_PATH) {
		strncpy((char*)outBuffer, "File does not exist\n", outBufferLen);
	}
	else if (res != FR_OK) {
		strncpy((char*)outBuffer, "Error opening file\n", outBufferLen);
	}

	if (res != FR_OK) {
		vPortFree(file);
		return NULL;
	}

	return file;
}

/* Called from ISR */
void fileTransferHandle(void *p) {
	signed portBASE_TYPE contextSwitch = pdFALSE;
	xSemaphoreGiveFromISR(CLILoadCompleteSemaphore, &contextSwitch);
	portEND_SWITCHING_ISR(contextSwitch);
}

portBASE_TYPE loadCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command) {
	char *param[4];
	const uint32_t maxPayload = 2000;

	size_t nOfParams = sliceCommand((char*)command, param, 4);
	if (nOfParams != 2 && !(nOfParams == 3 && cmatch("append", param[2], 1))) {
		strncpy((char*)outBuffer, incorrectMessage, outBufferLen);
		return pdFALSE;
	}

	int len = atoi(param[1]);
	if (len <= 0) {
		strncpy((char*)outBuffer, "Error - incorrect number of bytes specified\n", outBufferLen);
		return pdFALSE;
	}
	if (len > maxPayload) {
		snprintf((char*)outBuffer, outBufferLen, "Error - payload too big - max is %ld\n", maxPayload);
		return pdFALSE;
	}

	if (getWiFiStatus() == OFF) {
		strncpy((char*)outBuffer, "Error - WiFi is needed for this transfer\n", outBufferLen);
		return pdFALSE;
	}

	BYTE mode = FA_WRITE;
	if (nOfParams == 3) {
		mode |= FA_OPEN_ALWAYS;
	}
	else {
		mode |= FA_CREATE_ALWAYS;
	}

	FIL *file = createFileHandle(param[0], mode, (char*)outBuffer, outBufferLen);
	if (file == NULL) {
		return pdFALSE;
	}

	if (nOfParams == 3) {
		if (f_lseek(file, f_size(file)) != FR_OK) {
			strncpy((char*)outBuffer, "Error appending to file\n", outBufferLen);
			f_close(file);
			vPortFree(file);
			return pdFALSE;
		}
	}

	/* Indicate ongoing transmission */
	safePrint(18, "Send data now...\n");

	char* writeBuffer = pvPortMalloc(len*sizeof(char));
	streamAcquire(portMAX_DELAY);
	streamStartTransmission(writeBuffer, len, fileTransferHandle, NULL);

	/* Wait for transmission to finish */
	xSemaphoreTake(CLILoadCompleteSemaphore, portMAX_DELAY);

	/* Clean up */
	streamFinishTransmission();
	streamRelease();

	UINT written;
	f_write(file, writeBuffer, len, &written);

	f_close(file);
	vPortFree(file);
	vPortFree(writeBuffer);

	strncpy((char*)outBuffer, "Transfer complete\n", outBufferLen);
	return pdFALSE;
}

portBASE_TYPE catCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command) {
	char *param[1];
	sliceCommand((char*)command, param, 1);
	// nOfParams is checked by OS - must be 1

	FIL* file = createFileHandle(param[0], FA_READ | FA_OPEN_EXISTING, (char*)outBuffer, outBufferLen);
	if (file == NULL) {
		return pdFALSE;   /* Error output is already present in outBuffer */
	}

	/* Allocate buffer for reading */
	char *buffer = pvPortMalloc(500*sizeof(char));
	UINT bytesRead;

	/* Acquire resources for printing */
	xSemaphoreTake(printfMutex, portMAX_DELAY);

	/* Read the whole file and print its content */
	do {
		f_read(file, buffer, 500, &bytesRead);
		printInterfaceBlocking(buffer, bytesRead, Interface_All_Active);
	} while (bytesRead == 500);

	/* Release resources, free space, close file */
	xSemaphoreGive(printfMutex);
	vPortFree(buffer);
	f_close(file);
	vPortFree(file);

	strncpy((char*)outBuffer, "\n", outBufferLen);
	return pdFALSE;
}

portBASE_TYPE execCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command) {
	if (CLISourceFile != NULL) {
		strncpy((char*)outBuffer, "Error - another file is currently being executed\n", outBufferLen);
		return pdFALSE;
	}

	char *param[1]; // nOfParams is checked by OS - must be 1
	sliceCommand((char*)command, param, 1);

	/* open file for reading */
	CLISourceFile = createFileHandle(param[0], FA_READ | FA_OPEN_EXISTING, (char*)outBuffer, outBufferLen);

	if (CLISourceFile != NULL) {
		strncpy((char*)outBuffer, "File execution started\n", outBufferLen);
	}

	return pdFALSE;
}

portBASE_TYPE lsCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command) {
	if (!globalSDMounted) {
		strncpy((char*)outBuffer, "Error - SD card not present\n", outBufferLen);
		return pdFALSE;
	}

	char *param[1];
	sliceCommand((char*)command, param, 1);
	// nOfParams is checked by OS - must be 1

	DIR dir;
	FRESULT res = f_opendir(&dir, param[0]);

	if (res != FR_OK) {
		strncpy((char*)outBuffer, "Error opening directory\n", outBufferLen);
		return pdFALSE;
	}

	TCHAR *lfn = pvPortMalloc((1+_MAX_LFN)*sizeof(TCHAR));
	FILINFO fno;
	fno.lfname = lfn;
	fno.lfsize = _MAX_LFN + 1;

	while((res = f_readdir(&dir, &fno)) == FR_OK) {
		if (fno.fname[0] == '\0')
			break;

		if (fno.fattrib & AM_DIR)
			safePrint(12, "\n[  DIR  ]");
		else
			safePrint(12, "[%5ldB ]", fno.fsize);

		safePrint(50, " %s\n", *fno.lfname ? fno.lfname : fno.fname);
	}
	f_closedir(&dir);

	vPortFree(lfn);
	strncpy((char*)outBuffer, "", outBufferLen);
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
	FreeRTOS_CLIRegisterCommand(&loadComDef);
	FreeRTOS_CLIRegisterCommand(&catComDef);
	FreeRTOS_CLIRegisterCommand(&lsComDef);
	FreeRTOS_CLIRegisterCommand(&execComDef);
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

size_t sliceCommand(char *command, char **params, const size_t n) {
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
	CLIInputQueue = xQueueCreate(100, sizeof(char*));
	vSemaphoreCreateBinary(CLILoadCompleteSemaphore);
}
