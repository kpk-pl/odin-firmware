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
#include "wifiactions.h"
#include "radioRcvr.h"

#include "TaskCLI.h"
#include "TaskPrintfConsumer.h"
#include "TaskMotorCtrl.h"
#include "TaskPenCtrl.h"
#include "TaskTelemetry.h"
#include "TaskAsyncCallHandler.h"
#include "TaskTrajectory.h"

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
				AsyncCall_Type call = {
					.Type = AsyncCallProc_Int,
					.CallInt = reconnectToAP,
					.IntParam = (int)true
				};
				if (xQueueSendToBack(AsyncCallHandlerQueue, &call, 0) != pdTRUE) {
					lightLED(1, ON);
				}
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
static portBASE_TYPE trajectoryCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command);
#ifdef DRIVE_COMMANDS
static portBASE_TYPE driveCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command);
#endif
static portBASE_TYPE loadCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command);
static portBASE_TYPE catCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command);
static portBASE_TYPE lsCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command);
static portBASE_TYPE execCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command);
static portBASE_TYPE radioCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command);

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
    		 "\tscale [#value]\n",
    telemetryCommand,
    -1
};
static const CLI_Command_Definition_t motorComDef = {
    (const int8_t*)"motor",
    (const int8_t*)"motor ...\n"
    		 "\tspeed [<left #val>|<right #val>|<#valL #valR>]\n"
    		 "\tpwm <<left #val>|<right #val>|<#valL #valR)>>\n"
    		 "\tregulator [enable|disable|<params all|[left|right forward|backward] [<#K #B #Kp #Ki #Kd>|<K|B|Kp|Ki|Kd #val>]>]\n"
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
    (const int8_t*)"log <<all|events|log|error|debug|rc5|speed|ordered|telemetry|camera|drive> [off]>\n",
    logCommand,
    -1
};
static const CLI_Command_Definition_t trajectoryComDef = {
    (const int8_t*)"trajectory",
    (const int8_t*)"trajectory ...\n"
    		 "\tregulator params [#k_x #k #k_s]\n"
    		 "\timport <(Npoints)>\n"
    		 "\tfollow <streaming|<file #filename>|stop|reset>\n",
    trajectoryCommand,
    -1
};
#ifdef DRIVE_COMMANDS
static const CLI_Command_Definition_t driveComDef = {
    (const int8_t*)"drive",
    (const int8_t*)"drive ...\n"
    		 "\twait finish\n"
    		 "\tline dist #mm speed #speed [pen] [smooth]\n"
    		 "\tturn <relative|absolute> angle #deg speed #speed [pen] [smooth]\n"
    		 "\tarc angle #deg radius #mm speed #speed [pen] [smooth]\n"
    		 "\tpoint x #x y #y speed #speed [pen] [smooth]\n",
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
static const CLI_Command_Definition_t radioComDef = {
	(const int8_t*)"radio",
	(const int8_t*)"radio <on|off|test|reset>\n",
	radioCommand,
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
				xQueueOverwrite(penCommandQueue, &usePen);
				strncpy((char*)outBuffer, "Pen is up\n", outBufferLen);
				ok = true;
			}
		}
		else if (cmatch("down", param[0], 2)) { // do
			if (nOfParams == 1) {
				bool usePen = true;
				xQueueOverwrite(penCommandQueue, &usePen);
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
				snprintf((char*)outBuffer, outBufferLen, "[Tel] X: %.2f Y: %.2f O: %.1f\n", tl.X, tl.Y, tl.O/DEGREES_TO_RAD);
				ok = true;
			}
		}
		else if (cmatch("scaled", param[0], 6)) { // scaled
			if (nOfParams == 1) {
				TelemetryData_Struct tl;
				getTelemetry(&tl, TelemetryStyle_Common);
				snprintf((char*)outBuffer, outBufferLen, "[Tel] X: %.2f Y: %.2f O: %.1f\n", tl.X, tl.Y, tl.O/DEGREES_TO_RAD);
				ok = true;
			}
			else if (nOfParams == 2) {
				if (cmatch("raw", param[1], 1)) { // r
					TelemetryData_Struct tl;
					getTelemetry(&tl, TelemetryStyle_Scaled);
					snprintf((char*)outBuffer, outBufferLen, "[Tel] X: %.2f Y: %.2f O: %.1f\n", tl.X, tl.Y, tl.O/DEGREES_TO_RAD);
					ok = true;
				}
			}
		}
	}
	else {
		TelemetryData_Struct tl;
		getTelemetry(&tl, TelemetryStyle_Normalized);
		snprintf((char*)outBuffer, outBufferLen, "[Tel] X: %.2f Y: %.2f O: %.1f\n", tl.X, tl.Y, tl.O/DEGREES_TO_RAD);
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
			if (nOfParams == 1) {
				snprintf((char*)outBuffer, outBufferLen, "[Speed] L: %.2f R: %.2f\n",
						globalCurrentMotorSpeed.LeftSpeed,
						globalCurrentMotorSpeed.RightSpeed);
				ok = true;
			}
			else if (nOfParams == 3) {
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
				sendSpeeds(left, right);
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
									ok = true;
									if (strcmp(param[3], "K") == 0) {
										globalLeftMotorParams.forward.K = newVal;
										globalLeftMotorParams.backward.K = newVal;
										globalRightMotorParams.forward.K = newVal;
										globalRightMotorParams.backward.K = newVal;
									}
									else if (strcmp(param[3], "B") == 0) {
										globalLeftMotorParams.forward.B = newVal;
										globalLeftMotorParams.backward.B = newVal;
										globalRightMotorParams.forward.B = newVal;
										globalRightMotorParams.backward.B = newVal;
									}
									else if (strcmp(param[3], "Kp") == 0) {
										globalLeftMotorParams.pid2.forward.Kp = newVal;
										globalLeftMotorParams.pid2.backward.Kp = newVal;
										globalRightMotorParams.pid2.forward.Kp = newVal;
										globalRightMotorParams.pid2.backward.Kp = newVal;
									}
									else if (strcmp(param[3], "Ki") == 0) {
										globalLeftMotorParams.pid2.forward.Ki = newVal;
										globalLeftMotorParams.pid2.backward.Ki = newVal;
										globalRightMotorParams.pid2.forward.Ki = newVal;
										globalRightMotorParams.pid2.backward.Ki = newVal;
									}
									else if (strcmp(param[3], "Kd") == 0) {
										globalLeftMotorParams.pid2.forward.Kd = newVal;
										globalLeftMotorParams.pid2.backward.Kd = newVal;
										globalRightMotorParams.pid2.forward.Kd = newVal;
										globalRightMotorParams.pid2.backward.Kd = newVal;
									}
									else
										ok = false;

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
								}
								else {
									strncpy((char*)outBuffer, "Cannot change all params that way!\n", outBufferLen);
								}
								ok = true;
							}
						}
					}
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
			}
			else if (nOfParams == 2) {
				if (cmatch("left", param[1], 1)) { // l
					snprintf((char*)outBuffer, outBufferLen, "Encoder left: %ld\n", getEncoderL());
				}
				else if (cmatch("right", param[1], 1)) { // r
					snprintf((char*)outBuffer, outBufferLen, "Encoder right: %ld\n", getEncoderR());
				}
			}
			ok = true;
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
				sendSpeeds(0.0f, 0.0f);
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
				AsyncCall_Type call = {
					.Type = AsyncCallProc_Int,
					.CallInt = reconnectToAP,
					.IntParam = (int)true
				};
				if (xQueueSendToBack(AsyncCallHandlerQueue, &call, 0) != pdTRUE)
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

	if (nOfParams == 0) {
		strncpy((char*)outBuffer, "Logging settings:\nall: ?\nevents: ?\nlog: ?\nerror: ?\ndebug: ?\nrc5: ?\n"
				"speed: ?\nspeed ordered: ?\ntelemetry: ?\ndrive: ?\ncamera: ?\n", outBufferLen);
		outBuffer[23] = globalLogSettings.enableAll ? '1' : '0';
		outBuffer[33] = globalLogSettings.enableEvents ? '1' : '0';
		outBuffer[40] = globalLogSettings.enableLog ? '1' : '0';
		outBuffer[49] = globalLogSettings.enableError ? '1' : '0';
		outBuffer[58] = globalLogSettings.enableDebug ? '1' : '0';
		outBuffer[65] = globalLogSettings.enableRC5 ? '1' : '0';
		outBuffer[74] = globalLogSettings.enableSpeed ? '1' : '0';
		outBuffer[91] = globalLogSettings.enableSpeedOrdered ? '1' : '0';
		outBuffer[104] = globalLogSettings.enableTelemetry ? '1' : '0';
		outBuffer[113] = globalLogSettings.enableDrive ? '1' : '0';
		outBuffer[123] = globalLogSettings.enableCamera ? '1' : '0';
		ok = true;

#if configCOMMAND_INT_MAX_OUTPUT_SIZE < 125
#error ERROR_IN_CLI
#endif
	}
	else if (nOfParams <= 2) {
		bool off = false;
		if (nOfParams == 2) {
			if (cmatch("off", param[1], 1)) {
				off = true;
			}
		}
		if (nOfParams == 1 || off) {
			ok = true;
			if (cmatch("all", param[0], 1))				// a
				globalLogSettings.enableAll = !off;
			else if (cmatch("events", param[0], 2))		// ev
				globalLogSettings.enableEvents = !off;
			else if (cmatch("log", param[0], 1))		// l
				globalLogSettings.enableLog = !off;
			else if (cmatch("error", param[0], 2))		// er
				globalLogSettings.enableError = !off;
			else if (cmatch("debug", param[0], 2))		// de
				globalLogSettings.enableDebug = !off;
			else if (cmatch("rc5", param[0], 1))		// r
				globalLogSettings.enableRC5 = !off;
			else if (cmatch("speed", param[0], 1))		// s
				globalLogSettings.enableSpeed = !off;
			else if (cmatch("ordered", param[0], 1))	// o
				globalLogSettings.enableSpeedOrdered = !off;
			else if (cmatch("telemetry", param[0], 1))	// t
				globalLogSettings.enableTelemetry = !off;
			else if (cmatch("drive", param[0], 2))		// dr
				globalLogSettings.enableDrive = !off;
			else if (cmatch("camera", param[0], 1))		// c
				globalLogSettings.enableCamera = !off;
			else
				ok = false;
			if (ok)
				strncpy((char*)outBuffer, "Logging set\n", outBufferLen);
		}
	}

	if (!ok) {
		strncpy((char*)outBuffer, incorrectMessage, outBufferLen);
	}

	return pdFALSE;
}

portBASE_TYPE trajectoryCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command) {
	char *param[6];
	bool ok = false;
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
							TrajectoryRequest_Struct request;
							request.source = TrajectorySource_File;
							request.fileName = pvPortMalloc((strlen(param[2])+1)*sizeof(char));
							strcpy(request.fileName, param[2]);
							if (xQueueSendToBack(trajectoryRequestQueue, &request, portMAX_DELAY) == pdTRUE) {
								strncpy((char*)outBuffer, "Request sent\n", outBufferLen);
							}
							else {
								vPortFree(request.fileName);
								strncpy((char*)outBuffer, "Could not send request\n", outBufferLen);
							}
							ok = true;
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

#ifdef DRIVE_COMMANDS
portBASE_TYPE driveCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command) {
	char *param[10];
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
		else if (cmatch("line", param[0], 1) || cmatch("turn", param[0], 1) || cmatch("arc", param[0], 1) || cmatch("point", param[0], 1)) { // l || t || a || p
			/* Initial command with defaults */
			DriveCommand_Struct cmd = {
				.Smooth = false,
				.UsePen = false
			};
			size_t currParam = 1;

			/* One time loop to allow break statement */
			do {
				/* Minimum is 5 for line command*/
				if (nOfParams < 5)
					break;

				/* Get command type */
				switch(param[0][0]) {
				case 'l':
					cmd.Type = DriveCommand_Type_Line;
					break;
				case 't':
					cmd.Type = DriveCommand_Type_Angle;
					break;
				case 'a':
					cmd.Type = DriveCommand_Type_Arc;
					break;
				case 'p':
					cmd.Type = DriveCommand_Type_Point;
					break;
				}

				/* Handle relative/absolute switch for turn */
				if (cmd.Type == DriveCommand_Type_Angle) {
					if (cmatch("relative", param[currParam], 1))
						cmd.Param1 = DRIVECOMMAND_ANGLE_PARAM1_RELATIVE;
					else if (cmatch("absolute", param[currParam], 1))
						cmd.Param1 = DRIVECOMMAND_ANGLE_PARAM1_ABSOLUTE;
					else
						break;
					currParam++;
				}

				/* Check all constant strings */
				if (cmd.Type == DriveCommand_Type_Line && !cmatch("dist", param[currParam], 1))
					break;
				else if (cmd.Type == DriveCommand_Type_Point && !(cmatch("x", param[currParam], 1) && cmatch("y", param[currParam+2], 1)))
					break;
				else if (cmd.Type == DriveCommand_Type_Arc && !cmatch("radius", param[currParam+2], 1))
					break;
				else if ((cmd.Type == DriveCommand_Type_Angle || cmd.Type == DriveCommand_Type_Arc) && !cmatch("angle", param[currParam], 1))
					break;

				/* Handle all parameters */
				if (cmd.Type == DriveCommand_Type_Line || cmd.Type == DriveCommand_Type_Point)
					cmd.Param1 = strtof(param[currParam+1], NULL);
				else // line or arc here
					cmd.Param2 = strtof(param[currParam+1], NULL);
				currParam += 2;
				if (cmd.Type == DriveCommand_Type_Point) {
					cmd.Param2 = strtof(param[currParam+1], NULL);
					currParam += 2;
				}
				else if (cmd.Type == DriveCommand_Type_Arc) {
					cmd.Param1 = strtof(param[currParam+1], NULL);
					currParam += 2;
				}

				/* Handle speed */
				if (nOfParams < currParam + 2 || !cmatch("speed", param[currParam], 1))
					break;
				cmd.Speed = strtof(param[currParam+1], NULL);
				currParam += 2;

				/* Handle optional pen */
				if (nOfParams >= currParam + 1 && cmatch("pen", param[currParam], 1)) {
					cmd.UsePen = true;
					currParam++;
				}

				/* Handle optional smooth */
				if (nOfParams >= currParam + 1 && cmatch("smooth", param[currParam], 1)) {
					cmd.Smooth = true;
					currParam++;
				}

				/* Check if there is no more parameters */
				if (nOfParams > currParam)
					break;

				/* All in, send */
				DriveCommand_Struct *pcmd = pvPortMalloc(sizeof(DriveCommand_Struct));
				*pcmd = cmd;
				xQueueSendToBack(driveQueue, &pcmd, portMAX_DELAY);
				strncpy((char*)outBuffer, "Drive command accepted\n", outBufferLen);
				ok = true;
			} while(0);
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
//TODO: loop here to load continuously to buffer of length 512, handle errors when timing fails
	char* writeBuffer = pvPortMalloc(len*sizeof(char));
	streamAcquire(portMAX_DELAY);
	streamStartTransmission(writeBuffer, len, fileTransferHandle, NULL);

	/* Wait for transmission to finish */
	xSemaphoreTake(CLILoadCompleteSemaphore, portMAX_DELAY);

	/* Clean up */
	streamFinishTransmission();
	streamRelease();

// TODO: library bug here
	UINT written;
	for (uint16_t i = 0; i<len; i += 1023) {
		f_write(file, &writeBuffer[i], (len - i > 1023 ? 1023 : len-i), &written);
	}
	//f_write(file, writeBuffer, len, &written);

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
			safePrint(12, "\n[%5ldB ]", fno.fsize);

		safePrint(50, " %s", *fno.lfname ? fno.lfname : fno.fname);
	}
	f_closedir(&dir);

	vPortFree(lfn);
	strncpy((char*)outBuffer, "", outBufferLen);
	return pdFALSE;
}

portBASE_TYPE radioCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command) {
	char *param[1]; // oOfParams is checked by OS - must be 1
	sliceCommand((char*)command, param, 1);

	AsyncCall_Type call = {
		.Type = AsyncCallProc_Void,
	};

	if (cmatch("on", param[0], 2)) { // on
		call.CallVoid = radioEnable;
	} else if (cmatch("off", param[0], 2)) { // of
		call.CallVoid = radioDisable;
	} else if (cmatch("test", param[0], 1)) { // t
		call.CallVoid = radioTestCommand;
	} else if (cmatch("reset", param[0], 1)) { // r
		call.CallVoid = radioResetCommand;
	} else if (cmatch("indicator", param[0], 1)) { // i
		call.CallVoid = radioResetIndicatorCommand;
	} else {
		strncpy((char*)outBuffer, "Invalid option provided\n", outBufferLen);
		return pdFALSE;
	}

	xQueueSendToBack(AsyncCallHandlerQueue, &call, portMAX_DELAY);
	strncpy((char*)outBuffer, "Scheduled radio command\n", outBufferLen);
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
	FreeRTOS_CLIRegisterCommand(&trajectoryComDef);
#ifdef DRIVE_COMMANDS
	FreeRTOS_CLIRegisterCommand(&driveComDef);
#endif
	FreeRTOS_CLIRegisterCommand(&loadComDef);
	FreeRTOS_CLIRegisterCommand(&catComDef);
	FreeRTOS_CLIRegisterCommand(&lsComDef);
	FreeRTOS_CLIRegisterCommand(&execComDef);
	FreeRTOS_CLIRegisterCommand(&radioComDef);
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
	CLILoadCompleteSemaphore = xSemaphoreCreateBinary();
}
