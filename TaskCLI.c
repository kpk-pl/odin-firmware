#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS_CLI.h"

#include "priorities.h"
#include "stackSpace.h"
#include "hwinterface.h"
#include "main.h"

#include "TaskCLI.h"
#include "TaskPrintfConsumer.h"

#define CLI_INPUT_BUF_MAX_LEN 200

xTaskHandle CLITask;
xQueueHandle CLIInputQueue;

static const char* const welcomeMessage = "FreeRTOS command server.\r\nType \"help\" to view a list of registered commands.\n";
static const char* const promptMessage = "\nodin>";

static void registerAllCommands();
static portBASE_TYPE systemCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command);
static portBASE_TYPE lanternCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command);
static portBASE_TYPE delayCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command);
static portBASE_TYPE penCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command);
static portBASE_TYPE telemetryCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command);
static portBASE_TYPE motorCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command);
static portBASE_TYPE wifiCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command);
static portBASE_TYPE logCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command);
static portBASE_TYPE trajectoryCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command);

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
    (int8_t*)"telemetry ...\n"
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
    		 "\tregulator <enable|disable|<params #P #I #D>>\n"
    		 "\tencoder [left|right]\n"
    		 "\tenable|disable\n"
    		 "\tbrake\n",
    motorCommand,
    -1
};
static const CLI_Command_Definition_t wifiComDef =
{
    (int8_t*)"wifi",
    (int8_t*)"wifi <reset|<set <command|data>>>",
    wifiCommand,
    -1
};
static const CLI_Command_Definition_t logComDef =
{
    (int8_t*)"log",
    (int8_t*)"log ...\n"
    		 "\toff\n"
    		 "\tall\n"
    		 "\t <event|telemetry|speed|imu> [off]\n"
    		 "\t motor something here\n",
    logCommand,
    -1
};
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

void TaskCLI(void *p) {
	portBASE_TYPE moreDataComing;
	char * msg;
	static char outputString[CLI_INPUT_BUF_MAX_LEN];

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
			moreDataComing = FreeRTOS_CLIProcessCommand((int8_t*)msg, (int8_t*)outputString, CLI_INPUT_BUF_MAX_LEN);
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
	FreeRTOS_CLIRegisterCommand(&trajectoryComDef);
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
		strncpy((char*)outBuffer, "Incorrect command parameter(s).  Enter \"help\" to view a list of available commands.\r\n", outBufferLen);
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
		strncpy((char*)outBuffer, "Incorrect command parameter(s).  Enter \"help\" to view a list of available commands.\r\n", outBufferLen);
	}

	return pdFALSE;
}

portBASE_TYPE delayCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command) {
	char* param;
	portBASE_TYPE paramLen;

	param = (char*)FreeRTOS_CLIGetParameter(command, 1, &paramLen);
	param[paramLen] = '\0';

	vTaskDelay(strtol(param, NULL, 10) / portTICK_RATE_MS);

	snprintf((char*)outBuffer, outBufferLen, "Delayed %s milliseconds\n", param);
	return pdFALSE;
}

portBASE_TYPE penCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command) {
	strncpy((char*)outBuffer, "Incorrect command parameter(s).  Enter \"help\" to view a list of available commands.\r\n", outBufferLen);
	return pdFALSE;
}

portBASE_TYPE telemetryCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command) {
	strncpy((char*)outBuffer, "Incorrect command parameter(s).  Enter \"help\" to view a list of available commands.\r\n", outBufferLen);
	return pdFALSE;
}

portBASE_TYPE motorCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command) {
	strncpy((char*)outBuffer, "Incorrect command parameter(s).  Enter \"help\" to view a list of available commands.\r\n", outBufferLen);
	return pdFALSE;
}

portBASE_TYPE wifiCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command) {
	strncpy((char*)outBuffer, "Incorrect command parameter(s).  Enter \"help\" to view a list of available commands.\r\n", outBufferLen);
	return pdFALSE;
}

portBASE_TYPE logCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command) {
	strncpy((char*)outBuffer, "Incorrect command parameter(s).  Enter \"help\" to view a list of available commands.\r\n", outBufferLen);
	return pdFALSE;
}

portBASE_TYPE trajectoryCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command) {
	strncpy((char*)outBuffer, "Incorrect command parameter(s).  Enter \"help\" to view a list of available commands.\r\n", outBufferLen);
	return pdFALSE;
}

void TaskCLIConstructor() {
	xTaskCreate(TaskCLI, NULL, TASKCLI_STACKSPACE, NULL, PRIORITY_TASK_CLI, &CLITask);
	CLIInputQueue = xQueueCreate(configCOMMAND_INT_MAX_OUTPUT_SIZE, sizeof(char*));
}
