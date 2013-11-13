#include <string.h>
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS_CLI.h"

#include "priorities.h"
#include "stackSpace.h"
#include "hwinterface.h"
#include "main.h"

#include "TaskCLI.h"
#include "TaskPrintfConsumer.h"

#define CLI_INPUT_BUF_MAX_LEN 100

xTaskHandle CLITask;
xQueueHandle CLIInputQueue;

static const char* const welcomeMessage = "FreeRTOS command server.\r\nType \"help\" to view a list of registered commands.\n";
static const char* const promptMessage = "\nodin>";

static void registerAllCommands();
static portBASE_TYPE systemCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command);

static const CLI_Command_Definition_t systemComDef =
{
    (int8_t*)"system",
    (int8_t*)"system <reset|battery|cpu|stack|memory|aua>: System maintenance and diagnostic\n",
    systemCommand,
    1
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

void TaskCLIConstructor() {
	xTaskCreate(TaskCLI, NULL, TASKCLI_STACKSPACE, NULL, PRIORITY_TASK_CLI, &CLITask);
	CLIInputQueue = xQueueCreate(configCOMMAND_INT_MAX_OUTPUT_SIZE, sizeof(char*));
}
