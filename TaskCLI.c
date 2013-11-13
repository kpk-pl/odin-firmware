#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOS_CLI.h"

#include "priorities.h"
#include "stackSpace.h"

#include "TaskCLI.h"
#include "TaskPrintfConsumer.h"

xTaskHandle CLITask;
xQueueHandle CLIInputQueue;

#define MAX_OUTPUT_LENGTH   100

static const char* const pcWelcomeMessage =
  "FreeRTOS command server.\r\nType Help to view a list of registered commands.\r\n";

static void registerAllCommands();
static portBASE_TYPE systemCommand(int8_t* outBuffer, size_t outBufferLen, const int8_t* command);

static const CLI_Command_Definition_t systemComDef =
{
    (int8_t*)"system",
    (int8_t*)"system <command>: Deletes <filename> from the disk\r\n",
    systemCommand,
    1
};

void TaskCLI(void *p) {
	portBASE_TYPE moreDataComing;
	char * msg;
	static char outputString[MAX_OUTPUT_LENGTH];

	registerAllCommands();
	safePrint(strlen(pcWelcomeMessage)+1, "%s", pcWelcomeMessage);

    while(1) {
		/* Block till message is available */
		xQueueReceive(CLIInputQueue, &msg, portMAX_DELAY);

		/* Print new line after command received */
		safePrint(2, "\n");

		/* Process command and print as many lines as necessary */
		do {
			moreDataComing = FreeRTOS_CLIProcessCommand((int8_t*)msg, (int8_t*)outputString, MAX_OUTPUT_LENGTH);
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
		safePrint(5, "aua\n");
	}
	else if (strcmp(param, "memory") == 0) {
		safePrint(8, "memory\n");
	}
	else if (strcmp(param, "reset") == 0) {
		safePrint(7, "reset\n");
	}
	else if (strcmp(param, "battery") == 0) {
		safePrint(9, "battery\n");
	}
	else if (strcmp(param, "cpu") == 0) {
		safePrint(5, "cpu\n");
	}
	else if (strcmp(param, "stack") == 0) {
		safePrint(7, "stack\n");
	}

	return pdFALSE;
}

void TaskCLIConstructor() {
	xTaskCreate(TaskCLI, NULL, TASKCLI_STACKSPACE, NULL, PRIORITY_TASK_CLI, &CLITask);
	CLIInputQueue = xQueueCreate(configCOMMAND_INT_MAX_OUTPUT_SIZE, sizeof(char*));
}
