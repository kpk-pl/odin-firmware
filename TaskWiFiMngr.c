#include <string.h>

#include "stackSpace.h"
#include "priorities.h"
#include "hwinterface.h"

#include "TaskWiFiMngr.h"
#include "TaskPrintfConsumer.h"

#define WAIT_FOR_RESPONSE_TIME 1000

xTaskHandle WiFiMngrTask = NULL;
xQueueHandle WiFiMngrInputQueue = NULL;

static char* receiveResponseLine(uint32_t waitTime);
static bool wifiTransaction(const char *command, const char *response, uint32_t waitTime);

void TaskWiFiMngr(WiFiMngr_Command_Type *cmd) {
	safePrint(30, "WiFiMngr fired up\n");
	vTaskDelay(500/portTICK_RATE_MS);

	setWiFiMode(WiFiMode_Command);
	vTaskDelay(500/portTICK_RATE_MS);

	bool ret = wifiTransaction("AT\n", "[OK]", 500);

	setWiFiMode(WiFiMode_Data);
	vTaskDelay(500/portTICK_RATE_MS);

	if (ret) {
		safePrint(30, "Receive success");
	}
	else {
		safePrint(30, "Receive failed\n");
	}

	TaskWiFiMngrDestructor();
}

char* receiveResponseLine(uint32_t waitTime) {
	char *msg = NULL;

	/* Receive command echo */
	xQueueReceive(WiFiMngrInputQueue, &msg, waitTime/portTICK_RATE_MS);
	if (msg) {
		vPortFree(msg);
		msg = NULL;
	}
	else {
		return NULL;
	}

	/* Receive new line */
	xQueueReceive(WiFiMngrInputQueue, &msg, waitTime/portTICK_RATE_MS);
	if (msg) {
		vPortFree(msg);
		msg = NULL;
	}
	else {
		return NULL;
	}

	/* Receive actual output */
	xQueueReceive(WiFiMngrInputQueue, &msg, waitTime/portTICK_RATE_MS);
	return msg;
}

bool wifiTransaction(const char *command, const char *response, uint32_t waitTime) {
	if (pdPASS != xQueueReset(WiFiMngrInputQueue))
		return false;

	safePrint(50, "%s", command);

	char *resp = receiveResponseLine(waitTime);

	bool ok = false;
	if (strncmp(resp, response, strlen(response)) == 0) {
		ok = true;
	}

	if (resp)
		vPortFree(resp);

	return ok;
}

bool TaskWiFiMngrConstructor(WiFiMngr_Command_Type command) {
	static WiFiMngr_Command_Type param;

	/* Task already exists */
	if (WiFiMngrTask != NULL || WiFiMngrInputQueue != NULL)
		return false;

	/* Create input queue for task */
	WiFiMngrInputQueue = xQueueCreate(10, sizeof(char*));
	if (WiFiMngrInputQueue == NULL)
		return false;

	param = command;
	/* Create task */
	if (xTaskCreate((void(*)(void*))TaskWiFiMngr, NULL, TASKWIFIMNGR_STACKSPACE,
			(void*)&param, PRIORITY_TASK_WIFIMNGR, &WiFiMngrTask) != pdPASS) {
		vQueueDelete(WiFiMngrInputQueue);
		return false;
	}

	return true;
}

void TaskWiFiMngrDestructor() {
	if (WiFiMngrInputQueue != NULL)
		vQueueDelete(WiFiMngrInputQueue);
	WiFiMngrInputQueue = NULL;

	if (WiFiMngrTask != NULL) {
		xTaskHandle thisTask = WiFiMngrTask;
		WiFiMngrTask = NULL;
		vTaskDelete(thisTask);
	}
}
