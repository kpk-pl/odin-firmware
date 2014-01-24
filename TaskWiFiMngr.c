#include <string.h>

#include "stackSpace.h"
#include "priorities.h"
#include "hwinterface.h"

#include "TaskWiFiMngr.h"
#include "TaskPrintfConsumer.h"

xTaskHandle WiFiMngrTask = NULL;
xQueueHandle WiFiMngrInputQueue = NULL;

static char* receiveResponseLine(uint32_t waitTime, uint8_t ignoreLines);
static bool wifiTransaction(char *command, const char *response, uint8_t ignoreLinesResp, uint32_t waitTime);

void TaskWiFiMngr(WiFiMngr_Command_Type *cmd) {
	/* WiFi must be turned on for that */
	if (getWiFiStatus() == OFF) {
		safePrint(31, "Error: WiFi module turned off\n");
		TaskWiFiMngrDestructor();
	}

	/* Acquire resources for printing */
	xSemaphoreTake(printfMutex, portMAX_DELAY);

	/* Switch WiFi module to command input */
	setWiFiMode(WiFiMode_Command);
	vTaskDelay(200/portTICK_RATE_MS);

	/* Dummy transaction, might fail - to clean internal WiFi module buffer */
	wifiTransaction("AT\n", "", 2, 500);

	bool ok = false;
	do {
		if (!wifiTransaction("AT+WD\n", "[OK]", 2, 500)) break;
		if (!wifiTransaction("AT+WAUTO=0,OdinWN\n", "[OK]", 2, 500)) break;
		if (!wifiTransaction("AT+WAUTH=2\n", "[OK]", 2, 500)) break;
		if (!wifiTransaction("AT+WWPA=OdinTheGod\n", "[OK]", 2, 500)) break;
		if (!wifiTransaction("AT+NDHCP=0\n", "[OK]", 2, 500)) break;
		if (!wifiTransaction("AT+NSET=192.168.50.2,255.255.255.0,192.168.50.1\n", "[OK]", 2, 500)) break;
		if (!wifiTransaction("AT+NAUTO=1,1,,4000\n", "[OK]", 2, 500)) break;
		if (!wifiTransaction("AT+XDUM=1\n", "[OK]", 2, 500)) break;
		if (!wifiTransaction("ATA\n", "[OK]", 4, 20000)) break;
		ok = true;
	} while(0);

	/* Go back to data mode */
	setWiFiMode(WiFiMode_Data);
	vTaskDelay(200/portTICK_RATE_MS);

	/* Release resources for printing */
	xSemaphoreGive(printfMutex);

	if (ok)
		safePrint(26, "WiFi module reconnected\n");
	else
		safePrint(33, "Error: WiFi module reconnection\n");

	/* Destroy this task and all resources */
	TaskWiFiMngrDestructor();
}

char* receiveResponseLine(uint32_t waitTime, uint8_t ignoreLines) {
	char *msg = NULL;

	while (ignoreLines-- > 0) {
		xQueueReceive(WiFiMngrInputQueue, &msg, waitTime/portTICK_RATE_MS);
		if (msg) {
			vPortFree(msg);
			msg = NULL;
		}
		else {
			return NULL;
		}
	}

	/* Receive actual output */
	xQueueReceive(WiFiMngrInputQueue, &msg, waitTime/portTICK_RATE_MS);
	return msg;
}

bool wifiTransaction(char *command, const char *response, uint8_t ignoreLinesResp, uint32_t waitTime) {
	if (pdPASS != xQueueReset(WiFiMngrInputQueue))
		return false;

	/* Length is set to big number to print all what is in the 'command' */
	printInterfaceBlocking(command, 10000, Interface_WiFi_Active);

	char *resp = receiveResponseLine(waitTime, ignoreLinesResp);

	bool ok = false;
	if (resp != NULL && strncmp(resp, response, strlen(response)) == 0) {
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
