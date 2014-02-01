#include <string.h>

#include "stackSpace.h"
#include "priorities.h"
#include "hwinterface.h"
#include "hwinit.h"

#include "TaskWiFiMngr.h"
#include "TaskPrintfConsumer.h"

xTaskHandle WiFiMngrTask = NULL;
xQueueHandle WiFiMngrInputQueue = NULL;

/**
 * Receive selected line from response from WiFi module. Lines before selecred are ignored
 * @param waitTime Time to wait for a response in ms
 * @param ignoreLines How many lines to ignore before reading the one really needed
 * @retval Pointer with response. Must be freed afterwards
 */
static char* receiveResponseLine(uint32_t waitTime, uint8_t ignoreLines);

/**
 * Perform whole transaction with WiFi module. Send command and expect good response
 * @param command Command to be send
 * @param response Expected response (beginning of it). The actual response might be longer, but will be correctly recognized
 * @param ignoreLinesResp How many lines to ignore before reading actual output
 * @param waitTime Time to wait for response line
 * @retval true on success, false on error
 */
static bool wifiTransaction(const char *command, const char *response, uint8_t ignoreLinesResp, uint32_t waitTime);

/**
 * Performs reconnect action, setting up wifi connection
 * @retval true on success, false otherwise
 */
static bool actionReconnect();

/**
 * Sets the highest possible communication speed with WiFi module
 * @retval true on success, false otherwise
 */
static bool actionSetHighSpeed();

void TaskWiFiMngr(WiFiMngr_Command_Type *cmd) {
	bool turnedOn = false;

	/* WiFi must be turned on for that */
	if (getWiFiStatus() == OFF) {
		if (*cmd == WiFiMngr_Command_Reconnect) {
			enableWiFi(ENABLE);
			turnedOn = true;
		}
		else {
			safePrint(31, "Error: WiFi module turned off\n");
			TaskWiFiMngrDestructor();
		}
	}

	/* Acquire resources for printing */
	xSemaphoreTake(printfMutex, portMAX_DELAY);

	/* Switch WiFi module to command input */
	setWiFiMode(WiFiMode_Command);
	vTaskDelay(200/portTICK_RATE_MS);

	/* Dummy transaction, might fail - to clean internal WiFi module buffer */
	wifiTransaction("\nAT\n", "", 2, 500);

	bool ret;

	switch(*cmd) {
	case WiFiMngr_Command_Reconnect:
		ret = actionReconnect();
		break;
	case WiFiMngr_Command_SetHighSpeed:
		ret = actionSetHighSpeed();
		break;
	default:
		ret = false;
		break;
	}

	/* Go back to data mode */
	setWiFiMode(WiFiMode_Data);
	vTaskDelay(200/portTICK_RATE_MS);

	/* Release resources for printing */
	xSemaphoreGive(printfMutex);

	if (ret)
		safePrint(18, "OK: WiFi manager\n");
	else {
		if (turnedOn)
			enableWiFi(DISABLE);
		lightLED(1, ON);
		safePrint(21, "Error: WiFi manager\n");
	}

	/* Destroy this task and all resources */
	TaskWiFiMngrDestructor();
}

bool actionReconnect() {
	if (!wifiTransaction("AT+WD\n", "[OK]", 2, 500)) return false;
	if (!wifiTransaction("AT+WAUTO=0,OdinWN\n", "[OK]", 2, 500)) return false;
	if (!wifiTransaction("AT+WAUTH=2\n", "[OK]", 2, 500)) return false;
	if (!wifiTransaction("AT+WWPA=OdinTheGod\n", "[OK]", 2, 500)) return false;
	if (!wifiTransaction("AT+NDHCP=0\n", "[OK]", 2, 500)) return false;
	if (!wifiTransaction("AT+NSET=192.168.50.2,255.255.255.0,192.168.50.1\n", "[OK]", 2, 500)) return false;
	if (!wifiTransaction("AT+NAUTO=1,1,,4000\n", "[OK]", 2, 500)) return false;
	if (!wifiTransaction("AT+XDUM=1\n", "[OK]", 2, 500)) return false;
	if (!wifiTransaction("ATA\n", "[OK]", 4, 20000)) return false;
	return true;
}

bool actionSetHighSpeed() {
	printInterfaceBlocking("ATB=921600,8,n,1\n", 10000, Interface_WiFi_Active);
	vTaskDelay(200/portTICK_RATE_MS);
	InitializeWiFiUART(921600);
	return true;
}

char* receiveResponseLine(uint32_t waitTime, uint8_t ignoreLines) {
	char *msg = NULL;

	while (ignoreLines-- > 0) {
		xQueueReceive(WiFiMngrInputQueue, &msg, waitTime/portTICK_RATE_MS);
		printInterfaceBlocking(msg, 1000, Interface_USB_Active);
		if (msg) {
			if (strncmp(msg, "[ERROR", 6) == 0)
				return msg;
			vPortFree(msg);
			msg = NULL;
		}
		else {
			return NULL;
		}
	}

	/* Receive actual output */
	xQueueReceive(WiFiMngrInputQueue, &msg, waitTime/portTICK_RATE_MS);
	printInterfaceBlocking(msg, 1000, Interface_USB_Active);
	return msg;
}

bool wifiTransaction(const char *command, const char *response, uint8_t ignoreLinesResp, uint32_t waitTime) {
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
