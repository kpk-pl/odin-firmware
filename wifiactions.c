#include <stm32f4xx.h>
#include <string.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "wifiactions.h"
#include "hwinterface.h"

#include "TaskPrintfConsumer.h"

xQueueHandle WiFiActionsInputQueue = NULL;

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

void reconnectToAP(int turnOn) {
	bool turnedOn = false, error = false;

	WiFiActionsInputQueue = xQueueCreate(5, sizeof(char*));

	/* Acquire resources for printing */
	xSemaphoreTake(printfMutex, portMAX_DELAY);

	/* WiFi must be turned on for that */
	if (getWiFiStatus() == OFF) {
		enableWiFi(ENABLE);
		turnedOn = true;
	}

	/* Switch WiFi module to command input */
	setWiFiMode(WiFiMode_Command);
	vTaskDelay(200/portTICK_RATE_MS);

	/* Dummy transaction, might fail - to clean internal WiFi module buffer */
	wifiTransaction("\nAT\n", "", 2, 500);

	/* Actual transaction */
	const char *msgs[8] = {
		"AT+WD\n",
		"AT+WAUTO=0,OdinWN\n",
		"AT+WAUTH=2\n",
		"AT+WWPA=OdinTheGod\n",
		"AT+NDHCP=0\n",
		"AT+NSET=192.168.50.2,255.255.255.0,192.168.50.1\n",
		"AT+NAUTO=1,1,,4000\n",
		"AT+XDUM=1\n"
	};
	for (uint8_t i = 0; i < 8; ++i) {
		if (!wifiTransaction(msgs[i], "[OK]", 2, 500)) {
			error = true;
			break;
		}
	}
	if (!error) {
		if (!wifiTransaction("ATA\n", "[OK]", 4, 20000)) {
			error = true;
		}
	}

	/* Go back to data mode */
	setWiFiMode(WiFiMode_Data);
	vTaskDelay(200/portTICK_RATE_MS);

	/* Release resources for printing */
	xSemaphoreGive(printfMutex);

	/* Enable / disable WiFi according to previous state */
	if (turnedOn && !turnOn)
		enableWiFi(DISABLE);

	/* Check for errors */
	if (!error)
		safePrint(20, "[WiFi] Reconnected\n");
	else {
		lightLED(1, ON);
		safePrint(33, "[WiFi] Error: reconnect failed\n");
	}

	/* Clean up */
	vQueueDelete(WiFiActionsInputQueue);
	WiFiActionsInputQueue = NULL;
}

char* receiveResponseLine(uint32_t waitTime, uint8_t ignoreLines) {
	char *msg = NULL;

	while (ignoreLines-- > 0) {
		xQueueReceive(WiFiActionsInputQueue, &msg, waitTime/portTICK_RATE_MS);
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
	xQueueReceive(WiFiActionsInputQueue, &msg, waitTime/portTICK_RATE_MS);
	printInterfaceBlocking(msg, 1000, Interface_USB_Active);
	return msg;
}

bool wifiTransaction(const char *command, const char *response, uint8_t ignoreLinesResp, uint32_t waitTime) {
	if (pdPASS != xQueueReset(WiFiActionsInputQueue))
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

	vTaskDelay(100);
	return ok;
}
