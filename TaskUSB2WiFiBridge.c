#include "TaskUSB2WiFiBridge.h"
#include "main.h"
#include "priorities.h"
#include "stackSpace.h"

xTaskHandle USBWiFiBridgeTask;
xQueueHandle WiFi2USBBufferQueue = NULL;
xQueueHandle USB2WiFiBufferQueue = NULL;

void TaskUSBWiFiBridge(void *p) {
	WiFi2USBBufferQueue = xQueueCreate(50, sizeof(char));
	USB2WiFiBufferQueue = xQueueCreate(50, sizeof(char));

	xQueueSetHandle queueSet = xQueueCreateSet(100);
	xQueueAddToSet((xQueueSetMemberHandle)WiFi2USBBufferQueue, queueSet);
	xQueueAddToSet((xQueueSetMemberHandle)USB2WiFiBufferQueue, queueSet);

	char byte;

	while(1) {
		xQueueSetMemberHandle activeQueue = xQueueSelectFromSet(queueSet, portMAX_DELAY);
		if (activeQueue == WiFi2USBBufferQueue) {
			xQueueReceive(WiFi2USBBufferQueue, &byte, 0);
			while (USART_GetFlagStatus(COM_USART, USART_FLAG_TXE) == RESET);
			USART_SendData(COM_USART, byte);
		}
		else {
			xQueueReceive(USB2WiFiBufferQueue, &byte, 0);
			while (USART_GetFlagStatus(WIFI_USART, USART_FLAG_TXE) == RESET);
			USART_SendData(WIFI_USART, byte);
		}
	}
}

void TaskUSB2WiFiBridgeConstructor() {
	xTaskCreate(TaskUSBWiFiBridge, NULL, TASKBRIDGE_STACKSPACE, NULL, PRIOTITY_TASK_BRIDGE, &USBWiFiBridgeTask);
}
