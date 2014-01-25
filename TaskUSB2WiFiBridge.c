#include "TaskUSB2WiFiBridge.h"
#include "TaskInputMngr.h"
#include "main.h"
#include "priorities.h"
#include "stackSpace.h"

xTaskHandle USBWiFiBridgeTask = NULL;
xQueueHandle USB2WiFiBridgeQueue = NULL;

void TaskUSBWiFiBridge(void *p) {
	PrintInput_Struct input;

	while(1) {
		xQueueReceive(USB2WiFiBridgeQueue, &input, portMAX_DELAY);
		if (input.Source == PrintSource_Type_WiFi) {
			while (USART_GetFlagStatus(COM_USART, USART_FLAG_TXE) == RESET);
			USART_SendData(COM_USART, input.Input);
		}
		else if (input.Source == PrintSource_Type_USB){
			while (USART_GetFlagStatus(WIFI_USART, USART_FLAG_TXE) == RESET);
			USART_SendData(WIFI_USART, input.Input);
		}
	}
}

void TaskUSB2WiFiBridgeConstructor() {
	xTaskCreate(TaskUSBWiFiBridge, NULL, TASKBRIDGE_STACKSPACE, NULL, PRIOTITY_TASK_BRIDGE, &USBWiFiBridgeTask);
	USB2WiFiBridgeQueue = xQueueCreate(100, sizeof(PrintInput_Struct));
}

void TaskUSB2WiFiDestructor() {

}
