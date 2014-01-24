#include "TaskInputMngr.h"
#include "main.h"
#include "priorities.h"
#include "stackSpace.h"
#include "hwinterface.h"
#include "TaskPrintfConsumer.h"
#include "TaskUSB2WiFiBridge.h"
#include "TaskCommandHandler.h"
#include "TaskCLI.h"
#include "TaskWiFiMngr.h"

#define BUF_RX_LEN 100				/*!< Maximum length of UART command */

#define INDEX_USB 0
#define INDEX_WIFI 1

xTaskHandle commInputMngrTask;	/*!< This task handle */
xQueueHandle commInputMngrQueue;	/*!< Queue for incoming data that was received in ISR's */

void TaskInputMngr(void * p) {
	PrintInput_Struct newInput;

	bool incoming[2] = {false, false}; // state machine
	char RXBUF[2][BUF_RX_LEN+1];
	uint8_t RXBUFPOS[2] = {0, 0};
	uint8_t i;

	vTaskDelay(500/portTICK_RATE_MS);
	while (xQueueReceive(commInputMngrQueue, &newInput, 0) == pdTRUE);

	while(1) {
		/* Block until new command is available */
		xQueueReceive(commInputMngrQueue, &newInput, portMAX_DELAY);

		/* Check source */
		switch (newInput.Source) {
		case PrintSource_Type_USB:
			i = INDEX_USB;
			break;
		case PrintSource_Type_WiFi:
			i = INDEX_WIFI;
			break;
		default: break;
		}

		if (!globalUsingCLI && newInput.Input == '<') {  // msg begin character
			RXBUFPOS[i] = 0;
			incoming[i] = true;
		}
		else if ((!globalUsingCLI && newInput.Input == '>') || (globalUsingCLI && newInput.Input == '\n')) {
			if (incoming[i] || globalUsingCLI) { // if not using CLI then do this only in in the right state
				RXBUF[i][RXBUFPOS[i]++] = '\0';
				incoming[i] = false;

				char * ptr = (char *)pvPortMalloc(RXBUFPOS[i]*sizeof(char));
				strcpy(ptr, (const char*)RXBUF[i]);

				RXBUFPOS[i] = 0;

				portBASE_TYPE send;
				if (i == INDEX_WIFI && getWiFiMode() == WiFiMode_Command && WiFiMngrInputQueue != NULL) {
					send = xQueueSendToBack(WiFiMngrInputQueue, &ptr, 0);
				}
				else if (globalUsingCLI) {
					send = xQueueSendToBack(CLIInputQueue, &ptr, 0);
				}
				else {
					send = xQueueSendToBack(commandQueue, &ptr, 0);
				}

				if (send == errQUEUE_FULL) {
					vPortFree(ptr);
				}
			}
		}
		else {
			if (incoming[i] || globalUsingCLI) {
				if (RXBUFPOS[i] < BUF_RX_LEN)
					RXBUF[i][RXBUFPOS[i]++] = newInput.Input;
			}
		}
	}
}

void TaskInputMngrConstructor() {
	xTaskCreate(TaskInputMngr, NULL, TASKINPUTBUFFER_STACKSPACE, NULL, PRIOTITY_TASK_INPUTBUFFER, &commInputMngrTask);
	commInputMngrQueue = xQueueCreate(100, sizeof(PrintInput_Struct));
}

/* ISR for COM USART - must be called in USARTx_IRQHandler */
void COMAction(void) {
	portBASE_TYPE contextSwitch = pdFALSE;
	PrintInput_Struct in = {.Source = PrintSource_Type_USB};

	if (USART_GetITStatus(COM_USART, USART_IT_RXNE) == SET) {
		in.Input = USART_ReceiveData(COM_USART);
		if (getWiFi2USBBridgeStatus() == ON) {
			xQueueSendToBackFromISR(USB2WiFiBufferQueue, &in.Input, &contextSwitch);
		}
		else {
			xQueueSendToBackFromISR(commInputMngrQueue, &in, &contextSwitch);
		}
		USART_ClearFlag(COM_USART, USART_FLAG_RXNE);
	}
	else if (USART_GetITStatus(COM_USART, USART_IT_TC) == SET) {
		xSemaphoreGiveFromISR(comUSARTTCSemaphore, &contextSwitch);
		USART_ITConfig(COM_USART, USART_IT_TC, DISABLE);
		USART_ClearFlag(COM_USART, USART_FLAG_TC);
	}

	portEND_SWITCHING_ISR(contextSwitch);
}

/* ISR for WIFI Rx interrupt */
void WIFIAction(void) {
	portBASE_TYPE contextSwitch = pdFALSE;
	PrintInput_Struct in = {.Source = PrintSource_Type_WiFi};

	if (USART_GetITStatus(WIFI_USART, USART_IT_RXNE) == SET) {
		in.Input = USART_ReceiveData(WIFI_USART);
		if (getWiFi2USBBridgeStatus() == ON) {
			xQueueSendToBackFromISR(WiFi2USBBufferQueue, &in.Input, &contextSwitch);
		}
		else {
			xQueueSendToBackFromISR(commInputMngrQueue, &in, &contextSwitch);
		}
		USART_ClearFlag(WIFI_USART, USART_FLAG_RXNE);
	}
	else if (USART_GetITStatus(WIFI_USART, USART_IT_TC) == SET) {
		xSemaphoreGiveFromISR(wifiUSARTTCSemaphore, &contextSwitch);
		USART_ITConfig(WIFI_USART, USART_IT_TC, DISABLE);
		USART_ClearFlag(WIFI_USART, USART_FLAG_TC);
	}

	portEND_SWITCHING_ISR(contextSwitch);
}
