#include <stm32f4xx.h>
#include <stdio.h>

#include "TaskPrintfConsumer.h"
#include "main.h"
#include "hwinterface.h"
#include "priorities.h"
#include "stackSpace.h"

xTaskHandle printfConsumerTask;			/*!< This task handle */
xQueueHandle printfQueue;				/*!< Queue for data to be printed out from safePrint functions */
xSemaphoreHandle comUSARTTCSemaphore;	/*!< Transmission complete from USB-UART converter UART */
xSemaphoreHandle comDMATCSemaphore;		/*!< DMA transfer complete from USB-UART converter UART */
xSemaphoreHandle wifiUSARTTCSemaphore;	/*!< Transmission complete from WiFi module UART */
xSemaphoreHandle wifiDMATCSemaphore;	/*!< DMA transfer complete from WiFi module UART */
xSemaphoreHandle printfMutex;			/*!< Mutex that needs to be acquired to control printing on all interfaces */

void TaskPrintfConsumer(void * p) {
	char *msg;

	while(1) {
		/* Turn off printing if bridge is on */
		if (getWiFi2USBBridgeStatus() == ON) {
			vTaskDelay(100/portTICK_RATE_MS);
			continue;
		}

		/* Wait for new message, don't take it, process any incoming message as quickly as possible */
		xQueuePeek(printfQueue, &msg, portMAX_DELAY);

		/* Discard any messages coming when bridge is on */
		if (getWiFi2USBBridgeStatus() == ON) {
			continue;
		}

		/* Take message from queue */
		xQueueReceive(printfQueue, &msg, portMAX_DELAY);

		/* Acquire resource */
		xSemaphoreTake(printfMutex, portMAX_DELAY);

		/* Set start address at the beginning of the message */
		COM_TX_DMA_STREAM->M0AR = (uint32_t)msg; // base addr
		WIFI_TX_DMA_STREAM->M0AR = (uint32_t)msg;
		/* Set number of bytes to send */
		size_t len = strlen(msg);
		COM_TX_DMA_STREAM->NDTR = len;   // size in bytes
		WIFI_TX_DMA_STREAM->NDTR = len;

		/* Save USB and WiFi status, will be used in two different places and need to be the same */
		OnOff USBS = getUSBStatus();
		OnOff WIFIS = getWiFiStatus();

		/* If USB is enabled, then start DMA transfer */
		if (USBS == ON) {
			/* Enable stream */
			DMA_Cmd(COM_TX_DMA_STREAM, ENABLE);
			/* Enable DMA in USART */
			USART_DMACmd(COM_USART, USART_DMAReq_Tx, ENABLE);
			/* Enabling interrupt from Transmission Complete flag.
			 * Here, because it it set constantly when USART is idle.
			 * Disabled in ISR. */
			USART_ITConfig(COM_USART, USART_IT_TC, ENABLE);
		}
		/* The same for WiFi */
		if (WIFIS == ON) {
			DMA_Cmd(WIFI_TX_DMA_STREAM, ENABLE);
			USART_DMACmd(WIFI_USART, USART_DMAReq_Tx, ENABLE);
			USART_ITConfig(WIFI_USART, USART_IT_TC, ENABLE);
		}

		/* Waiting for transmission complete. One interrupt from DMA and one from USART. Disabling stream at the end */
		if (USBS == ON) {
			xSemaphoreTake(comUSARTTCSemaphore, portMAX_DELAY);
			xSemaphoreTake(comDMATCSemaphore, portMAX_DELAY);
			DMA_Cmd(COM_TX_DMA_STREAM, DISABLE);
		}
		if (WIFIS == ON) {
			xSemaphoreTake(wifiUSARTTCSemaphore, portMAX_DELAY);
			xSemaphoreTake(wifiDMATCSemaphore, portMAX_DELAY);
			DMA_Cmd(WIFI_TX_DMA_STREAM, DISABLE);
		}

		/* Free memory allocated for message */
		vPortFree(msg);

		/* Five resources back */
		xSemaphoreGive(printfMutex);
	}
}

void TaskPrintfConsumerConstructor() {
	xTaskCreate(TaskPrintfConsumer, NULL, TASKPRINTFCONSUMER_STACKSPACE, NULL, PRIORITY_TASK_PRINTFCONSUMER, &printfConsumerTask);
	printfQueue = xQueueCreate(50, sizeof(char*));
	comUSARTTCSemaphore = xSemaphoreCreateBinary();
	comDMATCSemaphore = xSemaphoreCreateBinary();
	wifiUSARTTCSemaphore = xSemaphoreCreateBinary();
	wifiDMATCSemaphore = xSemaphoreCreateBinary();
	printfMutex = xSemaphoreCreateMutex();
}

int safePrint(const size_t length, const char *format, ...) {
	if (length <= 0) return 0;
	va_list arglist;
	va_start(arglist, format);
	char *pbuf = (char*)pvPortMalloc(length*sizeof(char));
	int ret = vsnprintf(pbuf, length, format, arglist);
	if (ret > 0) {
		if (xQueueSendToBack(printfQueue, &pbuf, 0) == errQUEUE_FULL) {
			vPortFree(pbuf);
			lightLED(5, ON);
		}
	}
	else {
		vPortFree(pbuf);
	}
	va_end(arglist);
	return ret;
}

int safePrintFromISR(const size_t length, const char *format, ...) {
	va_list arglist;
	va_start(arglist, format);
	char *pbuf = (char*)pvPortMalloc(length*sizeof(char));
	int ret = vsnprintf(pbuf, length, format, arglist);
	portBASE_TYPE contextSwitch = pdFALSE;
	if (xQueueSendToBackFromISR(printfQueue, &pbuf, &contextSwitch) == errQUEUE_FULL) {
		lightLED(5, ON);
		vPortFree(pbuf);
	}
	va_end(arglist);
	portEND_SWITCHING_ISR(contextSwitch);
	return ret;
}

/* ISR for COM DMA Tx */
void COMDMANotify(void) {
	portBASE_TYPE contextSwitch = pdFALSE;
	xSemaphoreGiveFromISR(comDMATCSemaphore, &contextSwitch);
	DMA_ClearFlag(COM_TX_DMA_STREAM, COM_TX_DMA_FLAG_TCIF);
	portEND_SWITCHING_ISR(contextSwitch);
}

/* ISR for WIFI DMA Tx */
void WiFiDMANotify() {
	portBASE_TYPE contextSwitch = pdFALSE;
	xSemaphoreGiveFromISR(wifiDMATCSemaphore, &contextSwitch);
	DMA_ClearFlag(WIFI_TX_DMA_STREAM, WIFI_TX_DMA_FLAG_TCIF);
	portEND_SWITCHING_ISR(contextSwitch);
}
