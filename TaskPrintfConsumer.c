#include <stm32f4xx.h>

#include "TaskPrintfConsumer.h"
#include "main.h"
#include "hwinterface.h"

void TaskPrintfConsumer(void * p) {
	char *msg;

	xSemaphoreTake(comUSARTTCSemaphore, 0);
	xSemaphoreTake(comDMATCSemaphore, 0);
	xSemaphoreTake(wifiUSARTTCSemaphore, 0);
	xSemaphoreTake(wifiDMATCSemaphore, 0);

	while(1) {
		/* Turn off printing if bridge is on */
		if (getWiFi2USBBridgeStatus() == ON) {
			vTaskDelay(100/portTICK_RATE_MS);
			continue;
		}

		/* Wait for new message and take it, process any incomming message as quickly as possible */
		xQueueReceive(printfQueue, &msg, portMAX_DELAY);

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
	}
}
