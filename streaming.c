#include <stm32f4xx.h>
#include "FreeRTOS.h"
#include "semphr.h"

#include "streaming.h"
#include "hardware.h"

static xSemaphoreHandle wifiStreamDMAMutex = NULL;
static bool wifiStreamCleared = true;
static void (*finishTransmissionUserFunction)(void*) = NULL;
static void *finishTransmissionUserParam = NULL;

bool streamAcquire(portBASE_TYPE blockTime) {
	if (wifiStreamDMAMutex == NULL)
		wifiStreamDMAMutex = xSemaphoreCreateMutex();

	return xSemaphoreTake(wifiStreamDMAMutex, blockTime) == pdTRUE;
}

bool streamRelease() {
	if (wifiStreamCleared) {
		xSemaphoreGive(wifiStreamDMAMutex);
		return true;
	}
	else return false;
}

void streamStartTransmission(void *dest, uint32_t bytes, void (*finishHandle)(void*), void *finishParam) {
	finishTransmissionUserFunction = finishHandle;
	finishTransmissionUserParam = finishParam;

	WIFI_RX_DMA_STREAM->M0AR = (uint32_t)dest; 				// base address
	WIFI_RX_DMA_STREAM->NDTR = bytes;						// number of bytes to transfer

    USART_ITConfig(WIFI_USART, USART_IT_RXNE, DISABLE);		// disable interrupts from USART, disconnect Command Handling
    DMA_Cmd(WIFI_RX_DMA_STREAM, ENABLE);					// enable DMA stream
    USART_DMACmd(WIFI_USART, USART_DMAReq_Rx, ENABLE);		// set USART to DMA
}

void streamFinishTransmission(void) {
	// disable DMA handling in USART and enable receive interrupt allowing tasks to receive
	USART_DMACmd(WIFI_USART, USART_DMAReq_Rx, DISABLE);
	USART_ITConfig(WIFI_USART, USART_IT_RXNE, ENABLE);

	wifiStreamCleared = true;
}

/* ISR */
void StreamDMAWifiFinish(void) {
	/* Disable DMA stream */
	DMA_Cmd(WIFI_RX_DMA_STREAM, DISABLE);
	DMA_ClearFlag(WIFI_RX_DMA_STREAM, WIFI_RX_DMA_FLAG_TCIF); 	// clear flag here because it must be cleared before next DMA transfer is configured
	while (WIFI_RX_DMA_STREAM->CR & DMA_SxCR_EN);				// wait for the stream to be fully off

	if (finishTransmissionUserFunction)
		finishTransmissionUserFunction(finishTransmissionUserParam);
}
