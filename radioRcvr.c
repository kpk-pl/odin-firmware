#include <stdbool.h>
#include <string.h>

#include <stm32f4xx.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "timers.h"

#include "hardware.h"
#include "radioRcvr.h"

#include "TaskPrintfConsumer.h"
#include "TaskTelemetry.h"

#define RADIO_BUFFER_LEN 20

#define RADIO_MSG_TYPE_READ_RADIO_BUFFER 	'R'
#define RADIO_MSG_TYPE_CONST_0xE5 			'c'
#define RADIO_MSG_TYPE_RADIO_OFF 			'o'
#define RADIO_MSG_TYPE_RADIO_ON 			'O'
#define RADIO_MSG_TYPE_RESET				'x'
#define RADIO_MSG_TYPE_RESET_INDICATOR		'i'

static volatile uint8_t radioIncommingBuffer[RADIO_BUFFER_LEN];
static volatile uint8_t radioOutgoingBuffer[RADIO_BUFFER_LEN];
static volatile uint8_t radioTransmitCounter = 0;
static volatile uint8_t radioTransactionLength = 0;
static volatile bool isBlockingTransaction = false;

static xSemaphoreHandle radioSPIMutex = NULL;

///////////// GLOBALLY ACCESSIBLE FUNCTIONS ////////////////

void radioSetup() {
	if (!radioSPIMutex) {
		radioSPIMutex = xSemaphoreCreateMutex();
	}
}

void radioDisable() {
	xSemaphoreTake(radioSPIMutex, portMAX_DELAY);

	xSemaphoreGive(radioSPIMutex);
}

void radioEnable() {
	xSemaphoreTake(radioSPIMutex, portMAX_DELAY);

	xSemaphoreGive(radioSPIMutex);
}

void radioTestCommand() {
	xSemaphoreTake(radioSPIMutex, portMAX_DELAY);

	xSemaphoreGive(radioSPIMutex);
}


void radioResetIndicatorCommand() {
	xSemaphoreTake(radioSPIMutex, portMAX_DELAY);

	xSemaphoreGive(radioSPIMutex);
}

void radioResetCommand() {
	xSemaphoreTake(radioSPIMutex, portMAX_DELAY);

	xSemaphoreGive(radioSPIMutex);
}

///////////// INTERNAL FUNCTIONS //////////////////////

void RadioUSARTInterrupt() {
	portBASE_TYPE contextSwitch = pdFALSE;

	if (USART_GetITStatus(RADIO_USART, USART_IT_RXNE) == SET) {
		uint8_t in = USART_ReceiveData(RADIO_USART);
		// so something
		USART_ClearFlag(WIFI_USART, USART_FLAG_RXNE);
	}
	else if (USART_GetITStatus(RADIO_USART, USART_IT_TC) == SET) {
		//xSemaphoreGiveFromISR(wifiUSARTTCSemaphore, &contextSwitch);
		// do something
		USART_ITConfig(RADIO_USART, USART_IT_TC, DISABLE);
		USART_ClearFlag(RADIO_USART, USART_FLAG_TC);
	}

	portEND_SWITCHING_ISR(contextSwitch);
}

void RadioDMATxCompleteInterrupt() {
	portBASE_TYPE contextSwitch = pdFALSE;
	//xSemaphoreGiveFromISR(wifiDMATCSemaphore, &contextSwitch);
	DMA_ClearFlag(RADIO_DMA_TX_STREAM, RADIO_DMA_TX_FLAG_TCIF);
	portEND_SWITCHING_ISR(contextSwitch);
}
