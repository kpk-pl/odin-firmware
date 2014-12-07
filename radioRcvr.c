#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include <stm32f4xx.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "timers.h"

#include "hardware.h"
#include "radioRcvr.h"

#include "TaskPrintfConsumer.h"
#include "TaskTelemetry.h"

#define RADIO_BUFFER_LEN 20

#define RADIO_MSG_TYPE_READ_TELEMETRY	'U'

static volatile uint8_t radioIncommingBuffer[RADIO_BUFFER_LEN+1];
static volatile uint8_t radioOutgoingBuffer[RADIO_BUFFER_LEN+1];
static volatile uint8_t radioTransmitCounter = 0;
static volatile uint8_t radioReceiveCounter = 0;

static xSemaphoreHandle radioCommMutex = NULL;
static xSemaphoreHandle radioTransmissionCompletedSemaphore = NULL;

static void memvcpy(volatile void *to, volatile const void *from, const size_t length);

///////////// GLOBALLY ACCESSIBLE FUNCTIONS ////////////////

void radioSetup() {
	if (!radioCommMutex) {
		radioCommMutex = xSemaphoreCreateMutex();
	}
	if (!radioTransmissionCompletedSemaphore) {
		radioTransmissionCompletedSemaphore = xSemaphoreCreateBinary();
	}
}

void radioDisable() {
}

void radioEnable() {
}

void radioTestCommand() {
	xSemaphoreTake(radioCommMutex, portMAX_DELAY);
	memvcpy(radioOutgoingBuffer, "T", 2);
	radioTransmitCounter = 0;
	USART_ITConfig(RADIO_USART, USART_IT_TXE, ENABLE); // now interrupt will fire immediately
	xSemaphoreTake(radioTransmissionCompletedSemaphore, portMAX_DELAY);
	xSemaphoreGive(radioCommMutex);
}


void radioResetIndicatorCommand() {
}

void radioResetCommand() {
}

///////////// INTERNAL FUNCTIONS //////////////////////

static void memvcpy(volatile void *to, volatile const void *from, const size_t length) {
	size_t i;
	volatile const uint8_t *_from = (uint8_t*)from;
	volatile uint8_t *_to = (uint8_t*)to;
	for(i = 0; i<length; ++i) {
		_to[i] = _from[i];
	}
}

void RadioUSARTInterrupt() {
	portBASE_TYPE contextSwitch = pdFALSE;
	static bool command = false;

	if (USART_GetITStatus(RADIO_USART, USART_IT_RXNE) == SET) {
		uint8_t in = USART_ReceiveData(RADIO_USART); // this clears the IT flag
		if (!command && in == '<') {
			command = true;
		} else if (command && in == '>') {
			command = false;
			radioReceiveCounter = 0;
		} else if (command && radioReceiveCounter < sizeof(radioIncommingBuffer)) {
			radioIncommingBuffer[radioReceiveCounter++] = in;
		}

		switch (radioIncommingBuffer[0]) {
		case RADIO_MSG_TYPE_READ_TELEMETRY:
			if (radioReceiveCounter == 13) {
				TelemetryUpdate_Struct update = (TelemetryUpdate_Struct) {
					.Source = TelemetryUpdate_Source_Camera,
					.Timestamp = xTaskGetTickCountFromISR()
				};
				memvcpy(&update.Data, radioIncommingBuffer+1, 12);
				xQueueSendToBackFromISR(telemetryQueue, &update, &contextSwitch);
				radioReceiveCounter = 0;
			}
			break;
		default:
			radioReceiveCounter = 0; // discard unrecognized commands
		}
	}
	else if (USART_GetITStatus(RADIO_USART, USART_IT_TXE) == SET) {
		if (radioOutgoingBuffer[radioTransmitCounter] == '\0') {
			USART_ITConfig(RADIO_USART, USART_IT_TXE, DISABLE);
			xSemaphoreGiveFromISR(radioTransmissionCompletedSemaphore, &contextSwitch);
		}
		else {
			USART_SendData(RADIO_USART, radioOutgoingBuffer[radioTransmitCounter++]); // this clears the IT flag
		}
	}

	portEND_SWITCHING_ISR(contextSwitch);
}
