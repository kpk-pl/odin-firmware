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

static volatile uint8_t radioIncommingBuffer[RADIO_BUFFER_LEN];
static volatile uint8_t radioOutgoingBuffer[RADIO_BUFFER_LEN];
static volatile uint8_t radioTransmitCounter = 0;
static volatile uint8_t radioTransactionLength = 0;
static volatile bool isBlockingTransaction = false;

static xSemaphoreHandle radioSPIMutex = NULL;
static xSemaphoreHandle radioDMADoneSemaphore = NULL;

static void radioConfigureHardwareAndStart();
static void radioFinishTransmission();
static void radioSetupTransaction(const uint8_t length, const uint8_t type);
static void radioPerformBlockingTransaction();

static void radioTransactionTelemetryStartDeferred(void *, uint32_t);
static void radioTransactionTelemetryEndDeferred(void *, uint32_t);

///////////// GLOBALLY ACCESSIBLE FUNCTIONS ////////////////

void radioSetup() {
	if (!radioSPIMutex) {
		radioSPIMutex = xSemaphoreCreateMutex();
	}
	if (!radioDMADoneSemaphore) {
		radioDMADoneSemaphore = xSemaphoreCreateBinary();
	}
}

void radioDisable() {
	xSemaphoreTake(radioSPIMutex, portMAX_DELAY);
	radioSetupTransaction(1, RADIO_MSG_TYPE_RADIO_OFF);
	radioPerformBlockingTransaction();
	xSemaphoreGive(radioSPIMutex);
}

void radioEnable() {
	xSemaphoreTake(radioSPIMutex, portMAX_DELAY);
	radioSetupTransaction(1, RADIO_MSG_TYPE_RADIO_ON);
	radioPerformBlockingTransaction();
	xSemaphoreGive(radioSPIMutex);
}

void radioTestCommand() {
	xSemaphoreTake(radioSPIMutex, portMAX_DELAY);
	radioSetupTransaction(3, RADIO_MSG_TYPE_CONST_0xE5);
	radioPerformBlockingTransaction();
	if (radioIncommingBuffer[1] == 0xE5 && radioIncommingBuffer[2] == 0x5D) {
		safeLog(Log_Type_Debug, 19, "Radio test passed\n");
	} else {
		safeLog(Log_Type_Debug, 31, "Radio test failed 0x%02x 0x%02x\n", radioIncommingBuffer[1], radioIncommingBuffer[2]);
	}

	xSemaphoreGive(radioSPIMutex);
}

void radioDRDYInterruptFromISR() {
	if (GPIO_ReadOutputDataBit(RADIO_GPIO, RADIO_GPIO_CS_PIN) == Bit_SET) { // no SPI actions at the moment
		BaseType_t contextSwitch = pdFALSE;
		xTimerPendFunctionCallFromISR(radioTransactionTelemetryStartDeferred, NULL, 0, &contextSwitch);
		portEND_SWITCHING_ISR(contextSwitch);
	} else { // in the middle of transaction
		if (radioTransmitCounter < radioTransactionLength) {
			SPI_I2S_SendData(RADIO_SPI, radioOutgoingBuffer[radioTransmitCounter++]);
		}
	}
}

void radioSPI_RXDMA_TCIF_FromISR() {
	BaseType_t contextSwitch = pdFALSE;
	DMA_ITConfig(RADIO_RX_DMA_STREAM, DMA_IT_TC, DISABLE);
	if (radioOutgoingBuffer[0] == RADIO_MSG_TYPE_READ_RADIO_BUFFER) {
		xTimerPendFunctionCallFromISR(radioTransactionTelemetryEndDeferred, NULL, 0, &contextSwitch);
	} else {
		xSemaphoreGiveFromISR(radioDMADoneSemaphore, &contextSwitch);
	}
	portEND_SWITCHING_ISR(contextSwitch);
}

///////////// INTERNAL FUNCTIONS //////////////////////

void radioSetupTransaction(const uint8_t length, const uint8_t type) {
	radioTransmitCounter = 0;
	radioTransactionLength = length;
	radioOutgoingBuffer[0] = type;
}

void radioConfigureHardwareAndStart() {
	GPIO_ResetBits(RADIO_GPIO, RADIO_GPIO_CS_PIN);

	RADIO_RX_DMA_STREAM->NDTR = radioTransactionLength; 		// size of payload
	RADIO_RX_DMA_STREAM->M0AR = (uint32_t)radioIncommingBuffer; // base 0 memory address
	RADIO_RX_DMA_STREAM->CR |= DMA_MemoryInc_Enable;			// enable incrementing receive buffer
	DMA_ITConfig(RADIO_RX_DMA_STREAM, DMA_IT_TC, ENABLE);		// enable interrupt after finished reception
	DMA_Cmd(RADIO_RX_DMA_STREAM, ENABLE);						// enable stream
	while (DMA_GetCmdStatus(RADIO_RX_DMA_STREAM) != ENABLE);

	SPI_I2S_DMACmd(RADIO_SPI, SPI_I2S_DMAReq_Rx, ENABLE); 		// issue reception start
	SPI_I2S_SendData(RADIO_SPI, radioOutgoingBuffer[radioTransmitCounter++]); // send first byte
}

void radioFinishTransmission() {
	while (DMA_GetCmdStatus(RADIO_RX_DMA_STREAM) != DISABLE);
	SPI_I2S_DMACmd(RADIO_SPI, SPI_I2S_DMAReq_Rx, DISABLE);
	DMA_ClearFlag(RADIO_RX_DMA_STREAM, RADIO_RX_DMA_FLAG_TCIF);
	while (SPI_I2S_GetFlagStatus(RADIO_SPI, SPI_I2S_FLAG_BSY) == SET);
	GPIO_SetBits(RADIO_GPIO, RADIO_GPIO_CS_PIN);
}

void radioPerformBlockingTransaction() {
	radioConfigureHardwareAndStart();
	xSemaphoreTake(radioDMADoneSemaphore, portMAX_DELAY);
	radioFinishTransmission();
}

void radioTransactionTelemetryStartDeferred(void *ptrParam, uint32_t intParam) {
	xSemaphoreTake(radioSPIMutex, portMAX_DELAY);
	radioSetupTransaction(14, RADIO_MSG_TYPE_READ_RADIO_BUFFER);
	radioConfigureHardwareAndStart();
}

void radioTransactionTelemetryEndDeferred(void *ptrParam, uint32_t intParam) {
	static TelemetryUpdate_Struct telemetryUpdate = {.Source = TelemetryUpdate_Source_Camera};
	uint8_t checksum = 0x55;

	radioFinishTransmission();

	for (uint8_t i = 1; i<13; ++i)
		checksum ^= radioIncommingBuffer[i];

	if (checksum == radioIncommingBuffer[13]) {
		memcpy(&telemetryUpdate.Data, (void*)radioIncommingBuffer+1, 12);
		if (xQueueSendToBack(telemetryQueue, &telemetryUpdate, 0) == errQUEUE_FULL) {
			safeLog(Log_Type_Error, 25, "Telemetry queue full!\n");
		}
	} else {
		safeLog(Log_Type_Error, 57, "Radio incorrect checksum, received 0x%02x calculated 0x%02x\n", radioIncommingBuffer[13], checksum);
	}

	xSemaphoreGive(radioSPIMutex);
}
