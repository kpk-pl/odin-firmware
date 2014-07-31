#include <stdbool.h>
#include <string.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "hardware.h"
#include "radioRcvr.h"

#define RADIO_BUFFER_LEN 20

#define RADIO_MSG_TYPE_READ_RADIO_BUFFER 	'R'
#define RADIO_MSG_TYPE_CONST_0xE5 			'c'
#define RADIO_MSG_TYPE_RADIO_OFF 			'o'
#define RADIO_MSG_TYPE_RADIO_ON 			'O'

static volatile uint8_t radioPositionBuffer[RADIO_BUFFER_LEN];
static volatile uint8_t radioOutgoingBuffer[RADIO_BUFFER_LEN];
static volatile uint8_t radioTransmitCounter = 0;
static volatile uint8_t radioTransactionLength = 0;
static volatile bool isBlockingTransaction = false;

static xSemaphoreHandle radioSPIMutex = NULL;
static xSemaphoreHandle radioDMADoneSemaphore = NULL;
static xSemaphoreHandle radioTelemetryReceivedSemaphore = NULL;

static void radioConfigureHardwareAndStart();
static void radioFinishTransmission();
static void radioSetupTransaction(const uint8_t length, const uint8_t type);
static void radioPerformBlockingTransaction();

void radioSetup() {
	if (!radioSPIMutex) {
		radioSPIMutex = xSemaphoreCreateMutex();
	}
	if (!radioDMADoneSemaphore) {
		radioDMADoneSemaphore = xSemaphoreCreateBinary();
	}
	if (!radioTelemetryReceivedSemaphore) {
		radioTelemetryReceivedSemaphore = xSemaphoreCreateBinary();
	}
}

void radioSetupTransaction(const uint8_t length, const uint8_t type) {
	radioTransmitCounter = 0;
	radioTransactionLength = length;
	radioOutgoingBuffer[0] = type;
}

void radioConfigureHardwareAndStart() {
	GPIO_ResetBits(RADIO_GPIO, RADIO_GPIO_CS_PIN);

	RADIO_RX_DMA_STREAM->NDTR = radioTransactionLength; 		// size of payload
	RADIO_RX_DMA_STREAM->M0AR = (uint32_t)radioPositionBuffer; 	// base 0 memory address
	RADIO_RX_DMA_STREAM->CR |= DMA_MemoryInc_Enable;			// enable incrementing receive buffer
	DMA_ITConfig(RADIO_RX_DMA_STREAM, DMA_IT_TC, ENABLE);		// enable interrupt after finished reception
	DMA_Cmd(RADIO_RX_DMA_STREAM, ENABLE);						// enable stream
	while (DMA_GetCmdStatus(RADIO_RX_DMA_STREAM) != ENABLE);

	SPI_I2S_DMACmd(RADIO_SPI, SPI_I2S_DMAReq_Rx, ENABLE); 		// issue reception start
	SPI_I2S_ITConfig(RADIO_SPI, SPI_I2S_IT_TXE, ENABLE); 		// enable SPI transmission interrupt, should fire up
}

void radioFinishTransmission() {
	while (DMA_GetCmdStatus(RADIO_RX_DMA_STREAM) != DISABLE);
	SPI_I2S_DMACmd(RADIO_SPI, SPI_I2S_DMAReq_Rx, DISABLE);
	DMA_ClearFlag(RADIO_RX_DMA_STREAM, RADIO_RX_DMA_FLAG_TCIF);
	while (SPI_I2S_GetFlagStatus(RADIO_SPI, SPI_I2S_FLAG_BSY) == SET);
	GPIO_SetBits(RADIO_GPIO, RADIO_GPIO_CS_PIN);
}

void radioPerformBlockingTransaction() {
	taskENTER_CRITICAL();
		xSemaphoreTake(radioSPIMutex, portMAX_DELAY);
		isBlockingTransaction = true;
	taskEXIT_CRITICAL();

	radioConfigureHardwareAndStart();
	xSemaphoreTake(radioDMADoneSemaphore, portMAX_DELAY);
	radioFinishTransmission();

	taskENTER_CRITICAL();
		isBlockingTransaction = false;
		xSemaphoreGive(radioSPIMutex);
	taskEXIT_CRITICAL();
}

void radioTransactionTelemetryFromISR() {
	if (!isBlockingTransaction) {
		radioSetupTransaction(14, RADIO_MSG_TYPE_READ_RADIO_BUFFER);
		radioConfigureHardwareAndStart();
	}
}

void radioSPI_TXE_FromISR() {
	if (radioTransmitCounter >= radioTransactionLength) {
		SPI_I2S_ITConfig(RADIO_SPI, SPI_I2S_IT_TXE, DISABLE);
	} else {
		SPI_I2S_SendData(RADIO_SPI, radioOutgoingBuffer[radioTransmitCounter++]);
	}
}

void radioSPI_RXDMA_TCIF_FromISR() {
	portBASE_TYPE contextSwitch = pdFALSE;
	DMA_ITConfig(RADIO_RX_DMA_STREAM, DMA_IT_TC, DISABLE);
	if (isBlockingTransaction) {
		xSemaphoreGiveFromISR(radioDMADoneSemaphore, &contextSwitch);
	} else {
		xSemaphoreGiveFromISR(radioTelemetryReceivedSemaphore, &contextSwitch);
	}
	portEND_SWITCHING_ISR(contextSwitch);
}

///////////

void radioDisable() {
	xSemaphoreTake(radioSPIMutex, portMAX_DELAY);
	radioSetupTransaction(1, RADIO_MSG_TYPE_RADIO_OFF);
	radioPerformBlockingTransaction();
	xSemaphoreGive(radioSPIMutex);
}

void radioEnable() {
	radioSetupTransaction(1, RADIO_MSG_TYPE_RADIO_ON);
	radioPerformBlockingTransaction();
}
