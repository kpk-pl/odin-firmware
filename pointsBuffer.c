#include <stdbool.h>
#include "pointsBuffer.h"
#include "compilation.h"
#include "hardware.h"

#ifdef COMPILE_CIRCULAR_BUFFER
#define POINTSBUFFER_SIZE 800
#else
#define POINTSBUFFER_SIZE 1
#endif

#define ONE_POINT_SIZE_BYTES 20 	/*<< As program is saving data as 32-bit values, this must be a multiple of 4 */

/*
 * Static types
 */

typedef volatile uint16_t Iterator;

typedef volatile struct {
	TrajectoryPoint_Struct buffer[POINTSBUFFER_SIZE];	/*<< circular buffer */
	Iterator readIt;									/*<< iterator to current element in buffer */
	Iterator lockItBegin;								/*<< iterator to the beginning of space locked for DMA */
	Iterator lockItEnd;									/*<< iterator to the end of space locked for DMA */
	uint16_t continueLoading;							/*<< number of items to load after loading on last position in buffer */
} CircularBuffer_Struct;

CircularBuffer_Struct circularBuffer;

/*
 * Static declarations
 */

static void iteratorInc(Iterator* it);
static Iterator iteratorAdd(Iterator it, uint16_t inc);
static void startDMA(uint16_t num);

uint16_t TBloadNewPoints(uint16_t num) {
	if (circularBuffer.lockItBegin != circularBuffer.lockItEnd) return 0; // another transmission currently in place

	uint16_t availableSpace = POINTSBUFFER_SIZE - TBgetAvailablePoints() - 1;
	if (num > availableSpace) num = availableSpace;						  // trim excess points

	if (circularBuffer.lockItBegin + num > POINTSBUFFER_SIZE) {		  	  // two transfers, divide everything
		circularBuffer.continueLoading = circularBuffer.lockItBegin + num - POINTSBUFFER_SIZE;
	}
	else {																  // one transfer is enough
		circularBuffer.continueLoading = 0;
	}

	startDMA(num - circularBuffer.continueLoading);						  // start receiving first part or all

	return num;
}

TrajectoryPoint_Ptr TBgetNextPoint() {
	TrajectoryPoint_Struct* it;
	if (circularBuffer.readIt == circularBuffer.lockItBegin) return (void*)(0);

	it = &circularBuffer.buffer[circularBuffer.readIt];
	iteratorInc(&circularBuffer.readIt);
	return it;
}

uint16_t TBgetAvailablePoints() {
	int16_t available = (int16_t)circularBuffer.lockItBegin - (int16_t)circularBuffer.readIt;
	if (available < 0) available += POINTSBUFFER_SIZE;
	return available;
}

uint16_t TBgetSize() {
	return POINTSBUFFER_SIZE;
}

float TBgetFreeSpace() {
	return (float)(POINTSBUFFER_SIZE-TBgetAvailablePoints()) / (float)POINTSBUFFER_SIZE;
}

float TBgetUsedSpace() {
	return (float)TBgetAvailablePoints() / (float)POINTSBUFFER_SIZE;
}

void TBresetBuffer() {
	circularBuffer.readIt = 0;
	circularBuffer.lockItEnd = 0;
	circularBuffer.lockItBegin = 0;
	circularBuffer.continueLoading = 0;
}

void TBDMATransferCompletedSlot() {
	/* Disable DMA stream */
	DMA_Cmd(WIFI_RX_DMA_STREAM, DISABLE);
	DMA_ClearFlag(WIFI_RX_DMA_STREAM, WIFI_RX_DMA_FLAG_TCIF); // clear flag here because it must be cleared before next DMA transfer is configured
	while (WIFI_RX_DMA_STREAM->CR & DMA_SxCR_EN);			// wait for the stream to be fully off

	// unlock memory
	circularBuffer.lockItBegin = circularBuffer.lockItEnd;

	if (circularBuffer.continueLoading == 0) { 			// finish up, enable normal USART behavior
		USART_DMACmd(WIFI_USART, USART_DMAReq_Rx, DISABLE);
		USART_ITConfig(WIFI_USART, USART_IT_RXNE, ENABLE);
	}
	else {												// read second half
		startDMA(circularBuffer.continueLoading);
		circularBuffer.continueLoading = 0;
	}
}

void iteratorInc(Iterator* it) {
	(*it)++;
	if (*it == POINTSBUFFER_SIZE) {
		*it = 0;
	}
}

Iterator iteratorAdd(Iterator it, uint16_t inc) {
	return (it + inc) % POINTSBUFFER_SIZE;
}

void startDMA(uint16_t num) {
	if (num == 0) return;

	circularBuffer.lockItEnd = iteratorAdd(circularBuffer.lockItBegin, num);	// move iterator and lock memory

	WIFI_RX_DMA_STREAM->M0AR = (uint32_t)(&circularBuffer.buffer[circularBuffer.lockItBegin]); // base address
	WIFI_RX_DMA_STREAM->NDTR = num * ONE_POINT_SIZE_BYTES;	// number of bytes to transfer

    USART_ITConfig(WIFI_USART, USART_IT_RXNE, DISABLE);		// disable interrupts from USART, disconnect Command Handling
    DMA_Cmd(WIFI_RX_DMA_STREAM, ENABLE);					// enable DMA stream
    USART_DMACmd(WIFI_USART, USART_DMAReq_Rx, ENABLE);		// set USART to DMA
}
