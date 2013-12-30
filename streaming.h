#ifndef STREAMING_H_
#define STREAMING_H_

#include <stdbool.h>
#include "FreeRTOS.h"

/**
 * Waits for the stream to be available and acquires resources if timeout is not exceeded
 * @param blockTime Maximum time to wait for the resources
 * @retval true if resource was acquired, false in case of timeout
 */
bool streamAcquire(portBASE_TYPE blockTime);

/**
 * Releases previously acquired resources allowing other tasks to use the stream. Stream must be first
 * cleaned with a call to streamFinishTransmission.
 * @retval true if stream was successfully released, false if stream was being used or streamFinishTransmission
 * was not called after last transmission.
 */
bool streamRelease();

/**
 * Initiates transmission via DMA stream. When transmission ends used defined function will be called.
 * User might want to order a few transmission in the row without interrupting the incomming data stream.
 * This might be achieved by calling this function several times in the row.
 * After last transmission user must call streamFinishTransmission procedure
 * @param dest Pointer to the memory area to which incomming data will be saved. Must not be in CCM
 * @param bytes Number of bytes to receive.
 * @param finishHandle Pointer to user defined function which will be called after successfull transmission
 * @param finishParam Parameter that will be passed to finishHandle function when transmission ends
 */
void streamStartTransmission(void *dest, uint32_t bytes, void (*finishHandle)(void*), void *finishParam);

/**
 * Cleans up after last DMA transmission
 */
void streamFinishTransmission();

#endif /* STREAMING_H_ */
