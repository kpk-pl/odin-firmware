#ifndef _TASKINPUTBUFFER_H_
#define _TASKINPUTBUFFER_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

extern xTaskHandle commInputBufferTask;
extern xQueueHandle commInputBufferQueue;

/**
 *  \brief Types of input's character source
 */
typedef enum {
	PrintSource_Type_USB = 1,	/*!< From USB-UART converter */
	PrintSource_Type_WiFi = 2	/*!< From WiFi module */
} PrintSource_Type;

/**
 *  \brief Holds one input char with it's source
 */
typedef struct {
	char Input;					/*!< Incomming character */
	PrintSource_Type Source;	/*!< Source of the character */
} PrintInput_Struct;

/**
 * \brief Handles incoming characters from any source available making it possible to speed up transfer.
 *
 * Essentially it does the same work as interrupts should, but in this way interrupts only send received data to
 * process in this task. Interrupt routines are therefore shorter and can be called more often.
 */
void TaskInputBuffer(void *);			// Task to handle input characters from any source - it makes possible to set very high transfer speeds

/**
 * \brief Sets all OS-related objects for use. It should be called on startup, only once, before scheduler starts
 */
void TaskInputBufferConstructor();

#endif /* _TASKINPUTBUFFER_H_ */
