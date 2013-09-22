#ifndef _TASK_RC5_H_
#define _TASK_RC5_H_

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

extern xTaskHandle RC5Task;							/*!< Export this task handle */
extern xSemaphoreHandle rc5CommandReadySemaphore;	/*!< Export semaphore indicating that RC5 command was received */

/**
 * \brief Handles RC5 remote commands
 */
void TaskRC5(void *);

/**
 * \brief Sets all OS-related objects for use. It should be called on startup, only once, before scheduler starts
 */
void TaskRC5Constructor();

#endif /* TASK_RC5_H_ */
