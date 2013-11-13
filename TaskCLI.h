#ifndef _TASKCLI_H_
#define _TASKCLI_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

extern xTaskHandle CLITask;
extern xQueueHandle CLIInputQueue;

/**
 * Handles incoming lines from interfaces, performs commands and prints results on interfaces or into file
 * It is use instead of TaskCommandHandler. Only one should be running.
 * This task mimics all external interfaces that TaskCommandHandler uses (queues etc.)
 */
void TaskCLI(void *);

/**
 * \brief Sets all OS-related objects for use. It should be called on startup, only once, before scheduler starts
 */
void TaskCLIConstructor();

#endif /* _TASKCLI_H_ */
