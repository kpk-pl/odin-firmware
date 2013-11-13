#ifndef _TASKCOMMANDHANDLER_H_
#define _TASKCOMMANDHANDLER_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

extern xTaskHandle commandHandlerTask;
extern xQueueHandle commandQueue;

/**
 * \brief Handles incomming commands from UART via USB or WiFi
 *
 * It constantly checks if there is any message in queue and if there is it executes
 * an action according to command type in "commands.h"
 * @see commands.h
 */
void TaskCommandHandler(void *);

/**
 * \brief Sets all OS-related objects for use. It should be called on startup, only once, before scheduler starts
 */
void TaskCommandHandlerConstructor();

#endif /* _TASKCOMMANDHANDLER_H_ */
