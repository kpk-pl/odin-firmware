#ifndef _TASKCOMMANDHANDLER_H_
#define _TASKCOMMANDHANDLER_H_

/**
 * \brief Handles incomming commands from UART via USB or WiFi
 *
 * It constantly checks if there is any message in queue and if there is it executes
 * an action according to command type in "commands.h"
 * @see commands.h
 */
void TaskCommandHandler(void *);		// Task handling incomming commands

#endif /* _TASKCOMMANDHANDLER_H_ */
