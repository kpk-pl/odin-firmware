#ifndef _TASK_WIFI_MNGR_H_
#define _TASK_WIFI_MNGR_H_

#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

extern xTaskHandle WiFiMngrTask;
extern xQueueHandle WiFiMngrInputQueue;

typedef enum {
	WiFiMngr_Command_Reconnect = 0,
} WiFiMngr_Command_Type;

/**
 * Task fired up when WiFi reconfiguration is needed
 * @param cmd Command to perform by this task
 */
void TaskWiFiMngr(WiFiMngr_Command_Type *cmd);

/**
 * Constructor of TaskWiFiMngr
 * @param command Command to perform by WiFi manager
 * @retval Returns true on success, false otherwise. Task might not be
 * created due to OS problems. Only one WiFiMngr can exist at a time.
 */
bool TaskWiFiMngrConstructor(WiFiMngr_Command_Type command);

/**
 * Destructor of TaskWiFiMngr
 */
void TaskWiFiMngrDestructor();

#endif /* _TASK_WIFI_MNGR_H_ */
