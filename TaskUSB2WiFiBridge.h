#ifndef _TASKUSB2WIFIBRIDGE_H_
#define _TASKUSB2WIFIBRIDGE_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

extern xTaskHandle USBWiFiBridgeTask;
extern xQueueHandle USB2WiFiBridgeQueue;

/**
 * \brief Task for USB-WiFi bridge to transfer commands between two UARTs
 */
void TaskUSBWiFiBridge(void *);

/**
 * \brief Sets all OS-related objects for use. It should be called on startup, only once, before scheduler starts
 */
void TaskUSB2WiFiBridgeConstructor();

#endif /* _TASKUSB2WIFIBRIDGE_H_ */
