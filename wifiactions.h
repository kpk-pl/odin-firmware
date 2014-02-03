#ifndef _WIFIACTIONS_H_
#define _WIFIACTIONS_H_

#include "FreeRTOS.h"
#include "queue.h"

extern xQueueHandle WiFiActionsInputQueue; /* Queue to which input data will be passed */

void reconnectToAP(int turnOn);

#endif /* _WIFIACTIONS_H_ */
