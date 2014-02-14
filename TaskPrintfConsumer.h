#ifndef _TASKPRINTFCONSUMER_H_
#define _TASKPRINTFCONSUMER_H_

#include <stdarg.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "logging.h"

extern xTaskHandle printfConsumerTask;			/*!< Export this task handle */
extern xSemaphoreHandle comUSARTTCSemaphore;	/*!< Export transmission complete semaphore (USB-UART), will be given in ISR from other task */
extern xSemaphoreHandle wifiUSARTTCSemaphore;	/*!< Export transmission complete semaphore (WiFi module), will be given in ISR from other task */
extern xSemaphoreHandle printfMutex;			/*!< Export printf mutex - needs to be acquired to use ordinary printf */

extern volatile Log_Settings_Struct globalLogSettings;		/*!< Export log settings */


/**
 * \brief Print formated text via USB and WiFi (if enabled) in thread-safe and non-blocking way.
 *
 * This function performs the same logical action as normal printf.
 * This function uses <stdio.h> which is very resource-consuming and takes a lot of stack space
 * @param length Maximum length that will be allocated for printed string. If string is longer, then it will be truncated
 * @param format Refer to printf
 * @return Refer to printf
 */
int safePrint(const size_t length, const char *format, ...);

/**
 * \brief Helper function to invoke safePrint with the right log format and length. For example it prepends
 * log with [Telemetry] if log is of type Log_Type_Telemetry.
 * Log is only printed if associated Log Flag is set globally
 * This function is only safe outside ISRs.
 *
 * @see safePrint
 * @param type Type of log
 */
int safeLog(const Log_Type type, const size_t length, const char *format, ...);

/**
 * \brief Sets all OS-related objects for use. It should be called on startup, only once, before scheduler starts
 */
void TaskPrintfConsumerConstructor();

#endif /* _TASKPRINTFCONSUMER_H_ */
