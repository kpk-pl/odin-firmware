#ifndef _TASKPRINTFCONSUMER_H_
#define _TASKPRINTFCONSUMER_H_

#include <stdarg.h>

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

extern xTaskHandle printfConsumerTask;			/*!< Export this task handle */
extern xSemaphoreHandle comUSARTTCSemaphore;	/*!< Export transmission complete semaphore (USB-UART), will be given in ISR from other task */
extern xSemaphoreHandle wifiUSARTTCSemaphore;	/*!< Export transmission complete semaphore (WiFi module), will be given in ISR from other task */

/**
 * \brief Handles safePrint invocations and printing everything on active interfaces
 */
void TaskPrintfConsumer(void *);

/**
 * \brief Sets all OS-related objects for use. It should be called on startup, only once, before scheduler starts
 */
void TaskPrintfConsumerConstructor();

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
 *  \brief Implementation of safePrint that is safe to use in interrupts. DO NOT USE NORMAL VERSION IN ISR!
 *
 *  @see safePrint
 */
int safePrintFromISR(const size_t length, const char *format, ...);

#endif /* _TASKPRINTFCONSUMER_H_ */
