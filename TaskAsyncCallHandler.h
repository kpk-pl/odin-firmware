#ifndef _TASKASYNCCALLHANDLER_H_
#define _TASKASYNCCALLHANDLER_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

typedef enum {
	AsyncCallProc_Void = 0,
	AsyncCallProc_VoidPtr,
	AsyncCallProc_Int
} AsyncCallProc_Type;

typedef struct {
	 union {
		 void (*CallVoid)(void);    		/*!< Function taking void that will be called */
		 struct {
			 void (*CallVoidPtr)(void*);	/*!< Function taking void* that will be called */
			 void *VoidPtrParam;			/*!< Param that will be passed to CallParam function */
		 };
		 struct {
			 void (*CallInt)(int);			/*!< Function taking int that will be called */
			 int IntParam;					/*!< Param that will be passed to CallInt function */
		 };
	 };
	 AsyncCallProc_Type Type;				/*!< Type of call function */
} AsyncCall_Type;

extern xTaskHandle AsyncCallHandlerTask;
extern xQueueHandle AsyncCallHandlerQueue;

void AsyncCallHandlerTaskConstructor();

#endif /*_TASKASYNCCALLHANDLER_H_*/
