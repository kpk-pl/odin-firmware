#include "priorities.h"
#include "stackSpace.h"

#include "TaskAsyncCallHandler.h"

xTaskHandle AsyncCallHandlerTask;
xQueueHandle AsyncCallHandlerQueue;

static void TaskAsyncCallHandler(void *p) {
	AsyncCall_Type call;
	while(1) {
		xQueueReceive(AsyncCallHandlerQueue, &call, portMAX_DELAY);
		switch (call.Type) {
		case AsyncCallProc_Void:
			call.CallVoid();
			break;
		case AsyncCallProc_VoidPtr:
			call.CallVoidPtr(call.VoidPtrParam);
			break;
		case AsyncCallProc_Int:
			call.CallInt(call.IntParam);
			break;
		default:
			break;
		}
	}
}

void AsyncCallHandlerTaskConstructor() {
	xTaskCreate(TaskAsyncCallHandler, NULL, TASKASYNCCALLHANDLER_STACKSPACE, NULL, PRIORITY_TASK_ASYNCCALLHANDLER, &AsyncCallHandlerTask);
	AsyncCallHandlerQueue = xQueueCreate(5, sizeof(AsyncCall_Type));
}
