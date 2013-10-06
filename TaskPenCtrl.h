#ifndef _TASK_PENCTRL_H_
#define _TASK_PENCTRL_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

extern xTaskHandle penCtrlTask;						/*!< Export this task's handle */
extern xQueueHandle penCommandQueue;				/*!< Export queue that is used to issue pen commands */

typedef enum {
	PenLine_Dotted = 0,			// DO NOT CHANGE NUMBERING
	PenLine_DashedShort = 1,	// DO NOT CHANGE NUMBERING
	PenLine_DashedLong = 2,		// DO NOT CHANGE NUMBERING
	PenLine_DotDash = 3,		// DO NOT CHANGE NUMBERING
	PenLine_Continuous = 4
} PenLine_Type;
#define IS_PENLINE_TYPE(x) ((x) >= 0 && (x) <= 4)

/**
 * \brief Sets new type of line to be drawn while driving
 * @param type Line type to be set
 */
void setPenLineType(PenLine_Type type);


/**
 * \brief Controller for motors
 */
void TaskPenCtrl(void *);				// Motors' speed regulator

/**
 * \brief Sets all OS-related objects for use. It should be called on startup, only once, before scheduler starts
 */
void TaskPenCtrlConstructor();

#endif /* _TASK_PENCTRL_H_ */
