#ifndef _TASKDRIVE_H_
#define _TASKDRIVE_H_

#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

extern xTaskHandle driveTask;
extern xQueueHandle driveQueue;

/**
 * \brief Type of driving command to perform
 */
typedef enum {
	DriveCommand_Type_Line =  'l',		/*<< Drive straight line; need one parameter - length */
	DriveCommand_Type_Point = 'p',		/*<< Drive to point; need two parameters - X and Y coordinates */
	DriveCommand_Type_Arc =   'a',		/*<< Drive around an arc; need two parameters - radius and arc length in degrees */
	DriveCommand_Type_Angle = 'd'		/*<< Turn by an angle; need two parameters - one indicating wheather it is relative (0) or absolute (1) angle and second being an angle in degrees */
} DriveCommand_Type;

/**
 *  \brief Struct to hold driving information needed by trajectory controller
 */
typedef struct {
	DriveCommand_Type Type;			/*<< Command type, one of DriveCommand_Type */
	bool UsePen;					/*<< If true then pen will be held down, up otherwise */
	float Speed;					/*<< Robot's speed, only positive values */
	float Param1;					/*<< Command param #1 */
	float Param2;					/*<< Command param #2 */
} DriveCommand_Struct;

/**
 * \brief Controls following one of four available trajectories.
 *
 * This tasks executes drive commands one by one and tries to stay on the desired path.
 * Sometimes it calculates the best route to target point.
 */
void TaskDrive(void *);

/**
 * \brief Sets all OS-related objects for use. It should be called on startup, only once, before scheduler starts
 */
void TaskDriveConstructor();

#endif /* _TASKDRIVE_H_ */
