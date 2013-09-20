#ifndef _TASKDRIVE_H_
#define _TASKDRIVE_H_

#include "compilation.h"
#ifndef FOLLOW_TRAJECTORY

void TaskDrive(void *);	// Task controlling trajectory. Issues wheel's speed commands, checks if target is reached, calculates best route

#endif /* FOLLOW_TRAJECTORY */
#endif /* _TASKDRIVE_H_ */
