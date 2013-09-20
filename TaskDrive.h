#ifndef _TASKDRIVE_H_
#define _TASKDRIVE_H_

#include "compilation.h"
#ifndef FOLLOW_TRAJECTORY

/**
 * \brief Controls following one of four available trajectories.
 *
 * This tasks executes drive commands one by one and tries to stay on the desired path.
 * Sometimes it calculates the best route to target point.
 */
void TaskDrive(void *);

#endif /* FOLLOW_TRAJECTORY */
#endif /* _TASKDRIVE_H_ */
