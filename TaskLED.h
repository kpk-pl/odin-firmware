#ifndef _TASKLED_H_
#define _TASKLED_H_

/**
 * \brief Blinking LED task - indication that scheduler is running and no task is hang. Watchdog resetting
 */
void TaskLED(void *);

/**
 * \brief Sets all OS-related objects for use. It should be called on startup, only once, before scheduler starts
 */
void TaskLEDConstructor();

#endif /* _TASKLED_H_ */
