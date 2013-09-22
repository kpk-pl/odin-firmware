#ifndef _MAIN_H_
#define _MAIN_H_

#include <stm32f4xx.h>
#include "arm_math.h"

#include <stdbool.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "portmacro.h"

#include "compilation.h"

#ifdef FOLLOW_TRAJECTORY
#include "TaskTrajectory.h"
#endif

#include "TaskTelemetry.h"

/*
 * Global defines
 */

#define M_PI 				(3.14159265358979323846f)			/*<< PI */
#define IMPS_PER_REV 		(3592.0f)							/*<< Number of encoder impulses per wheel revolution */
#define IMPS_TO_RAD 		(2.0f * M_PI / IMPS_PER_REV)		/*<< Coefficient to convert encoder impulses to radians */
#define ROBOT_DIAM			(187.3f)//(187.0f)//(184.9f)		/*<< Distance between two wheels */
#define WHEEL_DIAM 			(69.76f)//(70.0f)					/*<< Wheel diameter */
#define IMPS_TO_MM_TRAVELED (WHEEL_DIAM * M_PI / IMPS_PER_REV)	/*<< Coefficient to convert encoder inpulses to distance traveled on wheel */
#define RAD_TO_MM_TRAVELED	(WHEEL_DIAM / 2.0f)					/*<< Coefficient to convert radians to distance traveled on wheel */
#define DEGREES_TO_RAD		(M_PI / 180.0f)						/*<< Coefficient to convert degrees to radians */

/*
 * Global types
 */

/* Types of different logging commands */
typedef enum {
	Logging_Type_Telemetry = 't',		/*<< Log position and orientation when it changes */
	Logging_Type_Speed = 's',			/*<< Log wheels speed */
	Logging_Type_Events = 'e'			/*<< Log system events */
} Logging_Type;
#define IS_LOGGING_TYPE(x) (x == Logging_Type_Telemetry || x == Logging_Type_Speed || x == Logging_Type_Events)

/*
 * Global variables, defined in main.c
 */

extern volatile FunctionalState globalLogEvents;
extern volatile FunctionalState globalLogTelemetry;
extern volatile FunctionalState globalLogSpeed;
extern volatile float globalCPUUsage;

/*
 * Global functions
 */

/* Prints free stack space for each running task */
void reportStackUsage();

/*
 * Interrupt routines
 */

void COMAction();
void COMDMANotify();
#ifdef FOLLOW_TRAJECTORY
void RawStreamDMAIncoming();
#endif
void WIFIAction();
void WiFiDMANotify();
#ifdef USE_IMU_TELEMETRY
void IMUGyroReady();
void IMUAccReady();
void IMUMagReady();
void IMUI2CEVHandler();
#endif /* USE_IMU_TELEMETRY */
void BatteryTooLow();
void OSBusyTimerHandler();

void Switch1Changed();
void Switch2Changed();
void Switch3Changed();
void Switch4Changed();
void Switch5Changed();
void Switch6Changed();

#endif
