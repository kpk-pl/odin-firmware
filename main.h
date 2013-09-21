#ifndef _MAIN_H_
#define _MAIN_H_

#include <stm32f4xx.h>
#include "arm_math.h"

#include <stdbool.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "timers.h"
#include "task.h"
#include "portmacro.h"

#include "compilation.h"

#ifdef USE_CUSTOM_MOTOR_CONTROLLER
#include "motorController.h"
#endif
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

/* Struct holding motors' speeds. Choosen unit is rad/sec */
typedef struct {
	float LeftSpeed;				/*<< Left motor's speed */
	float RightSpeed;				/*<< Right motor's speed */
} MotorSpeed_Struct;

/* Types of different logging commands */
typedef enum {
	Logging_Type_Telemetry = 't',		/*<< Log position and orientation when it changes */
	Logging_Type_Speed = 's',			/*<< Log wheels speed */
	Logging_Type_Events = 'e'		/*<< Log system events */
} Logging_Type;
#define IS_LOGGING_TYPE(x) (x == Logging_Type_Telemetry || x == Logging_Type_Speed || x == Logging_Type_Events)

/*
 * Global variables, defined in main.c
 */

extern volatile FunctionalState globalLogEvents;
extern volatile FunctionalState globalLogTelemetry;
extern volatile FunctionalState globalLogSpeed;
extern volatile FunctionalState globalSpeedRegulatorOn;
extern volatile uint32_t globalLogSpeedCounter;
extern volatile float globalCPUUsage;
extern volatile TelemetryData_Struct globalTelemetryData;

#ifdef USE_CUSTOM_MOTOR_CONTROLLER
	extern MotorControllerParameters_Struct globalLeftMotorParams;
	extern MotorControllerParameters_Struct globalRightMotorParams;
	extern volatile FunctionalState globalControllerVoltageCorrection;
#else
	extern arm_pid_instance_f32 globalPidLeft;
	extern arm_pid_instance_f32 globalPidRight;
#endif
#ifdef FOLLOW_TRAJECTORY
	extern TrajectoryControlerGains_Struct globalTrajectoryControlGains;
#endif

/*
 * Global OS objects - tasks
 */

#ifndef FOLLOW_TRAJECTORY
	extern xTaskHandle driveTask;
#else
	extern xTaskHandle trajectoryTask;
#endif

/*
 * Global OS objects - semaphores
 */

extern xSemaphoreHandle comUSARTTCSemaphore;			// USART_TC flag set for USB-USART
extern xSemaphoreHandle comDMATCSemaphore;				// DMA TC flag set for USB-USART
extern xSemaphoreHandle wifiUSARTTCSemaphore;			// USART TC flag set for WIFI-USART
extern xSemaphoreHandle wifiDMATCSemaphore;				// DMA TC flag set for WIFI-USART
extern xSemaphoreHandle rc5CommandReadySemaphore;		// used by RC5 API to inform about new finished transmission

/*
 * Global OS objects - queues
 */

extern xQueueHandle printfQueue;						// Queue for safePrint strings to send via active interfaces
extern xQueueHandle telemetryQueue;						// Queue for sending updates to telemetry task. This queue holds updates from all available sources
extern xQueueHandle WiFi2USBBufferQueue;				// Buffer for WiFi to USB characters
extern xQueueHandle USB2WiFiBufferQueue;				// Buffer for USB to WiFi characters

/*
 * Global functions
 */

/*
 * @brief Print formated text via USB and WiFi (if enabled) in thread-safe and non-blocking way.
 * This function performs the same logical action as normal printf.
 * This function uses <stdio.h> which is very resource-consuming and takes a lot of stack space
 * @param length Maximum length that will be allocated for printed string. If string is longer, then it will be truncated
 * @param format Refer to printf
 * @retval Refer to printf
 */
int safePrint(const size_t length, const char *format, ...);

/* Implementation of safePrint that is safe to use in interrupts. DO NOT USE NORMAL VERSION IN ISR! */
int safePrintFromISR(const size_t length, const char *format, ...);

/* Returns normalized orientation angle provided as input in radians, output is [-PI, +PI] */
float normalizeOrientation(float in);

/* Returns current up-to-date telemetry data and saves it in provided structure. This function provides mutual exclusion and data coherency */
void getTelemetry(TelemetryData_Struct *data);

/* Returns current telemetry data without orientation normalization to +-M_PI */
void getTelemetryRaw(TelemetryData_Struct *data);

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
