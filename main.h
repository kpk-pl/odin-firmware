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

/* Type of telemetry update */
typedef enum {
	TelemetryUpdate_Source_Odometry = 0,	/*<< Update from odometry - encoders */
#ifdef USE_IMU_TELEMETRY
	TelemetryUpdate_Source_IMU				/*<< Update from IMU */
#endif
} TelemetryUpdate_Source;

/* Struct to hold telemetry updates from various sources. Based on these updates, position and orientation is calculated */
typedef struct {
	TelemetryUpdate_Source Source;			/*<< Update source */
	float dX;								/*<< Change in X position */
	float dY;								/*<< Change in Y position */
	float dO;								/*<< Change of orientation angle (radians) */
} TelemetryUpdate_Struct;

/* Struct for holding position and orientation */
typedef struct {
	float X;				/*<< X coordinate */
	float Y;				/*<< Y coordinate */
	float O;				/*<< Orientation angle coordinate in radians */
} TelemetryData_Struct;

/* Struct holding motors' speeds. Choosen unit is rad/sec */
typedef struct {
	float LeftSpeed;				/*<< Left motor's speed */
	float RightSpeed;				/*<< Right motor's speed */
} MotorSpeed_Struct;

/*
 * Global variables, defined in main.c
 */

extern volatile FunctionalState globalLogEvents;
extern volatile FunctionalState globalLogTelemetry;
extern volatile FunctionalState globalLogSpeed;
extern volatile FunctionalState globalSpeedRegulatorOn;
extern volatile uint32_t globalLogSpeedCounter;
extern volatile TelemetryData_Struct globalTelemetryData;

#ifdef USE_IMU_TELEMETRY
	extern volatile bool globalIMUHang;
	extern volatile bool globalDoneIMUScaling;
	extern volatile FunctionalState globalMagnetometerScalingInProgress;
	extern float globalMagnetometerImprovData[721];
	extern arm_linear_interp_instance_f32 globalMagnetometerImprov;
#endif
#ifdef USE_CUSTOM_MOTOR_CONTROLLER
	extern MotorControllerParameters_Struct globalLeftMotorParams;
	extern MotorControllerParameters_Struct globalRightMotorParams;
	extern volatile FunctionalState globalControllerVoltageCorrection;
#else
	extern arm_pid_instance_f32 globalPidLeft;
	extern arm_pid_instance_f32 globalPidRight;
#endif

/*
 * Global OS objects
 */

#ifdef USE_IMU_TELEMETRY
	extern xTaskHandle imuTask;
#endif

extern xSemaphoreHandle rc5CommandReadySemaphore;		// used by RC5 API to inform about new finished transmission
#ifdef USE_IMU_TELEMETRY
	extern xSemaphoreHandle imuPrintRequest;			// request for IMU to print most up-to-date reading
	extern xSemaphoreHandle imuGyroReady;				// gyro ready flag set in interrupt
	extern xSemaphoreHandle imuAccReady;				// accelerometer ready flag set in interrupt
	extern xSemaphoreHandle imuMagReady;				// magnetometer ready flag set in interrupt
	extern xSemaphoreHandle imuI2CEV;					// semaphore to indicate correct event in I2C protocol
	extern xSemaphoreHandle imuMagScalingReq;			// request to perform magnetometer scaling
#endif

#ifdef USE_IMU_TELEMETRY
	extern xTimerHandle imuWatchdogTimer;				// used by software watchdog, if it expires then I2C is reset
#endif

extern xQueueHandle motorCtrlQueue;						// One-element queue for setting wheel's speed
extern xQueueHandle telemetryQueue;						// Queue for sending updates to telemetry task. This queue holds updates from all available sources
#ifdef USE_IMU_TELEMETRY
	extern xQueueHandle I2CEVFlagQueue;					// Buffer for I2C event interrupt that holds new event flag to wait for
	extern xQueueHandle magnetometerScalingQueue;		// Queue for data from IMU task for magnetometer scaling. Created on demand in scaling task.
#endif

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
