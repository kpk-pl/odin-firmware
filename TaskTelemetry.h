#ifndef _TASKTELEMETRY_H_
#define _TASKTELEMETRY_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <stdbool.h>

extern float globalOdometryCorrectionGain;	/*!< Export the parameter that corrects odometry while turning */
extern float globalPositionScale;			/*!< Scale for position */
extern bool globalUseIMUUpdates;			/*!< Export flag wheather to use IMU updates or not */
extern xQueueHandle telemetryQueue;			/*!< Export queue to which telemetry updates may be send */
extern xTaskHandle telemetryTask;			/*!< Export this task handle */

/**
 * \brief Type of telemetry update
 */
typedef enum {
	TelemetryUpdate_Source_Odometry = 0,	/*!< Update from odometry - encoders */
	TelemetryUpdate_Source_IMU				/*!< Update from IMU */
} TelemetryUpdate_Source;

/**
 * \brief Struct to hold telemetry updates from various sources. Based on these updates, position and orientation is calculated
 */
typedef struct {
	TelemetryUpdate_Source Source;			/*!< Update source */
	float dX;								/*!< Change in X position */
	float dY;								/*!< Change in Y position */
	float dO;								/*!< Change of orientation angle (radians) */
} TelemetryUpdate_Struct;

/**
 * \brief Struct for holding position and orientation
 */
typedef struct {
	float X;				/*!< X coordinate */
	float Y;				/*!< Y coordinate */
	float O;				/*!< Orientation angle coordinate in radians */
} TelemetryData_Struct;

void TaskTelemetry(void *);	// Task calculating global position and orientation based on all available sources (IMU, odometry, camera, etc. )

/**
 * \brief Sets all OS-related objects for use. It should be called on startup, only once, before scheduler starts
 */
void TaskTelemetryConstructor();

/**
 * \brief Returns current up-to-date telemetry data and saves it in provided structure.
 *
 * This function provides mutual exclusion and data coherency.
 */
void getTelemetry(TelemetryData_Struct *data);

/**
 *  \brief Returns current telemetry data without orientation normalization to +-M_PI
 */
void getTelemetryRaw(TelemetryData_Struct *data);

/**
 * \brief Returns telemetry with scaled position
 */
void getTelemetryScaled(TelemetryData_Struct *data);

/**
 * \brief Returns raw telemetry with scaled position
 */
void getTelemetryRawScaled(TelemetryData_Struct *data);

/**
 *  \brief Returns normalized orientation angle provided as input in radians, output is [-PI, +PI]
 */
float normalizeOrientation(float in);

/**
 *  \brief Perform globalOdometryCorrectionGain scaling by the following procedure with pen down:
 *  1. Drive straight for 10cm
 *  2. Turn left 360*turns - 180 degrees
 *  3. Drive straight for 10cm
 *
 *  This procedure will result in drawing an angle on the ground. By measuring it it will be
 *  possible to calculate new globalOdometryCorrectionGain. Instructions will be printed on
 *  the screen.
 *  @param turns How many turns robot should do. The more turns the more precise measuring will be
 */
void scaleOdometryCorrectionParam(int turns);

#endif /* _TASKTELEMETRY_H_ */
