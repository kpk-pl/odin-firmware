#ifndef _TASKTELEMETRY_H_
#define _TASKTELEMETRY_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <stdbool.h>

extern volatile float globalCameraTransmissionDelayMs; 		/*!< Export constant transmission delay for camera time calculations */
extern volatile float globalCameraTelemetryFilterConstant; 	/*!< Export complementary time constant for camera odometry */
extern volatile float globalOdometryCorrectionGain;			/*!< Export the parameter that corrects odometry while turning */
extern volatile float globalPositionScale;					/*!< Scale for position */
extern xQueueHandle telemetryQueue;			/*!< Export queue to which telemetry updates may be send */
extern xTaskHandle telemetryTask;			/*!< Export this task handle */

/**
 * \brief Type of telemetry update
 */
typedef enum {
	TelemetryUpdate_Source_Odometry = 0,	/*!< Update from odometry - encoders */
	TelemetryUpdate_Source_Camera,			/*!< Update from radio camera */
} TelemetryUpdate_Source;

/**
 * \brief Struct to hold telemetry updates from various sources. Based on these updates, position and orientation is calculated
 */
typedef struct {
	union {
		struct {
			float dX;						/*!< Change in X position */
			float dY;						/*!< Change in Y position */
			float dO;						/*!< Change of orientation angle (radians) */
		};
		uint8_t Data;						/*!< Raw access to data */
	};
	portTickType Timestamp;					/*!< Local timestamp of this update */
	TelemetryUpdate_Source Source;			/*!< Update source */
} TelemetryUpdate_Struct;

/**
 * \brief Struct for holding position and orientation
 */
typedef struct {
	float X;				/*!< X coordinate */
	float Y;				/*!< Y coordinate */
	float O;				/*!< Orientation angle coordinate in radians */
} TelemetryData_Struct;

/**
 *
 */
typedef enum {
	TelemetryStyle_Raw = 0, 		/*!< Raw telemetry data without any scaling or orientation normalization */
	TelemetryStyle_Scaled = 1,		/*!< Coordinates scaled, but orientation not normalized */
	TelemetryStyle_Normalized = 2,	/*!< Orientation normalized, coordinates not scaled */
	TelemetryStyle_Common = 3		/*!< Orientation normalized, coordinates scaled */
} TelemetryStyle_Type;

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
void getTelemetry(TelemetryData_Struct *data, TelemetryStyle_Type style);

/**
 * Checks if robot moved since last reset
 * @retval true if robot moved, false otherwise
 */
bool movedSinceReset();

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

/**
 * \brief ISR handler for VSYNC interrupt arrival
 * Called from interrupt context
 */
void radioCameraVSYNCHandler();

#endif /* _TASKTELEMETRY_H_ */
