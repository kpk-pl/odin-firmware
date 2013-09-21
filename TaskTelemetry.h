#ifndef _TASKTELEMETRY_H_
#define _TASKTELEMETRY_H_

/* Type of telemetry update */
typedef enum {
	TelemetryUpdate_Source_Odometry = 0,	/*<< Update from odometry - encoders */
	TelemetryUpdate_Source_IMU				/*<< Update from IMU */
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

void TaskTelemetry(void *);	// Task calculating global position and orientation based on all available sources (IMU, odometry, camera, etc. )

#endif /* _TASKTELEMETRY_H_ */
