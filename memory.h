#ifndef _MEMORY_H_
#define _MEMORY_H_

#include <stdbool.h>
#include "compilation.h"

#define CCMEM __attribute__ ((section (".ccm")))

#define INIT_TELEMETRY_PATH "init/telemetry.txt"
#define INIT_MOTOR_CTRL_CUSTOM_PATH "init/mot_ctrl_cust.txt"
#define INIT_MOTOR_CTRL_PID_PATH "init/mot_ctrl_pid.txt"
#define INIT_IMU_PATH "init/imu.txt"
#define INIT_TRAJECTORY_PATH "init/trajectory.txt"
#define INIT_LOGGING_PATH "init/logging.txt"

typedef enum {
	InitTarget_All = 0,
#ifdef USE_CUSTOM_MOTOR_CONTROLLER
	InitTarget_Custom_Motor_Controler = 1,
#else
	InitTarget_PID_Motor_Controler = 2,
#endif
#ifdef FOLLOW_TRAJECTORY
	InitTarget_Trajectory = 3,
#endif
#ifdef USE_IMU_TELEMETRY
	InitTarget_IMU = 4,
#endif
	InitTarget_Telemetry = 5,
	InitTarget_Logging = 6,
} InitTarget_Type;

bool readInit(const InitTarget_Type target);
bool saveConfig(const InitTarget_Type target);

#endif /* _MEMORY_H_ */
