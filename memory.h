#ifndef _MEMORY_H_
#define _MEMORY_H_

#include <stdbool.h>
#include "compilation.h"

#define CCMEM __attribute__ ((section (".ccm")))

#define INIT_TELEMETRY_PATH "init/telemetry.txt"
#define INIT_MOTOR_CTRL_CUSTOM_PATH "init/mot_ctrl_cust.txt"
#define INIT_IMU_PATH "init/imu.txt"
#define INIT_TRAJECTORY_PATH "init/trajectory.txt"
#define INIT_LOGGING_PATH "init/logging.txt"

typedef enum {
	InitTarget_All = 0,
	InitTarget_Custom_Motor_Controler = 1,
	InitTarget_Trajectory = 2,
#ifdef USE_IMU_TELEMETRY
	InitTarget_IMU = 3,
#endif
	InitTarget_Telemetry = 4,
	InitTarget_Logging = 5,
} InitTarget_Type;

bool readInit(const InitTarget_Type target);
bool saveConfig(const InitTarget_Type target);

#endif /* _MEMORY_H_ */
