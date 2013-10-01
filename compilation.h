#ifndef _ODIN_COMPILATION_H_
#define _ODIN_COMPILATION_H_

//#define USE_CUSTOM_MOTOR_CONTROLLER		/*!< If defined then custom controller will be used, else PID */
//#define USE_IMU_TELEMETRY				/*!< If program should use IMU data for telemetry, else disable anything IMU-related */
#define FOLLOW_TRAJECTORY				/*!< If defined then robot will use trajectory points as source, otherwise driving commands will be available */
#define DRIVE_COMMANDS					/*!< If defined then drive commands will be available */

#ifdef FOLLOW_TRAJECTORY
#define COMPILE_CIRCULAR_BUFFER			/*!< If defined then circular buffer will be compiled with not zero size */
#endif

#endif /* _ODIN_COMPILATION_H */
