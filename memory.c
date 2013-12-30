#include <stm32f4xx.h>
#include "memory.h"
#include "compilation.h"
#include "ff.h"
#include <stdlib.h>

#include "TaskMotorCtrl.h"
#include "TaskTrajectory.h"
#include "TaskTelemetry.h"
#include "TaskIMU.h"

static bool interpretEntryTelemetry(FIL* file);
#ifdef USE_CUSTOM_MOTOR_CONTROLLER
static bool interpretEntryCustomMotorController(FIL* file);
#else
static bool interpretEntryPIDMotorController(FIL* file);
#endif
#ifdef USE_IMU_TELEMETRY
static bool interpretEntryIMU(FIL* file);
#endif
#ifdef FOLLOW_TRAJECTORY
static bool interpretEntryTrajectory(FIL* file);
#endif
static bool readInitAll();

bool readInit(const InitTarget_Type target) {
	FIL file;
	bool (*interpretFun)(FIL *);
	FRESULT fileResult;

	switch(target) {
	case InitTarget_All:
		return readInitAll();
		break;
	case InitTarget_Telemetry:
		interpretFun = interpretEntryTelemetry;
		fileResult = f_open(&file, INIT_TELEMETRY_PATH, FA_OPEN_EXISTING | FA_READ);
		break;
#ifdef USE_CUSTOM_MOTOR_CONTROLLER
	case InitTarget_Custom_Motor_Controler:
		interpretFun = interpretEntryCustomMotorController;
		fileResult = f_open(&file, INIT_MOTOR_CTRL_CUSTOM_PATH, FA_OPEN_EXISTING | FA_READ);
		break;
#else
	case InitTarget_PID_Motor_Controler:
		interpretFun = interpretEntryPIDMotorController;
		fileResult = f_open(&file, INIT_MOTOR_CTRL_PID_PATH, FA_OPEN_EXISTING | FA_READ);
		break;
#endif
#ifdef USE_IMU_TELEMETRY
	case InitTarget_IMU:
		interpretFun = interpretEntryIMU;
		fileResult = f_open(&file, INIT_IMU_PATH, FA_OPEN_EXISTING | FA_READ);
		break;
#endif
#ifdef FOLLOW_TRAJECTORY
	case InitTarget_Trajectory:
		interpretFun = interpretEntryTrajectory;
		fileResult = f_open(&file, INIT_TRAJECTORY_PATH, FA_OPEN_EXISTING | FA_READ);
		break;
#endif
	default:
		return false;
	}

	if (FR_OK != fileResult)
		return false;

	bool ret = interpretFun(&file);

	f_close(&file);
	return ret;
}

bool readInitAll() {
	bool ret = true;

	// forbidden to use InitTarget_All as this will cause infinite recursion loop and system crash
	if (!readInit(InitTarget_Telemetry))
		ret = false;
#ifdef USE_CUSTOM_MOTOR_CONTROLLER
	if (!readInit(InitTarget_Custom_Motor_Controler))
		ret = false;
#else
	if (!readInit(InitTarget_PID_Motor_Controler))
		ret = false;
#endif
#ifdef FOLLOW_TRAJECTORY
	if (!readInit(InitTarget_Trajectory))
		ret = false;
#endif
#ifdef USE_IMU_TELEMETRY
	if (!readInit(InitTarget_IMU))
		ret = false;
#endif

	return ret;
}

bool interpretEntryTelemetry(FIL* file) {
	if (file == NULL) return false;

	char buffer[50];
	uint8_t line;

	for (line = 0; line < 1 && !f_eof(file); ++line) {
		f_gets(buffer, 50, file);
		switch(line) {
		case 0: globalOdometryCorrectionGain = strtof(buffer, NULL); break;
		default: return false;
		}
	}

	return (line == 1);
}

#ifdef USE_CUSTOM_MOTOR_CONTROLLER
bool interpretEntryCustomMotorController(FIL* file) {
	if (file == NULL) return false;

	char buffer[50];
	uint8_t line;

	for (line = 0; line < 20 && !f_eof(file); ++line) {
		f_gets(buffer, 50, file);
		switch(line) {
		case 0: globalLeftMotorParams.forward.K = strtof(buffer, NULL); break;
		case 1: globalLeftMotorParams.forward.B = strtof(buffer, NULL); break;
		case 2: globalLeftMotorParams.pid2.forward.Kp = strtof(buffer, NULL); break;
		case 3: globalLeftMotorParams.pid2.forward.Ki = strtof(buffer, NULL); break;
		case 4: globalLeftMotorParams.pid2.forward.Kd = strtof(buffer, NULL); break;
		case 5: globalLeftMotorParams.backward.K = strtof(buffer, NULL); break;
		case 6: globalLeftMotorParams.backward.B = strtof(buffer, NULL); break;
		case 7: globalLeftMotorParams.pid2.backward.Kp = strtof(buffer, NULL); break;
		case 8: globalLeftMotorParams.pid2.backward.Ki = strtof(buffer, NULL); break;
		case 9: globalLeftMotorParams.pid2.backward.Kd = strtof(buffer, NULL); break;
		case 10: globalRightMotorParams.forward.K = strtof(buffer, NULL); break;
		case 11: globalRightMotorParams.forward.B = strtof(buffer, NULL); break;
		case 12: globalRightMotorParams.pid2.forward.Kp = strtof(buffer, NULL); break;
		case 13: globalRightMotorParams.pid2.forward.Ki = strtof(buffer, NULL); break;
		case 14: globalRightMotorParams.pid2.forward.Kd = strtof(buffer, NULL); break;
		case 15: globalRightMotorParams.backward.K = strtof(buffer, NULL); break;
		case 16: globalRightMotorParams.backward.B = strtof(buffer, NULL); break;
		case 17: globalRightMotorParams.pid2.backward.Kp = strtof(buffer, NULL); break;
		case 18: globalRightMotorParams.pid2.backward.Ki = strtof(buffer, NULL); break;
		case 19: globalRightMotorParams.pid2.backward.Kd = strtof(buffer, NULL); break;
		default: return false;
		}
	}

	return (line == 20);
}
#else
bool interpretEntryPIDMotorController(FIL* file) {
	if (file == NULL) return false;

	char buffer[50];
	uint8_t line;

	for (line = 0; line < 3 && !f_eof(file); ++line) {
		f_gets(buffer, 50, file);
		switch(line) {
		case 0: globalMotorPidKp = strtof(buffer, NULL); break;
		case 1: globalMotorPidKi = strtof(buffer, NULL); break;
		case 2: globalMotorPidKd = strtof(buffer, NULL); break;
		default: return false;
		}
	}

	return (line == 3);
}
#endif

#ifdef USE_IMU_TELEMETRY
bool interpretEntryIMU(FIL* file) {
	return true;
}
#endif

#ifdef FOLLOW_TRAJECTORY
bool interpretEntryTrajectory(FIL* file) {
	if (file == NULL) return false;

	char buffer[50];
	uint8_t line;

	for (line = 0; line < 3 && !f_eof(file); ++line) {
		f_gets(buffer, 50, file);
		switch(line) {
		case 0: globalTrajectoryControlGains.k_x = strtof(buffer, NULL); break;
		case 1: globalTrajectoryControlGains.k = strtof(buffer, NULL); break;
		case 2: globalTrajectoryControlGains.k_s = strtof(buffer, NULL); break;
		default: return false;
		}
	}

	return (line == 3);
}
#endif
