#include <stm32f4xx.h>
#include <stdlib.h>
#include <stdio.h>

#include "memory.h"
#include "compilation.h"
#include "ff.h"

#include "TaskMotorCtrl.h"
#include "TaskTrajectory.h"
#include "TaskTelemetry.h"
#include "TaskIMU.h"

static bool readInitTelemetry(FIL* file);
static bool saveInitTelemetry(FIL* file);
#ifdef USE_CUSTOM_MOTOR_CONTROLLER
static bool readInitCustomMotorController(FIL* file);
static bool saveInitCustomMotorController(FIL* file);
#else
static bool readInitPIDMotorController(FIL* file);
static bool saveInitPICMotorController(FIL* file);
#endif
#ifdef USE_IMU_TELEMETRY
static bool readInitIMU(FIL* file);
static bool saveInitIMU(FIL* file);
#endif
#ifdef FOLLOW_TRAJECTORY
static bool readInitTrajectory(FIL* file);
static bool saveInitTrajectory(FIL* file);
#endif

static bool readInitAll();
static bool saveInitAll();
static FRESULT openInitFile(InitTarget_Type target, FIL* file, BYTE mode);

bool readInit(const InitTarget_Type target) {
	FIL file;
	bool (*initFun)(FIL *);

	switch (target) {
	case InitTarget_All:
		return readInitAll();
	case InitTarget_Telemetry:
		initFun = readInitTelemetry;
		break;
#ifdef USE_CUSTOM_MOTOR_CONTROLLER
	case InitTarget_Custom_Motor_Controler:
		initFun = readInitCustomMotorController;
		break;
#else
	case InitTarget_PID_Motor_Controler:
		initFun = readInitPIDMotorController;
		break;
#endif
#ifdef USE_IMU_TELEMETRY
	case InitTarget_IMU:
		initFun = readInitIMU;
		break;
#endif
#ifdef FOLLOW_TRAJECTORY
	case InitTarget_Trajectory:
		initFun = readInitTrajectory;
		break;
#endif
	default:
		return false;
	}

	if (FR_OK != openInitFile(target, &file, FA_OPEN_EXISTING | FA_READ))
		return false;

	bool ret = initFun(&file);

	f_close(&file);
	return ret;
}

bool saveConfig(const InitTarget_Type target) {
	FIL file;
	bool (*saveFun)(FIL *);

	switch (target) {
	case InitTarget_All:
		return saveInitAll();
	case InitTarget_Telemetry:
		saveFun = saveInitTelemetry;
		break;
#ifdef USE_CUSTOM_MOTOR_CONTROLLER
	case InitTarget_Custom_Motor_Controler:
		saveFun = saveInitCustomMotorController;
		break;
#else
	case InitTarget_PID_Motor_Controler:
		saveFun = saveInitPIDMotorController;
		break;
#endif
#ifdef USE_IMU_TELEMETRY
	case InitTarget_IMU:
		saveFun = saveInitIMU;
		break;
#endif
#ifdef FOLLOW_TRAJECTORY
	case InitTarget_Trajectory:
		saveFun = saveInitTrajectory;
		break;
#endif
	default:
		return false;
	}

	if (FR_OK != openInitFile(target, &file, FA_CREATE_ALWAYS | FA_WRITE))
		return false;

	bool ret = saveFun(&file);

	f_close(&file);
	return ret;
}

FRESULT openInitFile(InitTarget_Type target, FIL* file, BYTE mode) {
	switch (target) {
	case InitTarget_Telemetry:
		return f_open(file, INIT_TELEMETRY_PATH, mode);
#ifdef USE_CUSTOM_MOTOR_CONTROLLER
	case InitTarget_Custom_Motor_Controler:
		return f_open(file, INIT_MOTOR_CTRL_CUSTOM_PATH, mode);
#else
	case InitTarget_PID_Motor_Controler:
		return f_open(file, INIT_MOTOR_CTRL_PID_PATH, mode);
#endif
#ifdef USE_IMU_TELEMETRY
	case InitTarget_IMU:
		return f_open(file, INIT_IMU_PATH, mode);
#endif
#ifdef FOLLOW_TRAJECTORY
	case InitTarget_Trajectory:
		return f_open(file, INIT_TRAJECTORY_PATH, mode);
#endif
	default:
		file = NULL;
		return FR_INVALID_OBJECT;
	}
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

bool saveInitAll() {
	bool ret = true;

	// forbidden to use InitTarget_All as this will cause infinite recursion loop and system crash
	if (!saveConfig(InitTarget_Telemetry))
		ret = false;
#ifdef USE_CUSTOM_MOTOR_CONTROLLER
	if (!saveConfig(InitTarget_Custom_Motor_Controler))
		ret = false;
#else
	if (!saveConfig(InitTarget_PID_Motor_Controler))
		ret = false;
#endif
#ifdef FOLLOW_TRAJECTORY
	if (!saveConfig(InitTarget_Trajectory))
		ret = false;
#endif
#ifdef USE_IMU_TELEMETRY
	if (!saveConfig(InitTarget_IMU))
		ret = false;
#endif

	return ret;
}

bool readInitTelemetry(FIL* file) {
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

bool saveInitTelemetry(FIL* file) {
	if (file == NULL) return false;

	char buffer[20];
	snprintf(buffer, 20, "%.8g", globalOdometryCorrectionGain);

	f_puts(buffer, file); f_puts(" correction gain\n", file);

	return true;
}

#ifdef USE_CUSTOM_MOTOR_CONTROLLER
bool readInitCustomMotorController(FIL* file) {
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

bool saveInitCustomMotorController(FIL* file) {
	if (file == NULL) return false;

	char buffer[20];

	for (uint8_t line = 0; line < 20; ++line) {
		switch (line) {
			case 0:  snprintf(buffer, 20, "%.8g", globalLeftMotorParams.forward.K); break;
			case 1:  snprintf(buffer, 20, "%.8g", globalLeftMotorParams.forward.B); break;
			case 2:  snprintf(buffer, 20, "%.8g", globalLeftMotorParams.pid2.forward.Kp); break;
			case 3:  snprintf(buffer, 20, "%.8g", globalLeftMotorParams.pid2.forward.Ki); break;
			case 4:  snprintf(buffer, 20, "%.8g", globalLeftMotorParams.pid2.forward.Kd); break;
			case 5:  snprintf(buffer, 20, "%.8g", globalLeftMotorParams.backward.K); break;
			case 6:  snprintf(buffer, 20, "%.8g", globalLeftMotorParams.backward.B); break;
			case 7:  snprintf(buffer, 20, "%.8g", globalLeftMotorParams.pid2.backward.Kp); break;
			case 8:  snprintf(buffer, 20, "%.8g", globalLeftMotorParams.pid2.backward.Ki); break;
			case 9:  snprintf(buffer, 20, "%.8g", globalLeftMotorParams.pid2.backward.Kd); break;
			case 10: snprintf(buffer, 20, "%.8g", globalRightMotorParams.forward.K); break;
			case 11: snprintf(buffer, 20, "%.8g", globalRightMotorParams.forward.B); break;
			case 12: snprintf(buffer, 20, "%.8g", globalRightMotorParams.pid2.forward.Kp); break;
			case 13: snprintf(buffer, 20, "%.8g", globalRightMotorParams.pid2.forward.Ki); break;
			case 14: snprintf(buffer, 20, "%.8g", globalRightMotorParams.pid2.forward.Kd); break;
			case 15: snprintf(buffer, 20, "%.8g", globalRightMotorParams.backward.K); break;
			case 16: snprintf(buffer, 20, "%.8g", globalRightMotorParams.backward.B); break;
			case 17: snprintf(buffer, 20, "%.8g", globalRightMotorParams.pid2.backward.Kp); break;
			case 18: snprintf(buffer, 20, "%.8g", globalRightMotorParams.pid2.backward.Ki); break;
			case 19: snprintf(buffer, 20, "%.8g", globalRightMotorParams.pid2.backward.Kd); break;
			default: return false;
		}

		f_puts(buffer, file);
		f_puts(line < 10 ? " left" : " right", file);
		f_puts((line/5) % 2 ? " backward" : " forward", file);
		switch (line%5) {
			case 0: f_puts(" K\n", file); break;
			case 1: f_puts(" B\n", file); break;
			case 2: f_puts(" Kp\n", file); break;
			case 3: f_puts(" Ki\n", file); break;
			case 4: f_puts(" Kd\n", file); break;
		}
	}

	return true;
}
#else
bool readInitPIDMotorController(FIL* file) {
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

bool saveInitPIDMotorController(FIL* file) {
	if (file == NULL) return false;

	char buffer[25];

	snprintf(buffer, 20, "%.8g Kp\n", globalMotorPidKp);
	f_puts(buffer, file);
	snprintf(buffer, 20, "%.8g Ki\n", globalMotorPidKi);
	f_puts(buffer, file);
	snprintf(buffer, 20, "%.8g Kd\n", globalMotorPidKd);
	f_puts(buffer, file);

	return true;
}
#endif

#ifdef USE_IMU_TELEMETRY
bool readInitIMU(FIL* file) {
	return true;
}

bool saveInitIMU(FIL* file) {
	return true;
}
#endif

#ifdef FOLLOW_TRAJECTORY
bool readInitTrajectory(FIL* file) {
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

bool saveInitTrajectory(FIL* file) {
	if (file == NULL) return false;

	char buffer[25];

	snprintf(buffer, 20, "%.8g k_x\n", globalTrajectoryControlGains.k_x);
	f_puts(buffer, file);
	snprintf(buffer, 20, "%.8g k\n", globalTrajectoryControlGains.k);
	f_puts(buffer, file);
	snprintf(buffer, 20, "%.8g k_s\n", globalTrajectoryControlGains.k_s);
	f_puts(buffer, file);

	return true;
}
#endif
