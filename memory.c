#include <stm32f4xx.h>
#include "memory.h"
#include "compilation.h"
#include "ff.h"
#include <stdlib.h>

#include "TaskMotorCtrl.h"
#include "TaskTrajectory.h"
#include "TaskTelemetry.h"
#include "TaskIMU.h"

typedef struct {
	char* name;
	char* value;
	uint8_t name_len;
} init_entry;

static uint8_t parse_init_line(char* line, init_entry* entry);
static bool interpretEntryTelemetry(const init_entry* entry);
#ifdef USE_CUSTOM_MOTOR_CONTROLLER
static bool interpretEntryCustomMotorController(const init_entry* entry);
#else
static bool interpretEntryPIDMotorController(const init_entry* entry);
#endif
#ifdef USE_IMU_TELEMETRY
static bool interpretEntryIMU(const init_entry* entry);
#endif
#ifdef FOLLOW_TRAJECTORY
static bool interpretEntryTrajectory(const init_entry* entry);
#endif
static bool readInitAll();

bool readInit(const InitTarget_Type target) {
	FIL file;
	bool (*interpretFun)(const init_entry* entry);
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

	char buffer[100];
	init_entry entry;
	bool ret = true;

	while(!f_eof(&file)) {
		f_gets(buffer, 100, &file);
		if (parse_init_line(buffer, &entry) == 0) {
			if (!interpretFun(&entry))
				ret = false;
		}
		else {
			ret = false;
		}
	}

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

bool interpretEntryTelemetry(const init_entry* entry) {
	if (entry == NULL) return false;

	if (strncmp(entry->name, "correction_gain", entry->name_len) == 0) {
		globalOdometryCorrectionGain = strtof(entry->value, NULL);
		return true;
	}

	return false;
}

#ifdef USE_CUSTOM_MOTOR_CONTROLLER
bool interpretEntryCustomMotorController(const init_entry* entry) {
	return true;
}
#else
bool interpretEntryPIDMotorController(const init_entry* entry) {
	return true;
}
#endif

#ifdef USE_IMU_TELEMETRY
bool interpretEntryIMU(const init_entry* entry) {
	return true;
}
#endif

#ifdef FOLLOW_TRAJECTORY
bool interpretEntryTrajectory(const init_entry* entry) {
	return true;
}
#endif

uint8_t parse_init_line(char* line, init_entry* entry) {
	if (line == NULL || entry == NULL) return 1;
	char* eq_sign = strchr(line, '=');
	if (eq_sign == NULL) return 2;
	entry->name = line;
	entry->value = eq_sign+1;
	entry->name_len = eq_sign-line;
	return 0;
}
