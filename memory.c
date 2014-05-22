#include <stm32f4xx.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "memory.h"
#include "compilation.h"
#include "ff.h"

#include "TaskMotorCtrl.h"
#include "TaskTrajectory.h"
#include "TaskTelemetry.h"
#include "TaskIMU.h"
#include "TaskPrintfConsumer.h"

typedef enum {
	Config_Item_Type_Float = 0,
	Config_Item_Type_Uint32,
	Config_Item_Type_Hex32
} Config_Item_Type;

typedef struct {
	Config_Item_Type type;
	volatile void* content;
	const char* name;
	const char* format;
} Config_Item_Struct;

typedef enum {
	IO_Type_Save = 0,
	IO_Type_Read
} IO_Type;

#ifdef USE_IMU_TELEMETRY
static bool readInitIMU(FIL* file);
static bool saveInitIMU(FIL* file);
#endif

static bool readInitAll();
static bool saveInitAll();
static FRESULT openInitFile(InitTarget_Type target, FIL* file, BYTE mode);

static void readConfigItem(const char *buffer, const Config_Item_Struct *item);
static void saveConfigItem(char *buffer, const Config_Item_Struct *item);
static bool IOInitOp(FIL *file, IO_Type type, InitTarget_Type target);

const Config_Item_Struct telemetryConfig[] = {
	{.content = &globalOdometryCorrectionGain, .type = Config_Item_Type_Float, .name = "corr gain", .format = "%.8g"},
	{.content = &globalIMUComplementaryFilterTimeConstant, .type = Config_Item_Type_Float, .name = "imu t const",.format = "%.8g"},
	{.content = &globalUseIMUUpdates, .type = Config_Item_Type_Hex32, .name = "use imu updates", .format = "%lx"}
};

const Config_Item_Struct loggingConfig[] = {
	{.content = &globalLogSettings.bigFlags, .type = Config_Item_Type_Hex32, .name = "big flags", .format = "%lx"},
	{.content = &globalLogSettings.smallFlags, .type = Config_Item_Type_Hex32, .name = "small flags", .format = "%lx"}
};

#ifdef FOLLOW_TRAJECTORY
const Config_Item_Struct trajectoryConfig[] = {
	{.content = &globalTrajectoryControlGains.k_x, .type = Config_Item_Type_Float, .name = "k_x", .format = "%.8g"},
	{.content = &globalTrajectoryControlGains.k, .type = Config_Item_Type_Float, .name = "k", .format = "%.8g"},
	{.content = &globalTrajectoryControlGains.k_s, .type = Config_Item_Type_Float, .name = "k_s", .format = "%.8g"}
};
#endif

const Config_Item_Struct customMotorControllerConfig[] = {
	{.content = &globalLeftMotorParams.forward.K, .type = Config_Item_Type_Float, .name = "K lf", .format = "%.8g"},
	{.content = &globalLeftMotorParams.forward.B, .type = Config_Item_Type_Float, .name = "B lf", .format = "%.8g"},
	{.content = &globalLeftMotorParams.pid2.forward.Kp, .type = Config_Item_Type_Float, .name = "Kp lf", .format = "%.8g"},
	{.content = &globalLeftMotorParams.pid2.forward.Ki, .type = Config_Item_Type_Float, .name = "Ki lf", .format = "%.8g"},
	{.content = &globalLeftMotorParams.pid2.forward.Kd, .type = Config_Item_Type_Float, .name = "Kd lf", .format = "%.8g"},
	{.content = &globalLeftMotorParams.backward.K, .type = Config_Item_Type_Float, .name = "K lb", .format = "%.8g"},
	{.content = &globalLeftMotorParams.backward.B, .type = Config_Item_Type_Float, .name = "B lb", .format = "%.8g"},
	{.content = &globalLeftMotorParams.pid2.backward.Kp, .type = Config_Item_Type_Float, .name = "Kp lb", .format = "%.8g"},
	{.content = &globalLeftMotorParams.pid2.backward.Ki, .type = Config_Item_Type_Float, .name = "Ki lb", .format = "%.8g"},
	{.content = &globalLeftMotorParams.pid2.backward.Kd, .type = Config_Item_Type_Float, .name = "Kd lb", .format = "%.8g"},
	{.content = &globalRightMotorParams.forward.K, .type = Config_Item_Type_Float, .name = "K rf", .format = "%.8g"},
	{.content = &globalRightMotorParams.forward.B, .type = Config_Item_Type_Float, .name = "B rf", .format = "%.8g"},
	{.content = &globalRightMotorParams.pid2.forward.Kp, .type = Config_Item_Type_Float, .name = "Kp rf", .format = "%.8g"},
	{.content = &globalRightMotorParams.pid2.forward.Ki, .type = Config_Item_Type_Float, .name = "Ki rf", .format = "%.8g"},
	{.content = &globalRightMotorParams.pid2.forward.Kd, .type = Config_Item_Type_Float, .name = "Kd rf", .format = "%.8g"},
	{.content = &globalRightMotorParams.backward.K, .type = Config_Item_Type_Float, .name = "K rb", .format = "%.8g"},
	{.content = &globalRightMotorParams.backward.B, .type = Config_Item_Type_Float, .name = "B rb", .format = "%.8g"},
	{.content = &globalRightMotorParams.pid2.backward.Kp, .type = Config_Item_Type_Float, .name = "Kp rb", .format = "%.8g"},
	{.content = &globalRightMotorParams.pid2.backward.Ki, .type = Config_Item_Type_Float, .name = "Ki rb", .format = "%.8g"},
	{.content = &globalRightMotorParams.pid2.backward.Kd, .type = Config_Item_Type_Float, .name = "d rb", .format = "%.8g"},
};

void readConfigItem(const char *buffer, const Config_Item_Struct *item) {
	switch(item->type) {
	case Config_Item_Type_Float:
		*(float*)(item->content) = strtof(buffer, NULL);
		break;
	case Config_Item_Type_Uint32:
		*(uint32_t*)(item->content) = strtoul(buffer, NULL, 10);
		break;
	case Config_Item_Type_Hex32:
		*(uint32_t*)(item->content) = strtoul(buffer, NULL, 16);
		break;
	}
}

void saveConfigItem(char *buffer, const Config_Item_Struct *item) {
	size_t pos = 0;
	switch(item->type) {
	case Config_Item_Type_Float:
		pos = sprintf(buffer, item->format, *(float*)(item->content));
		break;
	case Config_Item_Type_Uint32:
	case Config_Item_Type_Hex32:
		pos = sprintf(buffer, item->format, *(uint32_t*)(item->content));
		break;
	}
	buffer[pos++] = ' ';
	strcpy(buffer+pos, item->name);
}

bool IOInitOp(FIL *file, IO_Type type, InitTarget_Type target) {
	if (file == NULL) return false;

	const Config_Item_Struct *config;
	uint8_t items;
	switch (target) {
	case InitTarget_Telemetry:
		config = telemetryConfig;
		items = sizeof(telemetryConfig)/sizeof(Config_Item_Struct);
		break;
	case InitTarget_Logging:
		config = loggingConfig;
		items = sizeof(loggingConfig)/sizeof(Config_Item_Struct);
		break;
#ifdef FOLLOW_TRAJECTORY
	case InitTarget_Trajectory:
		config = trajectoryConfig;
		items = sizeof(trajectoryConfig)/sizeof(Config_Item_Struct);
		break;
#endif
	case InitTarget_Custom_Motor_Controler:
		config = customMotorControllerConfig;
		items = sizeof(customMotorControllerConfig)/sizeof(Config_Item_Struct);
		break;
#ifdef USE_IMU_TELEMETRY
	case InitTarget_IMU:
		return (type == IO_Type_Save) ? saveInitIMU(file) : readInitIMU(file);
#endif
	default:
		return false;
	}

	uint8_t param;
	char buffer[50];

	if (type == IO_Type_Save) {
		for (param = 0; param < items; ++param) {
			saveConfigItem(buffer, &config[param]);
			f_puts(buffer, file);
			f_puts("\n", file);
		}
	}
	else {
		for (param = 0; param < items && !f_eof(file); ++param) {
			f_gets(buffer, 50, file);
			readConfigItem(buffer, &config[param]);
		}
	}

	return (param == items);
}

bool readInit(const InitTarget_Type target) {
	FIL file;

	if (target == InitTarget_All)
		return readInitAll();

	if (FR_OK != openInitFile(target, &file, FA_OPEN_EXISTING | FA_READ))
		return false;

	bool ret = IOInitOp(&file, IO_Type_Read, target);

	f_close(&file);
	return ret;
}

bool saveConfig(const InitTarget_Type target) {
	FIL file;

	if (target == InitTarget_All)
		return saveInitAll();

	if (FR_OK != openInitFile(target, &file, FA_CREATE_ALWAYS | FA_WRITE))
		return false;

	bool ret = IOInitOp(&file, IO_Type_Save, target);

	f_close(&file);
	return ret;
}

FRESULT openInitFile(InitTarget_Type target, FIL* file, BYTE mode) {
	switch (target) {
	case InitTarget_Telemetry:
		return f_open(file, INIT_TELEMETRY_PATH, mode);
	case InitTarget_Custom_Motor_Controler:
		return f_open(file, INIT_MOTOR_CTRL_CUSTOM_PATH, mode);
#ifdef USE_IMU_TELEMETRY
	case InitTarget_IMU:
		return f_open(file, INIT_IMU_PATH, mode);
#endif
#ifdef FOLLOW_TRAJECTORY
	case InitTarget_Trajectory:
		return f_open(file, INIT_TRAJECTORY_PATH, mode);
#endif
	case InitTarget_Logging:
		return f_open(file, INIT_LOGGING_PATH, mode);
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
	if (!readInit(InitTarget_Custom_Motor_Controler))
		ret = false;
#ifdef FOLLOW_TRAJECTORY
	if (!readInit(InitTarget_Trajectory))
		ret = false;
#endif
#ifdef USE_IMU_TELEMETRY
	if (!readInit(InitTarget_IMU))
		ret = false;
#endif
	if (!readInit(InitTarget_Logging))
		ret = false;
	return ret;
}

bool saveInitAll() {
	bool ret = true;

	// forbidden to use InitTarget_All as this will cause infinite recursion loop and system crash
	if (!saveConfig(InitTarget_Telemetry))
		ret = false;
	if (!saveConfig(InitTarget_Custom_Motor_Controler))
		ret = false;
#ifdef FOLLOW_TRAJECTORY
	if (!saveConfig(InitTarget_Trajectory))
		ret = false;
#endif
#ifdef USE_IMU_TELEMETRY
	if (!saveConfig(InitTarget_IMU))
		ret = false;
#endif
	if (!saveConfig(InitTarget_Logging))
		ret = false;
	return ret;
}

#ifdef USE_IMU_TELEMETRY
bool readInitIMU(FIL* file) {
	if (file == NULL) return false;

	char buffer[25];

	f_gets(buffer, 25, file);
	if (strtod(buffer, NULL) != MAG_IMPROV_DATA_POINTS) return false;

	uint8_t line;
	for (line = 0; !f_eof(file) && line < MAG_IMPROV_DATA_POINTS+1; ++line) {
		f_gets(buffer, 25, file);
		globalMagnetometerImprovData[line] = strtod(buffer, NULL);
	}

	return line == MAG_IMPROV_DATA_POINTS+1;
}

bool saveInitIMU(FIL* file) {
	if (file == NULL) return false;
	char buffer[25];

	snprintf(buffer, 25, "%d n\n", MAG_IMPROV_DATA_POINTS);
	f_puts(buffer, file);

	for (uint8_t i = 0; i<MAG_IMPROV_DATA_POINTS+1; ++i) {
		snprintf(buffer, 25, "%.8g\n", globalMagnetometerImprovData[i]);
		f_puts(buffer, file);
	}

	return true;
}
#endif
