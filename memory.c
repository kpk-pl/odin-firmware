#include "memory.h"
#include "hardware.h"
#include "hwinterface.h"
#include "compilation.h"

#define EEPROM_BASE					(0x00000)

#define EEPROM_GLOBALS_BASE			(EEPROM_BASE + 0x00000)
#define EEPROM_GLOBAL_LOG_EVENTS	(EEPROM_GLOBALS_BASE + 0x00000)			/* bool - 1b */
#define EEPROM_GLOBAL_LOG_TELEMETRY	(EEPROM_GLOBALS_BASE + 0x00001)			/* bool - 1b */
#define EEPROM_GLOBAL_LOG_SPEED		(EEPROM_GLOBALS_BASE + 0x00002)			/* bool - 1b */


#ifdef USE_CUSTOM_MOTOR_CONTROLLER
#define EEPROM_CONTROLLER_BASE		(EEPROM_BASE + 0x10000)
#define EEPROM_VOLTAGE_CORRECTION	(EEPROM_CONTROLLER_BASE + 0x00000)		/* FunctionalState - 1b */
#define EEPROM_LEFT_THRESHOLD		(EEPROM_CONTROLLER_BASE + 0x00004)		/* float - 4b */
#define EEPROM_RIGHT_THRESHOLD		(EEPROM_CONTROLLER_BASE + 0x00008)		/* float - 4b */
#define EEPROM_LEFT_A				(EEPROM_CONTROLLER_BASE + 0x0000c)		/* float - 4b */
#define EEPROM_RIGHT_A				(EEPROM_CONTROLLER_BASE + 0x00010)		/* float - 4b */
#define EEPROM_LEFT_B				(EEPROM_CONTROLLER_BASE + 0x00014)		/* float - 4b */
#define EEPROM_RIGHT_B				(EEPROM_CONTROLLER_BASE + 0x00018)		/* float - 4b */
#define EEPROM_LEFT_C				(EEPROM_CONTROLLER_BASE + 0x0001c)		/* float - 4b */
#define EEPROM_RIGHT_C				(EEPROM_CONTROLLER_BASE + 0x00020)		/* float - 4b */
#define EEPROM_LEFT_KP				(EEPROM_CONTROLLER_BASE + 0x00024)		/* float - 4b */
#define EEPROM_RIGHT_KP				(EEPROM_CONTROLLER_BASE + 0x00028)		/* float - 4b */
#define EEPROM_LEFT_KI				(EEPROM_CONTROLLER_BASE + 0x0002c)		/* float - 4b */
#define EEPROM_RIGHT_KI				(EEPROM_CONTROLLER_BASE + 0x00030)		/* float - 4b */
#define EEPROM_LEFT_KD				(EEPROM_CONTROLLER_BASE + 0x00034)		/* float - 4b */
#define EEPROM_RIGHT_KD				(EEPROM_CONTROLLER_BASE + 0x00038)		/* float - 4b */
#define EEPROM_LEFT_AT				(EEPROM_CONTROLLER_BASE + 0x0003c)		/* float - 4b */
#define EEPROM_RIGHT_AT				(EEPROM_CONTROLLER_BASE + 0x00040)		/* float - 4b */
#define EEPROM_LEFT_BT				(EEPROM_CONTROLLER_BASE + 0x00044)		/* float - 4b */
#define EEPROM_RIGHT_BT				(EEPROM_CONTROLLER_BASE + 0x00048)		/* float - 4b */
#define EEPROM_LEFT_KPT				(EEPROM_CONTROLLER_BASE + 0x0004c)		/* float - 4b */
#define EEPROM_RIGHT_KPT			(EEPROM_CONTROLLER_BASE + 0x00050)		/* float - 4b */
#define EEPROM_LEFT_KIT				(EEPROM_CONTROLLER_BASE + 0x00054)		/* float - 4b */
#define EEPROM_RIGHT_KIT			(EEPROM_CONTROLLER_BASE + 0x00058)		/* float - 4b */
#define EEPROM_LEFT_KDT				(EEPROM_CONTROLLER_BASE + 0x0005c)		/* float - 4b */
#define EEPROM_RIGHT_KDT			(EEPROM_CONTROLLER_BASE + 0x00060)		/* float - 4b */
#else	/* USE_CUSTOM_MOTOR_CONTROLLER */
#define EEPROM_CONTROLLER_BASE		(EEPROM_BASE + 0x20000)
#define EEPROM_MOTORS_KP			(EEPROM_CONTROLLER_BASE + 0x00000)		/* float - 4b */
#define EEPROM_MOTORS_KI			(EEPROM_CONTROLLER_BASE + 0x00004)		/* float - 4b */
#define EEPROM_MOTORS_KD			(EEPROM_CONTROLLER_BASE + 0x00008)		/* float - 4b */
#endif	/* USE_CUSTOM_MOTOR_CONTROLLER */

#ifdef USE_IMU_TELEMETRY
#define EEPROM_IMU_BASE					(EEPROM_BASE + 0x30000)
#define EEPROM_ODOMETRY_CORRECTION		(EEPROM_IMU_BASE + 0x00000)			/* float - 4b */
// space
#define EEPROM_MAGNETOMETER_IMPROV_DATA	(EEPROM_IMU_BASE + 0x00010)			/* 721 x float - 2884b */
#endif 	/* USE_IMU_TELEMETRY */

void readConfigData() {

}

void updateConfigData() {

}
