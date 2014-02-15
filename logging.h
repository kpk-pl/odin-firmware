#ifndef _LOGGING_H_
#define _LOGGING_H_

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

/**
 * Type of log. Numbering is used as indexes in arrays and other numerical operations. Treat as define statements
 * Maximum 32 fields
 */
typedef enum {
	Log_Type_Log = 0,	/*!< Log with no specific type */
	Log_Type_Error,		/*!< Error of unspecified kind */
	Log_Type_Debug,		/*!< Debug log of unspecified kind */
	Log_Type_RC5, 		/*!< Log from RC5 task - EVENT */
	Log_Type_Speed,		/*!< Speed logging */
	Log_Type_Telemetry, /*!< Logging telemetry aka position */
	Log_Type_IMU,		/*!< Logging IMU - EVENT*/
	Log_Type_Drive,		/*!< Log from taskDrive - EVENT */
} Log_Type;

/**
 * Holder for all log settings used
 */
typedef struct {
	union {
		struct {						/*!< This struct contains indivisible fields, may be masked by flags from the second struct */
			bool enableLog 		: 1;	/*!< Enable flag for "log" logs - Log_Type_None */
			bool enableError 	: 1;	/*!< Enable flag for error logs - Log_Type_Error */
			bool enableDebug	: 1;	/*!< Enable flag for debug logs - Log_Type_Debug */
			bool enableRC5		: 1;	/*!< Enable flag for RC5 logs - Log_Type_RC5 */
			bool enableSpeed	: 1;
			bool enableTelemetry :1;
			bool enableIMU		: 1;
			bool enableDrive	: 1;
		};
		uint32_t smallFlags;
	};
	union {
		struct { 						/*!< This struct contains flags that enable specific masks */
			bool enableAll		: 1;	/*!< Enable flag for all logging */
			bool enableEvents	: 1;	/*!< Enable flag for events logs */
		};
		uint32_t bigFlags;
	};
} Log_Settings_Struct;

/**
 * Checks if specific type of log is enabled in settings
 * @param settings Current log settings
 * @param type Log type to check
 * @retval true if log is enabled in settings, false otherwise
 */
bool isLogEnabled(const volatile Log_Settings_Struct *settings, const Log_Type type);

/**
 * Prepares given message inserting appropriate prefix at the beginning.
 * @param msg Pointer to the beginning of string that will contain log text. Must be of appropriate length
 * @param type Type of log to prepare
 * @retval Pointer to the first character in msg that should be written with log text. Before that is log prefix
 */
char* prepareLogMessage(char *msg, Log_Type type);

/**
 * Returns the length of the prefix that will be appended to log of specified type.
 * This function should be used to allocate enough space for prefix and log content
 * @param type Type of log to check
 * @retval Length of prefix for the specified type of log
 */
size_t getLogPrefixLength(Log_Type type);

#endif /* _LOGGING_H_ */
