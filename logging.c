#include <string.h>
#include "logging.h"

typedef struct {
	char *prefix;
	size_t length;
} Prefix_Info_Struct;

/* Prefix for all kinds of logs - numbering is strictly correlated with Log_Type numbering */
static const Prefix_Info_Struct logPrefix[] = {
	{"Log", 3},
	{"Error", 5},
	{"Debug", 5},
	{"RC5", 3},
	{"Speed", 5},
	{"Tel", 3},
	{"IMU", 3},
	{"Drive", 5}
};

#define EVENT_LOGS_MASK ((1<<3) | (1<<6))

bool isLogEnabled(const volatile Log_Settings_Struct *settings, const Log_Type type) {
	/* If all logs disabled */
	if (settings->enableAll == false)
		return false;
	/* If event logs disabled and log is of event type */
	if (settings->enableEvents == false && ((1<<type) & EVENT_LOGS_MASK))
		return false;
	/* If specific type of logs is disabled */
	if (!(settings->smallFlags & (1<<type)))
		return false;

	return true;
}

/*
 * Prefix consists of:
 * left bracket '['
 * string
 * right bracket ']'
 * and space ' '
 */
size_t getLogPrefixLength(Log_Type type) {
	return logPrefix[(uint8_t)type].length + 3;
}

char* prepareLogMessage(char *msg, Log_Type type) {
	*msg = '[';
	msg++;
	memcpy(msg, logPrefix[(uint8_t)type].prefix, logPrefix[(uint8_t)type].length);
	msg += logPrefix[(uint8_t)type].length;
	memcpy(msg, "] ", 2);
	return msg+2;
}
