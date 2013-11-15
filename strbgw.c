#include "strbgw.h"

size_t strbgw(const char *str, const char *bg) {
	size_t retval = 0;
	while (*str && *bg && *str == *bg) { // both not '\0'
		retval++; str++; bg++;
	}
	if (*bg) return retval+1;
	else return 0;
}

size_t strnbgw(const char *str, const char *bg, const size_t n) {
	size_t retval = 0;
	while (retval < n && *str && *bg && *str == *bg) { // both not '\0'
		retval++; str++; bg++;
	}
	if (retval == n && *bg) return 1; // strlen(bg) < n
	else if (*bg) return retval+1;
	else return 0;
}

size_t cmatch(const char *command, const char *input, const size_t shortest) {
	if (strncmp(command, input, shortest) != 0) return 0; // beginning does not match
	command += shortest;
	input += shortest;
	while (*input && *command && *input == *command) {
		input++;
		command++;
	}
	if (*input == '\0') return 1;
	return 0;
}
