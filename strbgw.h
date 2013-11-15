#ifndef _STRBGW_H_
#define _STRBGW_H_

#include <string.h>

/**
 * Checks if given string begins with another string.
 * @param str Null-terminated C-string to be checked
 * @param bg Null-terminared C-string being the serched beginning
 * @retval Number of characters that matches to each-other plus one, 0 if the whole string bg matches the beginning of str.
 */
size_t strbgw(const char *str, const char *bg);

/**
 * Checks if given string begins with another string.
 * @param str Null-terminated C-string to be checked
 * @param bg Null-terminared C-string being the serched beginning
 * @param n Maximum number of characters to search from str
 * @retval Number of characters that matches to each-other plus one, 0 if the whole string bg matches the beginning of str.
 * If bg is longer than n then 1 is returned immediately
 * If bg is longer than str then 1 is returned immediately
 */
size_t strnbgw(const char *str, const char *bg, const size_t n);

/**
 * Checks if given command matches
 * @param command Full command name
 * @param imput Command to check
 * @param Number of characters that must be present in command to be recognized
 * @result 0 if Command matches, 1 otherwise
 */
size_t cmatch(const char *command, const char *input, const size_t shortest);

#endif /* _STRBGW_H_ */
