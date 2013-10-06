#ifndef _MEMORY_H_
#define _MEMORY_H_

/**
 * Reads the content of eeprom and saves it to global variables
 */
void readConfigData();

/**
 * Writes the global variables to eeprom
 */
void updateConfigData();

#endif /* _MEMORY_H_ */
