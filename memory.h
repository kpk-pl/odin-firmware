#ifndef _MEMORY_H_
#define _MEMORY_H_

#define CCMEM __attribute__ ((section (".ccm")))

uint8_t saveToNVMemory();
void restoreFromNVMemory();

#endif /* _MEMORY_H_ */
