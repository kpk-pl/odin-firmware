#include <stm32f4xx.h>
#include "memory.h"

#define MEM_FLASH_START_ADDR 0x080E0000 // sector 11 - 128kB length
#define MEM_FLASH_LENGTH	 0x00020000 // 128kB

#define MEM_NUM0_ADDR		 (MEM_FLASH_START_ADDR)
#define MEM_NUM1_ADDR		 (MEM_FLASH_START_ADDR + 4)
#define MEM_NUM2_ADDR		 (MEM_FLASH_START_ADDR + 8)
#define MEM_NUM3_ADDR		 (MEM_FLASH_START_ADDR + 12)

float num0 = 12345788;
uint16_t num1 = 5421;
uint8_t num2 = 43;
uint32_t num3 = 4513547;

uint8_t saveToNVMemory() {
	FLASH_Unlock();

	if (FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3) != FLASH_COMPLETE) return 1;

	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

	if (FLASH_ProgramWord(MEM_NUM0_ADDR, num0) != FLASH_COMPLETE) return 2;
	if (FLASH_ProgramWord(MEM_NUM1_ADDR, num1) != FLASH_COMPLETE) return 2;
	if (FLASH_ProgramWord(MEM_NUM2_ADDR, num2) != FLASH_COMPLETE) return 2;
	if (FLASH_ProgramWord(MEM_NUM3_ADDR, num3) != FLASH_COMPLETE) return 2;

	FLASH_Lock();

	return 0;
}

void restoreFromNVMemory() {
	num0 = *(__IO uint32_t*)MEM_NUM0_ADDR;
	num1 = *(__IO uint32_t*)MEM_NUM1_ADDR;
	num2 = *(__IO uint32_t*)MEM_NUM2_ADDR;
	num3 = *(__IO uint32_t*)MEM_NUM3_ADDR;
}
