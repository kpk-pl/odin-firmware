#include <stm32f4xx.h>
#include "memory.h"
#include "compilation.h"

#include "TaskMotorCtrl.h"

#define MEM_FLASH_START_ADDR 0x080E0000 // sector 11 - 128kB length
#define MEM_FLASH_LENGTH	 0x00020000 // 128kB

#define TO_SAVE_FORMAT(var) *(uint32_t*)(&(var))

#define MEM_MOTOR_CTRL_BASE_ADDR 		(MEM_FLASH_START_ADDR + 0x00000000)

#define MEM_MOTOR_CTRL_CUSTOM_LEFT_FWD_K		(MEM_MOTOR_CTRL_BASE_ADDR + 0x00000000) // float
#define MEM_MOTOR_CTRL_CUSTOM_LEFT_FWD_B		(MEM_MOTOR_CTRL_BASE_ADDR + 0x00000004) // float
#define MEM_MOTOR_CTRL_CUSTOM_LEFT_FWD_KP		(MEM_MOTOR_CTRL_BASE_ADDR + 0x00000008) // float
#define MEM_MOTOR_CTRL_CUSTOM_LEFT_FWD_KI		(MEM_MOTOR_CTRL_BASE_ADDR + 0x0000000C) // float
#define MEM_MOTOR_CTRL_CUSTOM_LEFT_FWD_KD		(MEM_MOTOR_CTRL_BASE_ADDR + 0x00000010) // float
#define MEM_MOTOR_CTRL_CUSTOM_LEFT_BCK_K		(MEM_MOTOR_CTRL_BASE_ADDR + 0x00000014) // float
#define MEM_MOTOR_CTRL_CUSTOM_LEFT_BCK_B		(MEM_MOTOR_CTRL_BASE_ADDR + 0x00000018) // float
#define MEM_MOTOR_CTRL_CUSTOM_LEFT_BCK_KP		(MEM_MOTOR_CTRL_BASE_ADDR + 0x0000001C) // float
#define MEM_MOTOR_CTRL_CUSTOM_LEFT_BCK_KI		(MEM_MOTOR_CTRL_BASE_ADDR + 0x00000020) // float
#define MEM_MOTOR_CTRL_CUSTOM_LEFT_BCK_KD		(MEM_MOTOR_CTRL_BASE_ADDR + 0x00000024) // float
#define MEM_MOTOR_CTRL_CUSTOM_RIGHT_FWD_K		(MEM_MOTOR_CTRL_BASE_ADDR + 0x00000028) // float
#define MEM_MOTOR_CTRL_CUSTOM_RIGHT_FWD_B		(MEM_MOTOR_CTRL_BASE_ADDR + 0x0000002C) // float
#define MEM_MOTOR_CTRL_CUSTOM_RIGHT_FWD_KP		(MEM_MOTOR_CTRL_BASE_ADDR + 0x00000030) // float
#define MEM_MOTOR_CTRL_CUSTOM_RIGHT_FWD_KI		(MEM_MOTOR_CTRL_BASE_ADDR + 0x00000034) // float
#define MEM_MOTOR_CTRL_CUSTOM_RIGHT_FWD_KD		(MEM_MOTOR_CTRL_BASE_ADDR + 0x00000038) // float
#define MEM_MOTOR_CTRL_CUSTOM_RIGHT_BCK_K		(MEM_MOTOR_CTRL_BASE_ADDR + 0x0000003C) // float
#define MEM_MOTOR_CTRL_CUSTOM_RIGHT_BCK_B		(MEM_MOTOR_CTRL_BASE_ADDR + 0x00000040) // float
#define MEM_MOTOR_CTRL_CUSTOM_RIGHT_BCK_KP		(MEM_MOTOR_CTRL_BASE_ADDR + 0x00000044) // float
#define MEM_MOTOR_CTRL_CUSTOM_RIGHT_BCK_KI		(MEM_MOTOR_CTRL_BASE_ADDR + 0x00000048) // float
#define MEM_MOTOR_CTRL_CUSTOM_RIGHT_BCK_KD		(MEM_MOTOR_CTRL_BASE_ADDR + 0x0000004C) // float

#define MEM_MOTOR_CTRL_PID_KP					(MEM_MOTOR_CTRL_BASE_ADDR + 0x00000080) // float
#define MEM_MOTOR_CTRL_PID_KI					(MEM_MOTOR_CTRL_BASE_ADDR + 0x00000084) // float
#define MEM_MOTOR_CTRL_PID_KD					(MEM_MOTOR_CTRL_BASE_ADDR + 0x00000088) // float

uint8_t saveToNVMemory() {
	FLASH_Unlock();

	if (FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3) != FLASH_COMPLETE) return 1;

	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

#ifdef USE_CUSTOM_MOTOR_CONTROLLER
	if (FLASH_ProgramWord(MEM_MOTOR_CTRL_CUSTOM_LEFT_FWD_K, TO_SAVE_FORMAT(globalLeftMotorParams.forward.K)) != FLASH_COMPLETE) return 2;
	if (FLASH_ProgramWord(MEM_MOTOR_CTRL_CUSTOM_LEFT_FWD_B, TO_SAVE_FORMAT(globalLeftMotorParams.forward.B)) != FLASH_COMPLETE) return 2;
	if (FLASH_ProgramWord(MEM_MOTOR_CTRL_CUSTOM_LEFT_FWD_KP, TO_SAVE_FORMAT(globalLeftMotorParams.pid2.forward.Kp)) != FLASH_COMPLETE) return 2;
	if (FLASH_ProgramWord(MEM_MOTOR_CTRL_CUSTOM_LEFT_FWD_KI, TO_SAVE_FORMAT(globalLeftMotorParams.pid2.forward.Ki)) != FLASH_COMPLETE) return 2;
	if (FLASH_ProgramWord(MEM_MOTOR_CTRL_CUSTOM_LEFT_FWD_KD, TO_SAVE_FORMAT(globalLeftMotorParams.pid2.forward.Kd)) != FLASH_COMPLETE) return 2;
	if (FLASH_ProgramWord(MEM_MOTOR_CTRL_CUSTOM_LEFT_BCK_K, TO_SAVE_FORMAT(globalLeftMotorParams.backward.K)) != FLASH_COMPLETE) return 2;
	if (FLASH_ProgramWord(MEM_MOTOR_CTRL_CUSTOM_LEFT_BCK_B, TO_SAVE_FORMAT(globalLeftMotorParams.backward.B)) != FLASH_COMPLETE) return 2;
	if (FLASH_ProgramWord(MEM_MOTOR_CTRL_CUSTOM_LEFT_BCK_KP, TO_SAVE_FORMAT(globalLeftMotorParams.pid2.backward.Kp)) != FLASH_COMPLETE) return 2;
	if (FLASH_ProgramWord(MEM_MOTOR_CTRL_CUSTOM_LEFT_BCK_KI, TO_SAVE_FORMAT(globalLeftMotorParams.pid2.backward.Ki)) != FLASH_COMPLETE) return 2;
	if (FLASH_ProgramWord(MEM_MOTOR_CTRL_CUSTOM_LEFT_BCK_KD, TO_SAVE_FORMAT(globalLeftMotorParams.pid2.backward.Kd)) != FLASH_COMPLETE) return 2;
	if (FLASH_ProgramWord(MEM_MOTOR_CTRL_CUSTOM_RIGHT_FWD_K, TO_SAVE_FORMAT(globalRightMotorParams.forward.K)) != FLASH_COMPLETE) return 2;
	if (FLASH_ProgramWord(MEM_MOTOR_CTRL_CUSTOM_RIGHT_FWD_B, TO_SAVE_FORMAT(globalRightMotorParams.forward.B)) != FLASH_COMPLETE) return 2;
	if (FLASH_ProgramWord(MEM_MOTOR_CTRL_CUSTOM_RIGHT_FWD_KP, TO_SAVE_FORMAT(globalRightMotorParams.pid2.forward.Kp)) != FLASH_COMPLETE) return 2;
	if (FLASH_ProgramWord(MEM_MOTOR_CTRL_CUSTOM_RIGHT_FWD_KI, TO_SAVE_FORMAT(globalRightMotorParams.pid2.forward.Ki)) != FLASH_COMPLETE) return 2;
	if (FLASH_ProgramWord(MEM_MOTOR_CTRL_CUSTOM_RIGHT_FWD_KD, TO_SAVE_FORMAT(globalRightMotorParams.pid2.forward.Kd)) != FLASH_COMPLETE) return 2;
	if (FLASH_ProgramWord(MEM_MOTOR_CTRL_CUSTOM_RIGHT_BCK_K, TO_SAVE_FORMAT(globalRightMotorParams.backward.K)) != FLASH_COMPLETE) return 2;
	if (FLASH_ProgramWord(MEM_MOTOR_CTRL_CUSTOM_RIGHT_BCK_B, TO_SAVE_FORMAT(globalRightMotorParams.backward.B)) != FLASH_COMPLETE) return 2;
	if (FLASH_ProgramWord(MEM_MOTOR_CTRL_CUSTOM_RIGHT_BCK_KP, TO_SAVE_FORMAT(globalRightMotorParams.pid2.backward.Kp)) != FLASH_COMPLETE) return 2;
	if (FLASH_ProgramWord(MEM_MOTOR_CTRL_CUSTOM_RIGHT_BCK_KI, TO_SAVE_FORMAT(globalRightMotorParams.pid2.backward.Ki)) != FLASH_COMPLETE) return 2;
	if (FLASH_ProgramWord(MEM_MOTOR_CTRL_CUSTOM_RIGHT_BCK_KD, TO_SAVE_FORMAT(globalRightMotorParams.pid2.backward.Kd)) != FLASH_COMPLETE) return 2;
#else
	if (FLASH_ProgramWord(MEM_MOTOR_CTRL_PID_KP, TO_SAVE_FORMAT(globalMotorPidKp)) != FLASH_COMPLETE) return 2;
	if (FLASH_ProgramWord(MEM_MOTOR_CTRL_PID_KI, TO_SAVE_FORMAT(globalMotorPidKi)) != FLASH_COMPLETE) return 2;
	if (FLASH_ProgramWord(MEM_MOTOR_CTRL_PID_KD, TO_SAVE_FORMAT(globalMotorPidKd)) != FLASH_COMPLETE) return 2;
#endif

	FLASH_Lock();

	return 0;
}

void restoreFromNVMemory() {
#ifdef USE_CUSTOM_MOTOR_CONTROLLER
	globalLeftMotorParams.forward.K = *(__IO float*)MEM_MOTOR_CTRL_CUSTOM_LEFT_FWD_K;
	globalLeftMotorParams.forward.B = *(__IO float*)MEM_MOTOR_CTRL_CUSTOM_LEFT_FWD_B;
	globalLeftMotorParams.pid2.forward.Kp = *(__IO float*)MEM_MOTOR_CTRL_CUSTOM_LEFT_FWD_KP;
	globalLeftMotorParams.pid2.forward.Ki = *(__IO float*)MEM_MOTOR_CTRL_CUSTOM_LEFT_FWD_KI;
	globalLeftMotorParams.pid2.forward.Kd = *(__IO float*)MEM_MOTOR_CTRL_CUSTOM_LEFT_FWD_KD;
	globalLeftMotorParams.backward.K = *(__IO float*)MEM_MOTOR_CTRL_CUSTOM_LEFT_BCK_K;
	globalLeftMotorParams.backward.B = *(__IO float*)MEM_MOTOR_CTRL_CUSTOM_LEFT_BCK_B;
	globalLeftMotorParams.pid2.backward.Kp = *(__IO float*)MEM_MOTOR_CTRL_CUSTOM_LEFT_BCK_KP;
	globalLeftMotorParams.pid2.backward.Ki = *(__IO float*)MEM_MOTOR_CTRL_CUSTOM_LEFT_BCK_KI;
	globalLeftMotorParams.pid2.backward.Kd = *(__IO float*)MEM_MOTOR_CTRL_CUSTOM_LEFT_BCK_KD;
	globalRightMotorParams.forward.K = *(__IO float*)MEM_MOTOR_CTRL_CUSTOM_RIGHT_FWD_K;
	globalRightMotorParams.forward.B = *(__IO float*)MEM_MOTOR_CTRL_CUSTOM_RIGHT_FWD_B;
	globalRightMotorParams.pid2.forward.Kp = *(__IO float*)MEM_MOTOR_CTRL_CUSTOM_RIGHT_FWD_KP;
	globalRightMotorParams.pid2.forward.Ki = *(__IO float*)MEM_MOTOR_CTRL_CUSTOM_RIGHT_FWD_KI;
	globalRightMotorParams.pid2.forward.Kd = *(__IO float*)MEM_MOTOR_CTRL_CUSTOM_RIGHT_FWD_KD;
	globalRightMotorParams.backward.K = *(__IO float*)MEM_MOTOR_CTRL_CUSTOM_RIGHT_BCK_K;
	globalRightMotorParams.backward.B = *(__IO float*)MEM_MOTOR_CTRL_CUSTOM_RIGHT_BCK_B;
	globalRightMotorParams.pid2.backward.Kp = *(__IO float*)MEM_MOTOR_CTRL_CUSTOM_RIGHT_BCK_KP;
	globalRightMotorParams.pid2.backward.Ki = *(__IO float*)MEM_MOTOR_CTRL_CUSTOM_RIGHT_BCK_KI;
	globalRightMotorParams.pid2.backward.Kd = *(__IO float*)MEM_MOTOR_CTRL_CUSTOM_RIGHT_BCK_KD;
#else
	globalMotorPidKp = *(__IO float*)MEM_MOTOR_CTRL_PID_KP;
	globalMotorPidKi = *(__IO float*)MEM_MOTOR_CTRL_PID_KI;
	globalMotorPidKd = *(__IO float*)MEM_MOTOR_CTRL_PID_KD;
#endif
}
