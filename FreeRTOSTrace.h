#ifndef _FREERTOS_TRACE_
#define _FREERTOS_TRACE_

#include "hardware.h"

#define traceTASK_SWITCHED_IN() \
	do { \
		if ((xTaskHandle)pxCurrentTCB == xIdleTaskHandle) \
			TIM_Cmd(CPUUSAGE_CNT_TIM, DISABLE); \
		else \
			TIM_Cmd(CPUUSAGE_CNT_TIM, ENABLE); \
	} while(0)

#endif /* FREERTOS_TRACE */
