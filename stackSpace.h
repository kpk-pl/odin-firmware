#ifndef _STACKSPACE_H_
#define _STACKSPACE_H_

#include "portmacro.h"

#define SAFESTACK_STACKSPACE(x) (((x) > (configMINIMAL_STACK_SIZE)) ? (x) : (configMINIMAL_STACK_SIZE))

#define TASKIMU_STACKSPACE 				SAFESTACK_STACKSPACE(300)
#define TASKCOMMANDHANDLER_STACKSPACE 	SAFESTACK_STACKSPACE(300)
#define TASKDRIVE_STACKSPACE 			SAFESTACK_STACKSPACE(1000)
#define TASKIMUSCALING_STACKSPACE 		SAFESTACK_STACKSPACE(1100)
#define TASKINPUTBUFFER_STACKSPACE 		SAFESTACK_STACKSPACE(500)
#define TASKLED_STACKSPACE 				SAFESTACK_STACKSPACE(50)
#define TASKPRINTFCONSUMER_STACKSPACE 	SAFESTACK_STACKSPACE(200)
#define TASKRC5_STACKSPACE 				SAFESTACK_STACKSPACE(250)
#define TASKTELEMETRY_STACKSPACE 		SAFESTACK_STACKSPACE(300)
#define TASKTRAJECTORY_STACKSPACE 		SAFESTACK_STACKSPACE(1000)
#define TASKBRIDGE_STACKSPACE 			SAFESTACK_STACKSPACE(150)
#define TASKPENCTRL_STACKSPACE			SAFESTACK_STACKSPACE(300)
#define TASKCLI_STACKSPACE				SAFESTACK_STACKSPACE(800)
#define TASKBOOT_STACKSPACE				SAFESTACK_STACKSPACE(2000)
#define TASKWIFIMNGR_STACKSPACE			SAFESTACK_STACKSPACE(800)
#define TASKASYNCCALLHANDLER_STACKSPACE SAFESTACK_STACKSPACE(400)

#endif /* _STACKSPACE_H_ */
