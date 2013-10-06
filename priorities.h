#ifndef _PRIORITIES_H_
#define _PRIORITIES_H_

#define PRIORITY_TASK_HARDRT			4
#define PRIORITY_TASK_HIGH				3
#define PRIORITY_TASK_MEDIUM			2
#define PRIORITY_TASK_LOW				1
#define PRIORITY_TASK_IDLE				0

#define PRIOTITY_TASK_INPUTBUFFER		PRIORITY_TASK_HARDRT
#define PRIORITY_TASK_MOTORCTRL			PRIORITY_TASK_HARDRT
#define PRIORITY_TASK_TELEMETRY			PRIORITY_TASK_HIGH
#define PRIORITY_TASK_DRIVE				PRIORITY_TASK_HIGH
#define PRIORITY_TASK_TRAJECTORY		PRIORITY_TASK_HIGH
#define PRIOTITY_TASK_COMMANDHANDLER 	PRIORITY_TASK_MEDIUM
#define PRIORITY_TASK_IMU				PRIORITY_TASK_HIGH
#define PRIORITY_TASK_RC5				PRIORITY_TASK_MEDIUM
#define PRIORITY_TASK_PRINTFCONSUMER	PRIORITY_TASK_LOW
#define PRIOTITY_TASK_BRIDGE			PRIORITY_TASK_MEDIUM
#define PRIORITY_TASK_LED				PRIORITY_TASK_HARDRT
#define PRIORITY_TASK_IMUMAGSCALING		PRIORITY_TASK_MEDIUM
#define PRIORITY_TASK_PENCTRL			PRIORITY_TASK_HIGH

/* PRIORITIES LOWER THAN configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY THAT DO NOT USE API CALLS */
#define PRIORITY_ISR_LANTERN			8

/* PRIORITIES HIGHER THAN 'configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY' (5) CAN USE CONTEXT SAFE FREERTOS API CALLS */
#define PRIORITY_ISR_IMUREADY			5
#define PRIORITY_ISR_IMUI2CEVENT		5
#define PRIORITY_ISR_COMDMARX			6
#define PRIORITY_ISR_WIFIDMARX			6
#define PRIORITY_ISR_RC5_TIM			7
#define PRIORITY_ISR_SWITCHES_TIM		7
#define PRIORITY_ISR_SWITCHES			8
#define PRIORITY_ISR_COMUSART			9
#define PRIORITY_ISR_WIFIUSART			9
#define PRIORITY_ISR_ADC				11
#define PRIORITY_ISR_COMDMATX			15
#define PRIORITY_ISR_WIFIDMATX			15
#define PRIORITY_ISR_OSBUSY				15


#endif
