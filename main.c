#include <stdio.h>

#include "compilation.h"
#include "main.h"
#include "hwinterface.h"
#include "hwinit.h"
#include "memory.h"
#include "priorities.h"
#include "stackSpace.h"
#include "ff.h"

#include "TaskTelemetry.h"
#include "TaskRC5.h"
#include "TaskMotorCtrl.h"
#include "TaskPrintfConsumer.h"
#include "TaskCommandHandler.h"
#include "TaskCLI.h"
#include "TaskLED.h"
#include "TaskUSB2WiFiBridge.h"
#include "TaskInputMngr.h"
#include "TaskPenCtrl.h"
#include "TaskWiFiMngr.h"
#ifdef USE_IMU_TELEMETRY
#include "TaskIMU.h"
#include "TaskIMUMagScaling.h"
#endif
#ifdef FOLLOW_TRAJECTORY
#include "TaskTrajectory.h"
#endif
#ifdef DRIVE_COMMANDS
#include "TaskDrive.h"
#endif

volatile FunctionalState globalLogTelemetry = DISABLE;
volatile FunctionalState globalLogSpeed = DISABLE;
volatile FunctionalState globalLogEvents = ENABLE;
volatile FunctionalState globalLogIMU = DISABLE;
volatile float globalCPUUsage = 0.0f;
volatile bool globalUsingCLI = false;
volatile bool globalSDMounted = false;
xSemaphoreHandle sdDMATCSemaphore = NULL;
xTaskHandle taskBootIdleHandle;

#ifdef USE_SHORT_ASSERT
static volatile uint16_t globalAssertionFailed = 0;
#endif

static FATFS FatFS;
static void TaskBoot(void *p);
static void TaskBootIdle(void *p);

int main(void)
{
	/* Ensure all priority bits are assigned as preemption priority bits. */
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	SystemInit();
	SystemCoreClockUpdate();

	Initialize();

	if (globalLogEvents) {
		printf("Reset!\nCompilation settings:\n");
#ifndef USE_FULL_ASSERT
#ifndef USE_SHORT_ASSERT
		printf("WARNING! Assertion disabled\n");
#else
		printf("WARNING! Only short assertion enabled\n");
#endif
#endif
#ifdef USE_CUSTOM_MOTOR_CONTROLLER
		printf("\tCustom motor controller\n");
#else
		printf("\tPID motor controller\n");
#endif
#ifdef USE_IMU_TELEMETRY
#ifdef USE_GYRO_FOR_IMU
		printf("\tUsing IMU with Gyro\n");
#else
		printf("\tUsing IMU without Gyro\n");
#endif
#else
		printf("\tIMU not used\n");
#endif
#ifdef FOLLOW_TRAJECTORY
		printf("\tFollowing trajectory enabled\n");
#ifndef COMPILE_CIRCULAR_BUFFER
		printf("\tCircular buffer DISABLED\n");
#endif
#else
		printf("\tFollowing trajectory DISABLED\n");
#endif
#ifdef DRIVE_COMMANDS
		printf("\tDrive commands enabled\n");
#else
		printf("\tDrive commands DISABLED\n");
#endif
		if (globalUsingCLI)
			printf("\tUsing CLI\n");
		else
			printf("\tUsing command handler\n");
	}

	// start normal tasks - these should not really fire up because boot task is a DOMINATOR
	TaskInputMngrConstructor();
	if (globalUsingCLI)
		TaskCLIConstructor();
	else
		TaskCommandHandlerConstructor();
	TaskLEDConstructor();
	TaskMotorCtrlConstructor();
	TaskPrintfConsumerConstructor();
	TaskRC5Constructor();
	TaskTelemetryConstructor();
	TaskUSB2WiFiBridgeConstructor();
	TaskPenCtrlConstructor();
#ifdef FOLLOW_TRAJECTORY
	TaskTrajectoryConstructor();
#endif
#ifdef DRIVE_COMMANDS
	TaskDriveConstructor();
#endif
#ifdef USE_IMU_TELEMETRY
	TaskIMUConstructor();
	TaskIMUMagScalingConstructor();	// this should be called at last
#endif

	// boot task with DOMINATOR priority
	xTaskCreate(TaskBoot, NULL, TASKBOOT_STACKSPACE, NULL, PRIORITY_TASK_BOOT, NULL);
	// idle task with very high priority to disallow other tasks execution when boot task waits
	xTaskCreate(TaskBootIdle, NULL, configMINIMAL_STACK_SIZE, NULL, PRIORITY_TASK_BOOT, &taskBootIdleHandle);

	vTaskStartScheduler();
    while(1);
}

void TaskBootIdle(void *p) {
	while(1) {
		/*
		 * This will force context switch to TaskBoot if it is in Ready State,
		 * and will do nothing if it is in Blocked State. In the effect this task will
		 * try to change context switch as soon as it gets it, and the only one task
		 * the context will be switched to is TaskBoot. This will BLOCK all the other tasks
		 * allowing TaskBoot to be the only one in charge of the whole system.
		 */
		taskYIELD();
	}
}

void TaskBoot(void *p) {
	if (globalLogEvents)
		printf("Booting...\n");

	// create semaphore for SD SPI DMA transfer
	vSemaphoreCreateBinary(sdDMATCSemaphore);
	xSemaphoreTake(sdDMATCSemaphore, 0);

	// Filesystem mount
	printf("Mounting SD card...");
	FRESULT res = f_mount(&FatFS, "", 1);
	if (res == FR_OK) {
		printf(" done OK\n");
		globalSDMounted = true;
	}
	else {
		printf(" error: ");
		switch (res) {
		case FR_DISK_ERR:
			printf("DISK ERROR\n");
			break;
		case FR_NOT_READY:
			printf("NOT READY\n");
			break;
		case FR_NO_FILESYSTEM:
			printf("No FILESYSTEM\n");
			break;
		default:
			printf("ERRNO: %d\n", res);
			break;
		}
	}

	// read init files
	if (globalSDMounted) {
		bool allOK = true;
		printf("Reading init files...");
		if (!readInit(InitTarget_Telemetry)) {
			printf("\nErrors while reading %s", INIT_TELEMETRY_PATH);
			allOK = false;
		}
#ifdef USE_IMU_TELEMETRY
		if (!readInit(InitTarget_IMU)) {
			printf("\nErrors while reading %s", INIT_IMU_PATH);
			allOK = false;
		}
#endif
#ifdef FOLLOW_TRAJECTORY
		if (!readInit(InitTarget_Trajectory)) {
			printf("\nErrors while reading %s", INIT_TRAJECTORY_PATH);
			allOK = false;
		}
#endif
#ifdef USE_CUSTOM_MOTOR_CONTROLLER
		if (!readInit(InitTarget_Custom_Motor_Controler)) {
			printf("\nErrors while reading %s", INIT_MOTOR_CTRL_CUSTOM_PATH);
			allOK = false;
		}
#else
		if (!readInit(InitTarget_PID_Motor_Controler)) {
			printf("\nErrors while reading %s", INIT_MOTOR_CTRL_PID_PATH);
			allOK = false;
		}
#endif
		if (allOK) printf(" done OK\n");
		else printf("\nSD card: there were errors\n");
	}

#ifdef USE_SHORT_ASSERT
	if (globalAssertionFailed != 0)
		printf("Detected %d assertion errors\n", globalAssertionFailed);
#endif

	printf("Booting completed\n");

	/* Issue immediate setting WiFi speed to higher */
	//TaskWiFiMngrConstructor(WiFiMngr_Command_SetHighSpeed);

	// delete TaskBootIdle and itself
	vTaskDelete(taskBootIdleHandle);
	vTaskDelete(NULL);
}

void reportStackUsage() {
	safePrint(45, "High water mark of stack usage (free space)\n");
	safePrint(28, "printfConsumerTask: %d\n", uxTaskGetStackHighWaterMark(printfConsumerTask));
	if (globalUsingCLI)
		safePrint(28, "CLITask: %d\n", uxTaskGetStackHighWaterMark(CLITask));
	else
		safePrint(28, "commandHandlerTask: %d\n", uxTaskGetStackHighWaterMark(commandHandlerTask));
	safePrint(23, "motorCtrlTask: %d\n", uxTaskGetStackHighWaterMark(motorCtrlTask));
	safePrint(17, "RC5Task: %d\n", uxTaskGetStackHighWaterMark(RC5Task));
	safePrint(23, "telemetryTask: %d\n", uxTaskGetStackHighWaterMark(telemetryTask));
	safePrint(27, "USBWiFiBridgeTask: %d\n", uxTaskGetStackHighWaterMark(USBWiFiBridgeTask));
	safePrint(23, "InputMngrTask: %d\n", uxTaskGetStackHighWaterMark(commInputMngrTask));
	safePrint(21, "penCtrlTask: %d\n", uxTaskGetStackHighWaterMark(penCtrlTask));
#ifdef USE_IMU_TELEMETRY
	safePrint(17, "imuTask: %d\n", uxTaskGetStackHighWaterMark(imuTask));
#endif
#ifdef FOLLOW_TRAJECTORY
	safePrint(24, "trajectoryTask: %d\n", uxTaskGetStackHighWaterMark(trajectoryTask));
#endif
#ifdef DRIVE_COMMANDS
	safePrint(19, "driveTask: %d\n", uxTaskGetStackHighWaterMark(driveTask));
#endif
}

/* Called from ISR */
void Switch1Changed() {
	if (getSwitchStatus(1) == ON) {
		enableUSB(ENABLE);
	}
	else {
		enableUSB(DISABLE);
	}
}

/* Called from ISR */
void Switch2Changed() {
	if (getSwitchStatus(2) == ON) {
		enableWiFi(ENABLE);
	}
	else {
		enableWiFi(DISABLE);
	}
}

/* Called from ISR */
void Switch3Changed() {
	if (getSwitchStatus(3) == ON) {
		enableWiFi2USBBridge(ENABLE);
	}
	else {
		enableWiFi2USBBridge(DISABLE);
	}
}

/* Called from ISR */
void Switch4Changed() {
	safePrint(16, "Four changed\n");
}

/* Called from ISR */
void Switch5Changed() {
	safePrint(16, "Five changed\n");
}

/* Called from ISR */
void Switch6Changed() {
	if (getSwitchStatus(6) == ON) {
		enableLantern(ENABLE);
	}
	else {
		enableLantern(DISABLE);
	}
}

/* Analog watchdog interrupt handler */
void BatteryTooLow() {
	/* Stop all tasks */
	vTaskEndScheduler();
	/* Inform about low battery level */
	printf("Battery Low (%.2fV)!\nShutting Down\n", getBatteryVoltage());
	/* Turn off motors and lantern */
	enableMotors(DISABLE);
	enableLantern(DISABLE);
	/* Light 2 red LEDs */
	lightLED(6, ON);
	lightLED(1, ON);
	lightLED(2, OFF);
	lightLED(3, OFF);
	lightLED(4, OFF);
	lightLED(5, OFF);
	/* Infinite loop */
	while(1) {
		/* Reload watchdog to prevent from reset */
		IWDG_ReloadCounter();
	}
}

/* ISR for OS busy timer update */
void OSBusyTimerHandler() {
	uint16_t p = TIM_GetCounter(CPUUSAGE_CNT_TIM);
	TIM_SetCounter(CPUUSAGE_CNT_TIM, 0);
	globalCPUUsage = (float)p / (float)(CPUUSAGE_TIM_PERIOD + 1);
}

void vApplicationMallocFailedHook( void )
{
	printf("Malloc failed\n");
	while(1);
}

void vApplicationStackOverflowHook( void )
{
	printf("Stack overflow\n");
	while(1);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	vTaskEndScheduler();
	printf("Wrong parameters value: file %s on line %ld\r\n", file, line);
	while (1);
}

#else
#ifdef  USE_SHORT_ASSERT
/**
 * @brief  Indicates that some assertion failed. No additional information is retrieved. Increments failed assertion counter
 * @retval None
 */
void sh_assert_failed(void) {
	globalAssertionFailed++;
	printf("Assertion failed\n");
}
#endif
#endif
