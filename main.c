#include <stdio.h>

#include "compilation.h"
#include "main.h"
#include "hwinterface.h"
#include "hwinit.h"

#include "TaskTelemetry.h"
#include "TaskRC5.h"
#include "TaskMotorCtrl.h"
#include "TaskPrintfConsumer.h"
#include "TaskCommandHandler.h"
#include "TaskLED.h"
#include "TaskUSB2WiFiBridge.h"
#include "TaskInputBuffer.h"
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

int main(void)
{
	/* Ensure all priority bits are assigned as preemption priority bits. */
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	SystemInit();
	SystemCoreClockUpdate();
	//RCC_HSEConfig(RCC_HSE_ON);

	Initialize();
	if (globalLogEvents) {
		printf("Reset!\nCompilation settings:\n");
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
		printf("Booting...\n"); // "Booting done is printed from TaskMotorCtrl as it always starts with high priority"
	}

	TaskCommandHandlerConstructor();
	TaskInputBufferConstructor();
	TaskLEDConstructor();
	TaskMotorCtrlConstructor();
	TaskPrintfConsumerConstructor();
	TaskRC5Constructor();
	TaskTelemetryConstructor();
	TaskUSB2WiFiBridgeConstructor();
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

	vTaskStartScheduler();
    while(1);
}

void reportStackUsage() {
	safePrint(45, "High water mark of stack usage (free space)\n");
	safePrint(28, "printfConsumerTask: %d\n", uxTaskGetStackHighWaterMark(printfConsumerTask));
	safePrint(28, "commandHandlerTask: %d\n", uxTaskGetStackHighWaterMark(commandHandlerTask));
	safePrint(23, "motorCtrlTask: %d\n", uxTaskGetStackHighWaterMark(motorCtrlTask));
	safePrint(17, "RC5Task: %d\n", uxTaskGetStackHighWaterMark(RC5Task));
	safePrint(23, "telemetryTask: %d\n", uxTaskGetStackHighWaterMark(telemetryTask));
	safePrint(27, "USBWiFiBridgeTask: %d\n", uxTaskGetStackHighWaterMark(USBWiFiBridgeTask));
	safePrint(29, "commInputBufferTask: %d\n", uxTaskGetStackHighWaterMark(commInputBufferTask));
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

#endif
