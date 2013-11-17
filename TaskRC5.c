#include <stm32f4xx.h>

#include "rc5_tim_exti.h"

#include "TaskRC5.h"
#include "main.h"
#include "priorities.h"
#include "stackSpace.h"
#include "compilation.h"
#include "hwinterface.h"

#include "TaskIMUMagScaling.h"
#include "TaskMotorCtrl.h"
#include "TaskPrintfConsumer.h"
#include "TaskPenCtrl.h"

/*
 *              (2)
 *           (1)   (3)
 *        (4)   (5)   (6)
 *           (7)   (9)
 *              (8)
 *   0 - last command
 */
static void issueDrive(uint8_t direction, float speed);

xTaskHandle RC5Task;						/*!< This task handle */
xSemaphoreHandle rc5CommandReadySemaphore;	/*!< Semaphore used by rc5_tim_exti.h library to incicate incomming command */

void TaskRC5(void * p) {
	/* Timer callback function to switch LED off */
	void rc5LEDOff(xTimerHandle xTimer) {
		lightLED(2, OFF);
	}
	/* Timer for switching led off */
	xTimerHandle rc5LEDTimer = xTimerCreate(NULL, 200/portTICK_RATE_MS, pdFALSE, NULL, rc5LEDOff);

	/* Initialize RC5 hardware */
	RC5_Receiver_Init();
	/* Initial semaphore take so it can be given and waited for */
	xSemaphoreTake(rc5CommandReadySemaphore, 0);

	RC5Frame_TypeDef frame;
	uint8_t toggle = 2;
	float maxSpeed = 3.0f;
	bool tempboolean;

	while(1) {
		/* Blocking wait - suspend task till semaphore is set */
		xSemaphoreTake(rc5CommandReadySemaphore, portMAX_DELAY);
		/* Decode command from external API */
		RC5_Decode(&frame);
		/* If this is a unique press (ignore continuous pressing) */
		if (toggle != frame.ToggleBit) {
			if (frame.Command >= 1 && frame.Command <= 9) { /* buttons 1-9 */
				issueDrive(frame.Command, maxSpeed);
			}
			else {
				switch(frame.Command) {
				case 12: /*<< OFF red button */
					/* Set too low preload value causing reset to occur */
					IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
					IWDG_SetReload(1);
					break;
				case 16: /*<< VOL up button */
					maxSpeed *= 1.2f;
					issueDrive(0, maxSpeed);
					break;
				case 17: /*<< VOL down button */
					maxSpeed /= 1.2f;
					issueDrive(0, maxSpeed);
					break;
				case 32: /*<< CH up button */
					tempboolean = false;
					xQueueSendToBack(penCommandQueue, &tempboolean, portMAX_DELAY);
					break;
				case 33: /*<< CH down button */
					tempboolean = true;
					xQueueSendToBack(penCommandQueue, &tempboolean, portMAX_DELAY);
					break;
#ifdef USE_IMU_TELEMETRY
				case 42: /*<< clock button */
					xSemaphoreGive(imuMagScalingReq);
					break;
#endif
				case 43: /*<< screen button above purple button */
					setWiFiMode(getWiFiMode() == WiFiMode_Data ? WiFiMode_Command : WiFiMode_Data);
					break;
				case 52: /*<< purple button */
					enableLantern(!getLanternState());
					break;
				default:
					break;
				}
			}

			if (globalLogEvents) safePrint(10, "RC5: %d\n", frame.Command);
			/* Light LED and reset software timer; when it expires the LED will be turned off */
			lightLED(2, ON);
			xTimerReset(rc5LEDTimer, 0);
		}
		toggle = frame.ToggleBit;
	}
}

void issueDrive(uint8_t direction, float speed) {
	static uint8_t lastCommand = 5; // means stop

	if (direction == 0)
		direction = lastCommand;

	switch(direction) {
	case 1:
		sendSpeeds(speed * 0.75f, speed, 0);
		break;
	case 2:
		sendSpeeds(speed, speed, 0);
		break;
	case 3:
		sendSpeeds(speed, speed * 0.75f, 0);
		break;
	case 4:
		sendSpeeds(-speed * 0.75f, speed * 0.75f, 0);
		break;
	case 5:
		sendSpeeds(.0f, .0f, 0);
		break;
	case 6:
		sendSpeeds(speed * 0.75f, -speed * 0.75f, 0);
		break;
	case 7:
		sendSpeeds(-speed * 0.75f, -speed, 0);
		break;
	case 8:
		sendSpeeds(-speed, -speed, 0);
		break;
	case 9:
		sendSpeeds(-speed, -speed * 0.75f, 0);
		break;
	default:
		return;
	}

	lastCommand = direction;
}

void TaskRC5Constructor() {
	xTaskCreate(TaskRC5, NULL, TASKRC5_STACKSPACE, NULL, PRIORITY_TASK_RC5, &RC5Task);
	vSemaphoreCreateBinary(rc5CommandReadySemaphore);
}
