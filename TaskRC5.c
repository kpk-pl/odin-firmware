#include <stm32f4xx.h>

#include "FreeRTOS.h"
#include "timers.h"

#include "rc5_tim_exti.h"

#include "TaskRC5.h"
#include "main.h"
#include "compilation.h"
#include "hwinterface.h"

#include "TaskIMUMagScaling.h"

extern xQueueHandle motorCtrlQueue;		/*!< Queue with speeds for motor regulator. It should contain type (MotorSpeed_Struct) */

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

	MotorSpeed_Struct motorSpeed;
	RC5Frame_TypeDef frame;
	uint8_t toggle = 2;
	float maxSpeed = 3.0f;

	while(1) {
		/* Blocking wait - suspend task till semaphore is set */
		xSemaphoreTake(rc5CommandReadySemaphore, portMAX_DELAY);
		/* Decode command from external API */
		RC5_Decode(&frame);
		/* If this is a unique press (ignore continuous pressing) */
		if (toggle != frame.ToggleBit) {
			switch(frame.Command) {
			case 1: /*<< 1 button */
				motorSpeed.LeftSpeed = maxSpeed * 0.5f;
				motorSpeed.RightSpeed = maxSpeed;
				xQueueSendToBack(motorCtrlQueue, &motorSpeed, portMAX_DELAY);
				break;
			case 2: /*<< 2 button */
				motorSpeed.LeftSpeed = motorSpeed.RightSpeed = maxSpeed;
				xQueueSendToBack(motorCtrlQueue, &motorSpeed, portMAX_DELAY);
				break;
			case 3: /*<< 3 button */
				motorSpeed.LeftSpeed = maxSpeed;
				motorSpeed.RightSpeed = maxSpeed * 0.5f;
				xQueueSendToBack(motorCtrlQueue, &motorSpeed, portMAX_DELAY);
				break;
			case 4: /*<< 4 button */
				motorSpeed.LeftSpeed = -maxSpeed * 0.5f;
				motorSpeed.RightSpeed = maxSpeed * 0.5f;
				xQueueSendToBack(motorCtrlQueue, &motorSpeed, portMAX_DELAY);
				break;
			case 5: /*<< 5 button */
				motorSpeed.LeftSpeed = .0f;
				motorSpeed.RightSpeed = .0f;
				xQueueSendToBack(motorCtrlQueue, &motorSpeed, portMAX_DELAY);
				break;
			case 6: /*<< 6 button */
				motorSpeed.LeftSpeed = maxSpeed * 0.5f;
				motorSpeed.RightSpeed = -maxSpeed * 0.5f;
				xQueueSendToBack(motorCtrlQueue, &motorSpeed, portMAX_DELAY);
				break;
			case 7: /*<< 7 button */
				motorSpeed.LeftSpeed = -maxSpeed * 0.5f;
				motorSpeed.RightSpeed = -maxSpeed;
				xQueueSendToBack(motorCtrlQueue, &motorSpeed, portMAX_DELAY);
				break;
			case 8: /*<< 8 button */
				motorSpeed.LeftSpeed = motorSpeed.RightSpeed = -maxSpeed;
				xQueueSendToBack(motorCtrlQueue, &motorSpeed, portMAX_DELAY);
				break;
			case 9: /*<< 9 button */
				motorSpeed.LeftSpeed = -maxSpeed;
				motorSpeed.RightSpeed = -maxSpeed * 0.5f;
				xQueueSendToBack(motorCtrlQueue, &motorSpeed, portMAX_DELAY);
				break;
			case 12: /*<< OFF red button */
				/* Set too low preload value causing reset to occur */
				IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
				IWDG_SetReload(1);
				break;
			case 16: /*<< VOL up button */
				maxSpeed *= 1.2f;
				break;
			case 17: /*<< VOL down button */
				maxSpeed /= 1.2f;
				break;
			case 32: /*<< CH up button */
				setPenUp();
				break;
			case 33: /*<< CH down button */
				setPenDown();
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
			}

			if (globalLogEvents) safePrint(10, "RC5: %d\n", frame.Command);
			/* Light LED and reset software timer; when it expires the LED will be turned off */
			lightLED(2, ON);
			xTimerReset(rc5LEDTimer, 0);
		}
		toggle = frame.ToggleBit;
	}
}
