#include "TaskLED.h"
#include "main.h"
#include "priorities.h"
#include "stackSpace.h"

void TaskLED(void * p) {
	IWDG_Enable();
	while(1) {
		GPIO_ToggleBits(LEDS_GPIO_26, LEDS_GPIO_3_PIN);
		/* Every 100 ms */
		for (uint8_t t = 0; t < 50; t++) {
			IWDG_ReloadCounter();
			vTaskDelay(10 / portTICK_RATE_MS);
		}
	}
}

void TaskLEDConstructor() {
	xTaskCreate(TaskLED, NULL, TASKLED_STACKSPACE, NULL, PRIORITY_TASK_LED, NULL);
}
