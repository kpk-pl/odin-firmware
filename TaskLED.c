#include "TaskLED.h"
#include "main.h"

void TaskLED(void * p) {
	IWDG_Enable();
	while(1) {
		GPIO_ToggleBits(LEDS_GPIO_26, LEDS_GPIO_3_PIN);
		/* Every 100 ms */
		for (uint8_t t = 0; t < 5; t++) {
			IWDG_ReloadCounter();
			vTaskDelay(100 / portTICK_RATE_MS);
		}
	}
}
