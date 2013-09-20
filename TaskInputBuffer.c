#include "TaskInputBuffer.h"
#include "main.h"

#define BUF_RX_LEN 20		/*<< Maximum length of UART command */

void TaskInputBuffer(void * p) {
	PrintInput_Struct newInput;

	bool incoming[2] = {false, false};
	char RXBUF[2][BUF_RX_LEN+1];
	uint8_t RXBUFPOS[2] = {0, 0};
	uint8_t i;

	while(1) {
		/* Block until new command is available */
		xQueueReceive(commInputBufferQueue, &newInput, portMAX_DELAY);

		/* Check source */
		switch (newInput.Source) {
		case PrintSource_Type_USB:
			i = 0;
			break;
		case PrintSource_Type_WiFi:
			i = 1;
			break;
		default: break;
		}

		switch (newInput.Input) {
		case '<':	// msg begin character
			RXBUFPOS[i] = 0;
			incoming[i] = true;
			break;
		case '>':	// msg end character
			if (incoming[i]) {
				RXBUF[i][RXBUFPOS[i]++] = '\0';
				incoming[i] = false;

				char * ptr = (char *)pvPortMalloc(RXBUFPOS[i]*sizeof(char));
				strcpy(ptr, (const char*)RXBUF[i]);

				RXBUFPOS[i] = 0;

				portBASE_TYPE contextSwitch = pdFALSE;
				if (xQueueSendToBackFromISR(commandQueue, &ptr, &contextSwitch) == errQUEUE_FULL) {
					vPortFree(ptr);
				}
				portEND_SWITCHING_ISR(contextSwitch);
			}
			break;
		default:
			if (incoming[i]) {
				if (RXBUFPOS[i] < BUF_RX_LEN)
					RXBUF[i][RXBUFPOS[i]++] = newInput.Input;
			}
			break;
		}
	}
}
