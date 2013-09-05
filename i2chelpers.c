#include <stm32f4xx.h>
#include "i2chelpers.h"
#include "FreeRTOS.h"
#include "semphr.h"

extern xSemaphoreHandle imuI2CEV;
extern xQueueHandle I2CEVFlagQueue;

static void waitForI2CEventStart(I2C_TypeDef* I2Cx, uint32_t event) {
	if (xQueueSendToBack(I2CEVFlagQueue, &event, 0) == errQUEUE_FULL) {
		I2C_ITConfig(I2Cx, I2C_IT_EVT, DISABLE);
		xQueueReset(I2CEVFlagQueue);
		xQueueSendToBack(I2CEVFlagQueue, &event, portMAX_DELAY);
	}
}

static void waitForI2CEventEnd(void) {
	I2C_ITConfig(I2C1, I2C_IT_EVT, ENABLE);
	xSemaphoreTake(imuI2CEV, portMAX_DELAY);
}

void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction){
	// wait until I2C1 is not busy anymore
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));

	// Send I2C1 START condition
	I2C_GenerateSTART(I2Cx, ENABLE);

	// wait for I2C1 EV5 --> Slave has acknowledged start condition
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

	// Send slave Address for write
	I2C_Send7bitAddress(I2Cx, address, direction);

	/* wait for I2C1 EV6, check if
	 * either Slave has acknowledged Master transmitter or
	 * Master receiver mode, depending on the transmission
	 * direction
	 */
	if(direction == I2C_Direction_Transmitter){
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	}
	else if(direction == I2C_Direction_Receiver){
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	}
}

void I2C_write(I2C_TypeDef* I2Cx, uint8_t data){
	I2C_SendData(I2Cx, data);
	// wait for I2C1 EV8_2 --> byte has been transmitted
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

uint8_t I2C_read_ack(I2C_TypeDef* I2Cx){
	// enable acknowledge of recieved data
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

uint8_t I2C_read_nack(I2C_TypeDef* I2Cx){
	// disable acknowledge of received data
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

void I2C_stop(I2C_TypeDef* I2Cx){
	// Send I2C1 STOP Condition
	I2C_GenerateSTOP(I2Cx, ENABLE);
}

uint8_t ReadI2CRegister(I2C_TypeDef * I2Cx, uint8_t address, uint8_t reg) {
	// wait until I2C1 is not busy anymore
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));

	// enable ACK
	I2C_AcknowledgeConfig(I2Cx, ENABLE);

	// start transaction
	waitForI2CEventStart(I2Cx, I2C_EVENT_MASTER_MODE_SELECT);
	I2C_GenerateSTART(I2Cx, ENABLE);
	waitForI2CEventEnd();
//	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

	// send slave address and initiate writing
	waitForI2CEventStart(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	I2C_Send7bitAddress(I2Cx, address, I2C_Direction_Transmitter);
	waitForI2CEventEnd();
//	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	// send register address
	waitForI2CEventStart(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	I2C_SendData(I2Cx, reg);
	waitForI2CEventEnd();
//	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	// repeated start
	waitForI2CEventStart(I2Cx, I2C_EVENT_MASTER_MODE_SELECT);
	I2C_GenerateSTART(I2Cx, ENABLE);
	waitForI2CEventEnd();
//	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

	// send slave address and initiate reading
	waitForI2CEventStart(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
	I2C_Send7bitAddress(I2Cx, address, I2C_Direction_Receiver);
	// disable ACK right away, as described in Application Note
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	waitForI2CEventEnd();
//	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

	// wait for data
//	waitForI2CEventStart(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED);
//	waitForI2CEventEnd();
	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));
	uint8_t read = I2C_ReceiveData(I2Cx);

	// finish up
	I2C_GenerateSTOP(I2Cx, ENABLE);
	return read;
}

void ReadI2CMemory(I2C_TypeDef * I2Cx, uint8_t address, uint8_t baseReg, uint8_t *memory, uint8_t num) {
	// wait until I2C1 is not busy anymore
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));

	// enable ACK
	I2C_AcknowledgeConfig(I2Cx, ENABLE);

	// start transaction
	waitForI2CEventStart(I2Cx, I2C_EVENT_MASTER_MODE_SELECT);
	I2C_GenerateSTART(I2Cx, ENABLE);
	waitForI2CEventEnd();
//	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

	// send slave address and initiate writing
	waitForI2CEventStart(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	I2C_Send7bitAddress(I2Cx, address, I2C_Direction_Transmitter);
	waitForI2CEventEnd();
//	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	// send register address
	waitForI2CEventStart(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	I2C_SendData(I2Cx, baseReg);
	waitForI2CEventEnd();
//	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	// repeated start
	waitForI2CEventStart(I2Cx, I2C_EVENT_MASTER_MODE_SELECT);
	I2C_GenerateSTART(I2Cx, ENABLE);
	waitForI2CEventEnd();
//	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));

	// send slave address and initiate reading
	waitForI2CEventStart(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED);
	I2C_Send7bitAddress(I2Cx, address, I2C_Direction_Receiver);
	waitForI2CEventEnd();
//	while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

	for (uint8_t i = 0; i<num; i++) {
		// last read without ACK
		if (i == num-1) I2C_AcknowledgeConfig(I2Cx, DISABLE);

		// read next byte
		waitForI2CEventStart(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED);
		waitForI2CEventEnd();
//		while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED));
		memory[i] = I2C_ReceiveData(I2Cx);
	}

	// finish up
	I2C_GenerateSTOP(I2Cx, ENABLE);
}

void WriteI2CRegister(I2C_TypeDef * I2Cx, uint8_t address, uint8_t reg, uint8_t data) {
	// wait until I2C1 is not busy anymore
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));

	// enable ACK
	I2C_AcknowledgeConfig(I2Cx, ENABLE);

	// start transaction
	waitForI2CEventStart(I2Cx, I2C_EVENT_MASTER_MODE_SELECT);
	I2C_GenerateSTART(I2Cx, ENABLE);
	waitForI2CEventEnd();
//	while (I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT) == ERROR);

	// send slave address and initiate writing
	waitForI2CEventStart(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED);
	I2C_Send7bitAddress(I2Cx, address, I2C_Direction_Transmitter);
	waitForI2CEventEnd();
//	while (I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) == ERROR);

	// write register address
	waitForI2CEventStart(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	I2C_SendData(I2Cx, reg);
	waitForI2CEventEnd();
//	while (I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == ERROR);

	// write data
	waitForI2CEventStart(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED);
	I2C_SendData(I2Cx, data);
	waitForI2CEventEnd();
//	while (I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED) == ERROR);

	// finish up
	I2C_GenerateSTOP(I2Cx, ENABLE);
}
