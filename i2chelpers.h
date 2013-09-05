#ifndef __I2CHELPERS_H__
#define __I2CHELPERS_H__

#include <stm32f4xx.h>

/*
 * Code based on:
 * http://eliaselectronics.com/stm32f4-tutorials/stm32f4-i2c-master-tutorial/
 */

/* This function issues a start condition and
 * transmits the slave address + R/W bit
 *
 * Parameters:
 * 		I2Cx --> the I2C peripheral e.g. I2C1
 * 		address --> the 7 bit slave address
 * 		direction --> the tranmission direction can be:
 * 						I2C_Direction_Tranmitter for Master transmitter mode
 * 						I2C_Direction_Receiver for Master receiver
 */
void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction);

/* This function transmits one byte to the slave device
 * Parameters:
 *		I2Cx --> the I2C peripheral e.g. I2C1
 *		data --> the data byte to be transmitted
 */
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data);

/* This function reads one byte from the slave device
 * and acknowledges the byte (requests another byte)
 */
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx);

/* This function reads one byte from the slave device
 * and doesn't acknowledge the recieved data
 */
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx);

/* This funtion issues a stop condition and therefore
 * releases the bus
 */
void I2C_stop(I2C_TypeDef* I2Cx);

/*
 * Read one byte of data from register
 */
uint8_t ReadI2CRegister(I2C_TypeDef * I2Cx, uint8_t address, uint8_t reg);

/*
 * Read multiple bytes of data from remore I2C slave device
 * Parameters:
 * 		I2Cx --> the I2C peripheral e.g. I2C1
 * 		address --> slave device address
 * 		baseReg --> first register to read from
 * 		memory --> pointer to an array into which data will be read
 * 		num --> number of bytes to be read and written to memory array
 */
void ReadI2CMemory(I2C_TypeDef * I2Cx, uint8_t address, uint8_t baseReg, uint8_t *memory, uint8_t num);

/* Write one byte of data to register */
void WriteI2CRegister(I2C_TypeDef * I2Cx, uint8_t address, uint8_t reg, uint8_t data);

#endif /* I2CHELPERS_H */
