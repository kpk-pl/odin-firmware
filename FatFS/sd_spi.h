#ifndef _SPI_H_
#define _SPI_H_

#include <stm32f4xx.h>

void SPI_init(uint32_t prescaler);
uint8_t SPI_send_single(uint8_t data);
uint8_t SPI_receive_single();
void SPI_send(uint8_t* data, uint32_t length);
void SPI_receive(uint8_t* data, uint32_t length);

#endif // _SPI_H_
