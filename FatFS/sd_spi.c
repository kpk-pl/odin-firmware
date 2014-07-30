#include <stdbool.h>
#include "sd_spi.h"
#include "sd_hardware.h"
#include "priorities.h"

#include "FreeRTOS.h"
#include "semphr.h"

extern xSemaphoreHandle sdDMATCSemaphore; // defined and handled in main and task boot

static void stm32_dma_transfer(bool receive, uint8_t *buff, uint32_t length);

void SPI_init(uint32_t prescaler)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	// enable clocks
	SD_SPI_GPIO_CLOCK_FUN(SD_SPI_GPIO_CLOCK, ENABLE);
	SD_SPI_CLOCK_FUN(SD_SPI_CLOCK, ENABLE);

	// configure pins used by SPI
	GPIO_InitStruct.GPIO_Pin = SD_SPI_GPIO_PIN_SCK | SD_SPI_GPIO_PIN_MOSI | SD_SPI_GPIO_PIN_MISO;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(SD_SPI_GPIO, &GPIO_InitStruct);

	// connect SPI pins to SPI alternate function
	GPIO_PinAFConfig(SD_SPI_GPIO, SD_SPI_GPIO_PINSOURCE_SCK, SD_SPI_GPIO_AF);
	GPIO_PinAFConfig(SD_SPI_GPIO, SD_SPI_GPIO_PINSOURCE_MOSI, SD_SPI_GPIO_AF);
	GPIO_PinAFConfig(SD_SPI_GPIO, SD_SPI_GPIO_PINSOURCE_MISO, SD_SPI_GPIO_AF);

	/*
	 * configure SPI in Mode 0
	 * CPOL = 0 --> clock is low when idle
	 * CPHA = 0 --> data is sampled at the first edge
	 */
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // set to full duplex mode, seperate MOSI and MISO lines
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;     // transmit in master mode, NSS pin has to be always high
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; // one packet of data is 8 bits wide
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;        // clock is low when idle
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;      // data sampled at first edge
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft; // set the NSS management to internal and pull internal NSS high
	SPI_InitStruct.SPI_BaudRatePrescaler = prescaler; // SPI frequency is APB2 frequency / 4
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;// data is transmitted MSB first
	SPI_InitStruct.SPI_CRCPolynomial = 7;
	SPI_Init(SD_SPI, &SPI_InitStruct);

	/* Enable clock for DMA */
	RCC_AHB1PeriphClockCmd(SD_SPI_DMA_CLOCK, ENABLE);

	/* Set interrupt after finished transmission */
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PRIORITY_ISR_SDDMATCIF;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannel = SD_SPI_DMA_NVIC_IRQn_RX;
	NVIC_Init(&NVIC_InitStructure);

	/* Set common DMA settings - these will not change at all */
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(SD_SPI->DR));
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	/* Add some values to pass assert - these will change */
	DMA_InitStructure.DMA_Memory0BaseAddr = 0;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
	DMA_InitStructure.DMA_BufferSize = 1;
	/* Configure RX DMA stream */
	DMA_InitStructure.DMA_Channel = SD_SPI_DMA_CHANNEL_RX;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_Init(SD_SPI_DMA_STREAM_RX, &DMA_InitStructure);
	/* Configure TX DMA stream */
	DMA_InitStructure.DMA_Channel = SD_SPI_DMA_CHANNEL_TX;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_Init(SD_SPI_DMA_STREAM_TX, &DMA_InitStructure);

	/* Enable SPI */
	SPI_Cmd(SD_SPI, ENABLE);
}

uint8_t SPI_send_single(uint8_t data)
{
	SD_SPI->DR = data; // write data to be transmitted to the SPI data register
	while( !(SD_SPI->SR & SPI_I2S_FLAG_TXE) ); // wait until transmit complete
	while( !(SD_SPI->SR & SPI_I2S_FLAG_RXNE) ); // wait until receive complete
	while( SD_SPI->SR & SPI_I2S_FLAG_BSY ); // wait until SPI is not busy anymore
	return SD_SPI->DR; // return received data from SPI data register
}

uint8_t SPI_receive_single() {
	return SPI_send_single(0xFF);
}

void SPI_send(uint8_t* data, uint32_t length) {
	if (sdDMATCSemaphore) { // if semaphore was created, then use dma transfers
		stm32_dma_transfer(false, data, length);
	}
	else {
		while (length--) {
			SPI_send_single(*data);
			data++;
		}
	}
}

void SPI_receive(uint8_t* data, uint32_t length) {
	if (sdDMATCSemaphore) { // if semaphore was created, then use dma transfers
		stm32_dma_transfer(true, data, length);
	}
	else {
		while (length--) {
			*data = SPI_receive_single();
			data++;
		}
	}
}

void stm32_dma_transfer(bool receive, uint8_t *buff, uint32_t length) {
	uint32_t dummy = 0xFFFFFFFF;

	// size of payload
	SD_SPI_DMA_STREAM_RX->NDTR = length;
	SD_SPI_DMA_STREAM_TX->NDTR = length;

	if (receive) {
		// base 0 memory address - receiving real data, sending dummy payload
		SD_SPI_DMA_STREAM_RX->M0AR = (uint32_t)buff;
		SD_SPI_DMA_STREAM_TX->M0AR = (uint32_t)&dummy;

		// enable incrementing receive buffer, disable transmitting (one dummy byte)
		SD_SPI_DMA_STREAM_RX->CR |= DMA_MemoryInc_Enable;
		SD_SPI_DMA_STREAM_TX->CR &= ~(DMA_MemoryInc_Enable);
	}
	else {
		// base 0 memory address - receiving is irrelevant, transmitting real data
		SD_SPI_DMA_STREAM_RX->M0AR = (uint32_t)&dummy;
		SD_SPI_DMA_STREAM_TX->M0AR = (uint32_t)buff;

		// disable incrementing receive buffer, enable transmitting
		SD_SPI_DMA_STREAM_RX->CR &= ~(DMA_MemoryInc_Enable);
		SD_SPI_DMA_STREAM_TX->CR |= DMA_MemoryInc_Enable;
	}

	// enable interrupt after finished reception
	DMA_ITConfig(SD_SPI_DMA_STREAM_RX, DMA_IT_TC, ENABLE);

	// enable both streams
	DMA_Cmd(SD_SPI_DMA_STREAM_RX, ENABLE);
	DMA_Cmd(SD_SPI_DMA_STREAM_TX, ENABLE);
	while (DMA_GetCmdStatus(SD_SPI_DMA_STREAM_RX) != ENABLE);
	while (DMA_GetCmdStatus(SD_SPI_DMA_STREAM_TX) != ENABLE);

	// issue transmission start
	SPI_I2S_DMACmd(SD_SPI, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, ENABLE);

	// wait for transmission to end - non blocking
	xSemaphoreTake(sdDMATCSemaphore, portMAX_DELAY);

	// wait until both streams disable themselves
	while (DMA_GetCmdStatus(SD_SPI_DMA_STREAM_RX) != DISABLE);
	while (DMA_GetCmdStatus(SD_SPI_DMA_STREAM_TX) != DISABLE);

	// wait until SPI is not busy anymore
	while (SPI_I2S_GetFlagStatus(SD_SPI, SPI_I2S_FLAG_BSY) == SET);

	// disable transaction request
	SPI_I2S_DMACmd(SD_SPI, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, DISABLE);

	// clear flags for DMA allowing enabling streams in the future
	DMA_ClearFlag(SD_SPI_DMA_STREAM_RX, SD_SPI_DMA_FLAG_TCIF_RX);
	DMA_ClearFlag(SD_SPI_DMA_STREAM_TX, SD_SPI_DMA_FLAG_TCIF_TX);
}

/* ISR for SD DMA TCIF */
void SDTransferDoneHandler() {
	portBASE_TYPE contextSwitch = pdFALSE;
	if (DMA_GetITStatus(SD_SPI_DMA_STREAM_RX, SD_SPI_DMA_IT_TCIF_RX) == SET) {
		xSemaphoreGiveFromISR(sdDMATCSemaphore, &contextSwitch);
		DMA_ITConfig(SD_SPI_DMA_STREAM_RX, DMA_IT_TC, DISABLE);
	}
	portEND_SWITCHING_ISR(contextSwitch);
}
