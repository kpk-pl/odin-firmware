#include <stdbool.h>
#include "sd_spi.h"
#include "sd_hardware.h"

#include "FreeRTOS.h"
#include "semphr.h"

extern xSemaphoreHandle sdDMATCSemaphore;

void SPI_init(uint32_t prescaler)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;
	DMA_InitTypeDef DMA_InitStruture;
	NVIC_InitTypeDef NVIC_InitStructure;

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

//	/* Deinitialize DMA streams */
//	DMA_DeInit(SD_SPI_DMA_STREAM_RX);
//	DMA_DeInit(SD_SPI_DMA_STREAM_TX);
//
//	/* Configure DMA stream for transmission */
////DMA_InitStruture.DMA_FIFOMode = DMA_FIFOMode_Enable;
//	DMA_InitStruture.DMA_FIFOMode = DMA_FIFOMode_Disable;
//	DMA_InitStruture.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
//	DMA_InitStruture.DMA_MemoryBurst = DMA_MemoryBurst_INC16;
//	DMA_InitStruture.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//	DMA_InitStruture.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	DMA_InitStruture.DMA_Mode = DMA_Mode_Normal;
//	DMA_InitStruture.DMA_PeripheralBaseAddr = (uint32_t)(&(SD_SPI->DR));
//	DMA_InitStruture.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//	DMA_InitStruture.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//	DMA_InitStruture.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	DMA_InitStruture.DMA_Priority = DMA_Priority_High;
//	DMA_InitStruture.DMA_Channel = SD_SPI_DMA_CHANNEL_TX;
//	DMA_InitStruture.DMA_DIR = DMA_DIR_MemoryToPeripheral;
//	DMA_InitStruture.DMA_BufferSize = 1; // Number if bytes, temporary to pass assert, will be given later before wach transmission
//	DMA_Init(SD_SPI_DMA_STREAM_TX, &DMA_InitStruture);
//	/* Use the settings from above for reception */
//	DMA_InitStruture.DMA_Channel = SD_SPI_DMA_CHANNEL_RX;
//	DMA_InitStruture.DMA_DIR = DMA_DIR_PeripheralToMemory;
//	DMA_Init(SD_SPI_DMA_STREAM_RX, &DMA_InitStruture);
//	DMA_FlowControllerConfig(SD_SPI_DMA_STREAM_RX, DMA_FlowCtrl_Memory);
//
//	/* Disable double buffer mode */
//	DMA_DoubleBufferModeCmd(SD_SPI_DMA_STREAM_TX, DISABLE);
//	DMA_DoubleBufferModeCmd(SD_SPI_DMA_STREAM_RX, DISABLE);
//
	/* Set interrupt after finished transmissions */
//	NVIC_InitStructure.NVIC_IRQChannel = SD_SPI_DMA_NVIC_IRQn_TX;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 15; // TODO later
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 15;
//	NVIC_Init(&NVIC_InitStructure);
//	NVIC_InitStructure.NVIC_IRQChannel = SD_SPI_DMA_NVIC_IRQn_RX;
//	NVIC_Init(&NVIC_InitStructure);
//	DMA_ITConfig(SD_SPI_DMA_STREAM_TX, DMA_IT_TC, ENABLE);
//	DMA_ITConfig(SD_SPI_DMA_STREAM_RX, DMA_IT_TC, ENABLE);

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

void stm32_dma_transfer(bool receive, uint8_t *buff, uint32_t btr) {
	   DMA_InitTypeDef DMA_InitStructure;
	   uint8_t dummy = 0xFF;

	   /* shared DMA configuration values */
	   DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&(SD_SPI->DR));
	   DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	   DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	   DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	   DMA_InitStructure.DMA_BufferSize = btr;
	   DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	   DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	   DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	   DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	   DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	   DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;

	   if ( receive ) {
	      /* DMA1 channel4 configuration SPI2 RX ---------------------------------------------*/
	      DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)buff;
	      DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	      DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
		  DMA_InitStructure.DMA_Channel = SD_SPI_DMA_CHANNEL_RX;
	      DMA_Init(SD_SPI_DMA_STREAM_RX, &DMA_InitStructure);

	      /* DMA1 channel5 configuration SPI2 TX ---------------------------------------------*/
	      DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&dummy;
	      DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	      DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
		  DMA_InitStructure.DMA_Channel = SD_SPI_DMA_CHANNEL_TX;
	      DMA_Init(SD_SPI_DMA_STREAM_TX, &DMA_InitStructure);
	   }
	   else {
	      /* DMA1 channel2 configuration SPI2 RX ---------------------------------------------*/
	      DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&dummy;
	      DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	      DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;
	      DMA_InitStructure.DMA_Channel = SD_SPI_DMA_CHANNEL_RX;
	      DMA_Init(SD_SPI_DMA_STREAM_RX, &DMA_InitStructure);

	      /* DMA1 channel3 configuration SPI2 TX ---------------------------------------------*/
	      DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)buff;
	      DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	      DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	      DMA_InitStructure.DMA_Channel = SD_SPI_DMA_CHANNEL_TX;
	      DMA_Init(SD_SPI_DMA_STREAM_TX, &DMA_InitStructure);
	   }
	   //xSemaphoreTake(sdDMATCSemaphore, 0);
	   DMA_Cmd(SD_SPI_DMA_STREAM_RX, ENABLE);
	   DMA_Cmd(SD_SPI_DMA_STREAM_TX, ENABLE);

	   while (DMA_GetCmdStatus(SD_SPI_DMA_STREAM_RX) != ENABLE);
	   while (DMA_GetCmdStatus(SD_SPI_DMA_STREAM_TX) != ENABLE);

	   SPI_I2S_DMACmd(SD_SPI, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, ENABLE);

	   while (DMA_GetFlagStatus(SD_SPI_DMA_STREAM_RX, SD_SPI_DMA_FLAG_TCIF_RX) == RESET);
	   //xSemaphoreTake(sdDMATCSemaphore, portMAX_DELAY);

	   while (DMA_GetCmdStatus(SD_SPI_DMA_STREAM_RX) != DISABLE);
	   while (DMA_GetCmdStatus(SD_SPI_DMA_STREAM_TX) != DISABLE);

	   SPI_I2S_DMACmd(SD_SPI, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, DISABLE);

	   DMA_ClearFlag(SD_SPI_DMA_STREAM_RX, SD_SPI_DMA_FLAG_TCIF_RX);
	   DMA_ClearFlag(SD_SPI_DMA_STREAM_TX, SD_SPI_DMA_FLAG_TCIF_TX);
}

void SPI_send(uint8_t* data, uint32_t length) {
	if (sdDMATCSemaphore && length > 000) { // do not waste time sending rubbish
		stm32_dma_transfer(false, data, length);
//		SD_SPI_DMA_STREAM_TX->M0AR = (uint32_t)data; 	// base addr
//		SD_SPI_DMA_STREAM_TX->NDTR = length;   			// size in bytes
//		DMA_Cmd(SD_SPI_DMA_STREAM_TX, ENABLE);
//		while (DMA_GetCmdStatus(SD_SPI_DMA_STREAM_TX) != ENABLE);
//		SPI_I2S_DMACmd(SD_SPI, SPI_I2S_DMAReq_Tx, ENABLE);
//
//		// wait for finish
//		xSemaphoreTake(sdDMATCSemaphore, portMAX_DELAY);
//
//		//DMA_Cmd(SD_SPI_DMA_STREAM_TX, DISABLE);
//		while (DMA_GetCmdStatus(SD_SPI_DMA_STREAM_TX) != DISABLE);
	}
	else {
		while (length--) {
			SPI_send_single(*data);
			data++;
		}
	}
}

void SPI_receive(uint8_t* data, uint32_t length) {
	if (sdDMATCSemaphore && length > 00) { // TODO: why this does not work as should???
		stm32_dma_transfer(true, data, length);
//		SD_SPI_DMA_STREAM_RX->M0AR = (uint32_t)data; 	// base addr
//		SD_SPI_DMA_STREAM_RX->NDTR = length;   			// size in bytes
//		DMA_Cmd(SD_SPI_DMA_STREAM_RX, ENABLE);
//
//		// change DMA TX stream settings to send <length> random bytes
//		SD_SPI_DMA_STREAM_TX->CR &= ~(DMA_SxCR_MINC); // disable memory address increment
//		DMA_ITConfig(SD_SPI_DMA_STREAM_TX, DMA_IT_TC, DISABLE); // disable interrupt
//		uint8_t dummy = 0xFF;
//		SD_SPI_DMA_STREAM_TX->M0AR = (uint32_t)&dummy;
//		SD_SPI_DMA_STREAM_TX->NDTR = length;
//		DMA_Cmd(SD_SPI_DMA_STREAM_TX, ENABLE);
//
//		// wait for DMA to be effectively enabled
//		while (DMA_GetCmdStatus(SD_SPI_DMA_STREAM_RX) != ENABLE);
//		while (DMA_GetCmdStatus(SD_SPI_DMA_STREAM_TX) != ENABLE);
//
//		// start transmission
//		SPI_I2S_DMACmd(SD_SPI, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, ENABLE);
//
//		// wait for finish
//		//xSemaphoreTake(sdDMATCSemaphore, portMAX_DELAY);
//
//		//DMA_Cmd(SD_SPI_DMA_STREAM_RX, DISABLE);
//		//DMA_Cmd(SD_SPI_DMA_STREAM_TX, DISABLE);
//
//		// wait for full disable
//		while (DMA_GetCmdStatus(SD_SPI_DMA_STREAM_RX) != DISABLE);
//		while (DMA_GetCmdStatus(SD_SPI_DMA_STREAM_TX) != DISABLE);
//
//		// restore crucial DMA TX settings
//		SD_SPI_DMA_STREAM_TX->CR |= DMA_SxCR_MINC;
//		DMA_ClearITPendingBit(SD_SPI_DMA_STREAM_TX, DMA_IT_TCIF3);
//		DMA_ClearFlag(SD_SPI_DMA_STREAM_TX, SD_SPI_DMA_FLAG_TCIF_TX);
//		DMA_ITConfig(SD_SPI_DMA_STREAM_TX, DMA_IT_TC, ENABLE);
	}
	else {
		while (length--) {
			*data = SPI_receive_single();
			data++;
		}
	}
}
