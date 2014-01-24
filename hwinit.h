#ifndef _HWINIT_H_
#define _HWINIT_H_

#include <stm32f4xx.h>

/**
 * \brief Initialize all hardware.
 *
 * THIS FUNCTION DISABLES INTERRUPTS AND DO NOT ENABLES THEM AGAIN
 */
void Initialize();

/**
 *  Sets up WiFi UART to mode 8N1 with specified speed
 */
void InitializeWiFiUART(uint32_t speed);

#endif
