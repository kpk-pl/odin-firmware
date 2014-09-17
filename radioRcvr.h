#ifndef RADIO_RCVR_H
#define RADIO_RCVR_H

#include "hardware.h"

void radioSetup();

void radioDRDYInterruptFromISR();
void radioSPI_TXE_FromISR();
void radioSPI_RXDMA_TCIF_FromISR();

void radioDisable();
void radioEnable();
void radioTestCommand();
void radioResetCommand();

#endif /* RADIO_RCVR_H */
