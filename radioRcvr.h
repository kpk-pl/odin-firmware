#ifndef RADIO_RCVR_H
#define RADIO_RCVR_H

#include "hardware.h"

void radioSetup();
void RadioUSARTInterrupt();
void RadioDMATxCompleteInterrupt();

void radioDisable();
void radioEnable();
void radioTestCommand();
void radioResetCommand();
void radioResetIndicatorCommand();

#endif /* RADIO_RCVR_H */
