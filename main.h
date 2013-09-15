#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f4xx.h"
#include "compilation.h"

void COMAction();
void COMDMANotify();
#ifdef FOLLOW_TRAJECTORY
void RawStreamDMAIncoming();
#endif
void WIFIAction();
void WiFiDMANotify();
void IMUGyroReady();
void IMUAccReady();
void IMUMagReady();
void BatteryTooLow();
void OSBusyTimerHandler();
void IMUI2CEVHandler();

void Switch1Changed();
void Switch2Changed();
void Switch3Changed();
void Switch4Changed();
void Switch5Changed();
void Switch6Changed();

#endif
