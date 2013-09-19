#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f4xx.h"
#include "compilation.h"

/*
 * Global defines
 */

/*
 * Global types
 */

/*
 * Global variables
 */

/*
 * Interrupt routines
 */

/*
 * Global functions
 */

void COMAction();
void COMDMANotify();
#ifdef FOLLOW_TRAJECTORY
void RawStreamDMAIncoming();
#endif
void WIFIAction();
void WiFiDMANotify();
#ifdef USE_IMU_TELEMETRY
void IMUGyroReady();
void IMUAccReady();
void IMUMagReady();
void IMUI2CEVHandler();
#endif /* USE_IMU_TELEMETRY */
void BatteryTooLow();
void OSBusyTimerHandler();

void Switch1Changed();
void Switch2Changed();
void Switch3Changed();
void Switch4Changed();
void Switch5Changed();
void Switch6Changed();

#endif
