/**************************************************************************//*****
 * @file     stdio.c
 * @brief    Implementation of newlib syscall
 ********************************************************************************/

#include <stdio.h>
#include <stdarg.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stm32f4xx.h>
#include "hardware.h"
#include "hwinterface.h"

#undef errno
extern int errno;
extern int  _end;

caddr_t _sbrk ( int incr )
{
  static unsigned char *heap = NULL;
  unsigned char *prev_heap;

  if (heap == NULL) {
    heap = (unsigned char *)&_end;
  }
  prev_heap = heap;

  heap += incr;

  return (caddr_t) prev_heap;
}

int link(char *old, char *new) {
return -1;
}

int _close(int file)
{
  return -1;
}

int _fstat(int file, struct stat *st)
{
  st->st_mode = S_IFCHR;
  return 0;
}

int _isatty(int file)
{
  return 1;
}

int _lseek(int file, int ptr, int dir)
{
  return 0;
}

int _read(int file, char *ptr, int len)
{
  return 0;
}

int _write(int file, char *ptr, int len)
{
	if (getWiFi2USBBridgeStatus() != ON) {
		int counter = len;
		for (; counter > 0; counter--) {
			if (*ptr == 0) break;
			if (getUSBStatus() == ON) {
				USART_SendData(COM_USART, *ptr);
				while (USART_GetFlagStatus(COM_USART, USART_FLAG_TXE) == RESET);
			}
			if (getWiFiStatus() == ON) {
				USART_SendData(WIFI_USART, *ptr);
				while (USART_GetFlagStatus(WIFI_USART, USART_FLAG_TXE) == RESET);
			}
			ptr++;
		}
		return len;
	}
	else
		return 0;
}

void abort(void)
{
  /* Abort called */
  while(1);
}
          
/* --------------------------------- End Of File ------------------------------ */
