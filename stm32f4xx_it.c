/**
*****************************************************************************
**
**  File        : stm32f4xx_it.c
**
**  Abstract    : Main Interrupt Service Routines.
**                This file provides template for all exceptions handler and
**                peripherals interrupt service routine.
**
**  Environment : Atollic TrueSTUDIO(R)
**                STMicroelectronics STM32F4xx Standard Peripherals Library
**
**  Distribution: The file is distributed “as is,” without any warranty
**                of any kind.
**
**  (c)Copyright Atollic AB.
**  You may use this file as-is or modify it according to the needs of your
**  project. Distribution of this file (unmodified or modified) is not
**  permitted. Atollic AB permit registered Atollic TrueSTUDIO(R) users the
**  rights to distribute the assembled, compiled & linked contents of this
**  file as part of an application binary file, provided that it is built
**  using the Atollic TrueSTUDIO(R) toolchain.
**
**
*****************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "main.h"
#include "hardware.h"
#include "sd_hardware.h"
#include "hwinterface.h"
#include "rc5_tim_exti.h"
#include "compilation.h"
#include "radioRcvr.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t SWITCH_LAST_CHANGE = 0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
static void SWITCH_TIM_RESTART(void) {
	TIM_SetCounter(SWITCHES_TIM, 0);
	TIM_Cmd(SWITCHES_TIM, ENABLE);
}

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

void COM_USART_IRQHANDLER(void) {
	COMAction();
}

void WIFI_USART_IRQHANDLER(void) {
	WIFIAction();
}

void COM_TX_DMA_IRQHANDLER(void) {
	COMDMANotify();
}

void WIFI_TX_DMA_IRQHANDLER(void) {
	WiFiDMANotify();
}

void WIFI_RX_DMA_IRQHANDLER(void) {
	extern void StreamDMAWifiFinish(void);
	if (DMA_GetFlagStatus(WIFI_RX_DMA_STREAM, WIFI_RX_DMA_FLAG_TCIF) == SET)
		StreamDMAWifiFinish();
}

#ifdef USE_IMU_TELEMETRY
void IMU_GINT_IRQHANDLER(void) {
	IMUGyroReady();
	EXTI_ClearFlag(IMU_EXTI_GINT_LINE);
}

void IMU_AINT_IRQHANDLER(void) {
	IMUAccReady();
	EXTI_ClearFlag(IMU_EXTI_AINT_LINE);
}

void IMU_MINT_IRQHANDLER(void) {
	IMUMagReady();
	EXTI_ClearFlag(IMU_EXTI_MINT_LINE);
}
#endif /* USE_IMU_TELEMETRY */

void EXTI9_5_IRQHandler(void) {
	if (EXTI_GetFlagStatus(SWITCHES_EXTI_2_LINE) == SET) {
		SWITCH_LAST_CHANGE = 2;
		SWITCH_TIM_RESTART();
		EXTI_ClearITPendingBit(SWITCHES_EXTI_2_LINE);
	}
	else if (EXTI_GetFlagStatus(SWITCHES_EXTI_1_LINE) == SET) {
		SWITCH_LAST_CHANGE = 1;
		SWITCH_TIM_RESTART();
		EXTI_ClearITPendingBit(SWITCHES_EXTI_1_LINE);
	}
}

void EXTI15_10_IRQHandler(void) {
	/* Mesure first duration to validate the RC5 frame */
	if (EXTI_GetFlagStatus(RC5_EXTI_LINE) == SET) {
		RC5_MeasureFirstLowDuration();
	}
	/*else if (EXTI_GetFlagStatus(SWITCHES_EXTI_6_LINE) == SET) {
		SWITCH_LAST_CHANGE = 6;
		SWITCH_TIM_RESTART();
		EXTI_ClearITPendingBit(SWITCHES_EXTI_6_LINE);
	}
	else if (EXTI_GetFlagStatus(SWITCHES_EXTI_5_LINE) == SET) {
		SWITCH_LAST_CHANGE = 5;
		SWITCH_TIM_RESTART();
		EXTI_ClearITPendingBit(SWITCHES_EXTI_5_LINE);
	}
	else if (EXTI_GetFlagStatus(SWITCHES_EXTI_4_LINE) == SET) {
		SWITCH_LAST_CHANGE = 4;
		SWITCH_TIM_RESTART();
		EXTI_ClearITPendingBit(SWITCHES_EXTI_4_LINE);
	}*/
	else if (EXTI_GetFlagStatus(RADIO_EXTI_VSYNC_LINE) == SET) {
		extern void radioCameraVSYNCHandler(void);
		radioCameraVSYNCHandler();
		EXTI_ClearITPendingBit(RADIO_EXTI_VSYNC_LINE);
	}
	else if (EXTI_GetFlagStatus(RADIO_EXTI_DRDY_LINE) == SET) {
		radioTransactionTelemetryFromISR();
		EXTI_ClearITPendingBit(RADIO_EXTI_DRDY_LINE);
	}
	else if (EXTI_GetFlagStatus(SWITCHES_EXTI_3_LINE) == SET) {
		SWITCH_LAST_CHANGE = 3;
		SWITCH_TIM_RESTART();
		EXTI_ClearITPendingBit(SWITCHES_EXTI_3_LINE);
	}
}

void TIM4_IRQHandler(void) {
	/* Sample each bit of RC5 frame */
	RC5_Sample_Data();
}

void LANTERN_IRQHANDLER(void) {
	static uint8_t half = 0;

	if (!half) {
		TIM_OC1PolarityConfig(LANTERN_TIM, TIM_OCPolarity_Low);
		TIM_SetCompare1(LANTERN_TIM, (LANTERN_TIM_PERIOD + 1) * 3 / 4 - 1);
		TIM_OC2PolarityConfig(LANTERN_TIM, TIM_OCPolarity_High);
		TIM_SetCompare2(LANTERN_TIM, 0);
		TIM_OC3PolarityConfig(LANTERN_TIM, TIM_OCPolarity_High);
		TIM_SetCompare3(LANTERN_TIM, (LANTERN_TIM_PERIOD + 1) / 2 - 1);
		TIM_OC4PolarityConfig(LANTERN_TIM, TIM_OCPolarity_Low);
		TIM_SetCompare4(LANTERN_TIM, (LANTERN_TIM_PERIOD + 1) / 4 - 1);
	}
	else {
		TIM_OC1PolarityConfig(LANTERN_TIM, TIM_OCPolarity_High);
		TIM_SetCompare1(LANTERN_TIM, (LANTERN_TIM_PERIOD + 1) / 2 - 1);
		TIM_OC2PolarityConfig(LANTERN_TIM, TIM_OCPolarity_Low);
		TIM_SetCompare2(LANTERN_TIM, (LANTERN_TIM_PERIOD + 1) / 4 - 1);
		TIM_OC3PolarityConfig(LANTERN_TIM, TIM_OCPolarity_Low);
		TIM_SetCompare3(LANTERN_TIM, (LANTERN_TIM_PERIOD + 1) * 3 / 4 - 1);
		TIM_OC4PolarityConfig(LANTERN_TIM, TIM_OCPolarity_High);
		TIM_SetCompare4(LANTERN_TIM, 0);
	}

	TIM_ClearFlag(LANTERN_TIM, TIM_FLAG_Update);
	half = !half;
}

void ADC_IRQHandler(void) {
	if (ADC_GetFlagStatus(BATTLVL_ADC, BATTLVL_IRQ_FLAG) == SET) {
		BatteryTooLow();
		ADC_ClearFlag(BATTLVL_ADC, BATTLVL_IRQ_FLAG);
	}
	else if (ADC_GetFlagStatus(POT_ADC, POT_IRQ_FLAG) == SET) {
		setServoDownPosValue((float)ADC_GetConversionValue(POT_ADC) / 4096.0f);
		ADC_ClearFlag(POT_ADC, POT_IRQ_FLAG);
	}
}

void SWITCHES_TIM_IRQ_HANDLER(void) {
	switch (SWITCH_LAST_CHANGE) {
	case 1:
		Switch1Changed();
		break;
	case 2:
		Switch2Changed();
		break;
	case 3:
		Switch3Changed();
		break;
/*	case 4:
		Switch4Changed();
		break;
	case 5:
		Switch5Changed();
		break;
	case 6:
		Switch6Changed();
		break;*/
	default:
		break;
	}
	SWITCH_LAST_CHANGE = 0;
	TIM_Cmd(SWITCHES_TIM, DISABLE);
	TIM_ClearFlag(SWITCHES_TIM, TIM_FLAG_Update);
}

void SD_SPI_DMA_IRQHANDLER_RX(void) {
	extern void SDTransferDoneHandler(void);
	SDTransferDoneHandler();
}

void CPUUSAGE_BASE_TIM_IRQHANDLER(void) {
	OSBusyTimerHandler();
	TIM_ClearFlag(CPUUSAGE_BASE_TIM, TIM_FLAG_Update);
}

#ifdef USE_IMU_TELEMETRY
void IMU_I2C_EVENT_IRQHANDLER(void) {
	IMUI2CEVHandler();
}
#endif

void RADIO_RX_DMA_IRQHANDLER(void) {
	DMA_ClearFlag(RADIO_RX_DMA_STREAM, RADIO_RX_DMA_FLAG_TCIF);
	radioSPI_RXDMA_TCIF_FromISR();
}

void RADIO_SPI_IRQHANDLER(void) {
	if (SPI_I2S_GetFlagStatus(RADIO_SPI, SPI_I2S_FLAG_TXE) == SET) {
		radioSPI_TXE_FromISR();
	} else {
		while (1);
	}
}
