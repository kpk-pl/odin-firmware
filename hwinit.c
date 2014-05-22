#include <stm32f4xx.h>
#include <stdio.h>

#include "hwinit.h"
#include "priorities.h"
#include "hardware.h"
#include "compilation.h"
#include "hwinterface.h"
#include "main.h"

#include "TaskAsyncCallHandler.h"
#include "TaskUSB2WiFiBridge.h"

void Initialize() {
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	TIM_TimeBaseInitTypeDef	TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;

	/* Disable all interrupts, will be reenabled when scheduler starts successfully */
	__asm volatile ("cpsid i  \n");

	/* Enable clock for CCM memory */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_CCMDATARAMEN, ENABLE);

	/* COM UART configuration */
	/* Enabling clock for GPIO */
	RCC_AHB1PeriphClockCmd(COM_GPIO_CLOCK, ENABLE);
	/* Enabling clock for USART */
	COM_USART_CLOCK_FUN(COM_USART_CLOCK, ENABLE);
	/* Redirecting port lines to USART */
	GPIO_PinAFConfig(COM_GPIO, COM_GPIO_PINSOURCE_TX, COM_GPIO_AF);
	GPIO_PinAFConfig(COM_GPIO, COM_GPIO_PINSOURCE_RX, COM_GPIO_AF);
	/* Configuring GPIO pins */
	GPIO_InitStructure = (GPIO_InitTypeDef){
		.GPIO_Pin = COM_GPIO_PIN_TX | COM_GPIO_PIN_RX,
		.GPIO_Mode = GPIO_Mode_AF,
		.GPIO_OType = GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_NOPULL,
		.GPIO_Speed = GPIO_Speed_100MHz
	};
	GPIO_Init(COM_GPIO, &GPIO_InitStructure);
	/* Oversampling by 8 enable for higher speeds */
	USART_OverSampling8Cmd(COM_USART, ENABLE);
	/* Configuring UART 230400bits/s, no parity, 1 stop bit */
	USART_InitStructure = (USART_InitTypeDef){
		.USART_BaudRate = COM_USART_SPEED,
		.USART_HardwareFlowControl = USART_HardwareFlowControl_None,
		.USART_Mode = USART_Mode_Rx | USART_Mode_Tx,
		.USART_Parity = USART_Parity_No,
		.USART_StopBits = USART_StopBits_1,
		.USART_WordLength = USART_WordLength_8b
	};
	USART_Init(COM_USART, &USART_InitStructure);
	/* Configuring interrupt for USART */
	NVIC_InitStructure = (NVIC_InitTypeDef){
		.NVIC_IRQChannel = COM_NVIC_IRQn,
		.NVIC_IRQChannelCmd = ENABLE,
		.NVIC_IRQChannelPreemptionPriority = PRIORITY_ISR_COMUSART,
		.NVIC_IRQChannelSubPriority = 0
	};
	NVIC_Init(&NVIC_InitStructure);
	/* Configuring active interrupt flags for USART - Receive Ready */
	USART_ITConfig(COM_USART, USART_IT_RXNE, ENABLE);
	/* Enabling clock for DMA */
	RCC_AHB1PeriphClockCmd(COM_DMA_CLOCK, ENABLE);
	/*
	 * Configuring DMA stream for transmission over UART
	 * This stream should upload data from memory buffer to Tx peripheral
	 * Memory burst and FIFO are enabled to minimize DMA events
	 */
	DMA_InitStructure = (DMA_InitTypeDef){
		.DMA_FIFOMode = DMA_FIFOMode_Enable,
		.DMA_FIFOThreshold = DMA_FIFOThreshold_Full,
		.DMA_MemoryBurst = DMA_MemoryBurst_INC16,
		.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte,
		.DMA_MemoryInc = DMA_MemoryInc_Enable,
		.DMA_Memory0BaseAddr = 0, // temporarily
		.DMA_Mode = DMA_Mode_Normal,
		.DMA_PeripheralBaseAddr = (uint32_t) (&COM_USART -> DR),
		.DMA_PeripheralBurst = DMA_PeripheralBurst_Single,
		.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte,
		.DMA_PeripheralInc = DMA_PeripheralInc_Disable,
		.DMA_Priority = DMA_Priority_Low,
		.DMA_Channel = COM_TX_DMA_CHANNEL,
		.DMA_DIR = DMA_DIR_MemoryToPeripheral,
		.DMA_BufferSize = 1,	// Number of data items to transfer
	};
	DMA_Init(COM_TX_DMA_STREAM, &DMA_InitStructure);
	/* Disabling double buffer mode */
	DMA_DoubleBufferModeCmd(COM_TX_DMA_STREAM, DISABLE);
	/* Enabling interrupt after finished transmission */
	NVIC_InitStructure = (NVIC_InitTypeDef){
		.NVIC_IRQChannel = COM_TX_DMA_NVIC_IRQn,
		.NVIC_IRQChannelCmd = ENABLE,
		.NVIC_IRQChannelPreemptionPriority = PRIORITY_ISR_COMDMATX,
		.NVIC_IRQChannelSubPriority = 0
	};
	NVIC_Init(&NVIC_InitStructure);
	DMA_ITConfig(COM_TX_DMA_STREAM, DMA_IT_TC, ENABLE);
	/* Enabling UART */
	USART_Cmd(COM_USART, ENABLE);
	/* No buffer for stdout */
	setvbuf(stdout, 0, _IONBF, 0);


	/* Configuring UART for WiFi */
	/* Enabling clock for GPIO */
	RCC_AHB1PeriphClockCmd(WIFI_GPIO_CLOCK, ENABLE);
	/* Enabling clock for USART */
	WIFI_USART_CLOCK_FUN(WIFI_USART_CLOCK, ENABLE);
	/* Redirecting port lines to USART */
	GPIO_PinAFConfig(WIFI_GPIO_USART, WIFI_GPIO_USART_RX_PINSOURCE, WIFI_GPIO_USART_AF);
	GPIO_PinAFConfig(WIFI_GPIO_USART, WIFI_GPIO_USART_TX_PINSOURCE, WIFI_GPIO_USART_AF);
	/* Configuring GPIO pins */
	GPIO_InitStructure = (GPIO_InitTypeDef){
		.GPIO_Pin = WIFI_GPIO_USART_TX_PIN | WIFI_GPIO_USART_RX_PIN,
		.GPIO_Mode = GPIO_Mode_AF,
		.GPIO_OType = GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_NOPULL,
		.GPIO_Speed = GPIO_Speed_100MHz
	};
	GPIO_Init(WIFI_GPIO_USART, &GPIO_InitStructure);
	/* And logic pins for reset etc */
	GPIO_InitStructure = (GPIO_InitTypeDef){
		.GPIO_Pin = WIFI_GPIO_SIG_RESET_PIN | WIFI_GPIO_SIG_LMTFRES_PIN | WIFI_GPIO_SIG_CMDDATA_PIN | WIFI_GPIO_SIG_ALARM_PIN,
		.GPIO_Mode = GPIO_Mode_OUT,
		.GPIO_OType = GPIO_OType_OD,
		.GPIO_PuPd = GPIO_PuPd_NOPULL,
		.GPIO_Speed = GPIO_Speed_2MHz
	};
	/* Remember to set it default high */
	GPIO_SetBits(WIFI_GPIO_SIG, WIFI_GPIO_SIG_RESET_PIN | WIFI_GPIO_SIG_LMTFRES_PIN | WIFI_GPIO_SIG_ALARM_PIN);
	GPIO_ResetBits(WIFI_GPIO_SIG, WIFI_GPIO_SIG_CMDDATA_PIN);
	GPIO_Init(WIFI_GPIO_SIG, &GPIO_InitStructure);
	/* Set up UART */
	InitializeWiFiUART(WIFI_USART_SPEED_DEFAULT);
	/* Configuring interrupt for USART */
	NVIC_InitStructure = (NVIC_InitTypeDef){
		.NVIC_IRQChannel = WIFI_NVIC_IRQn,
		.NVIC_IRQChannelCmd = ENABLE,
		.NVIC_IRQChannelPreemptionPriority = PRIORITY_ISR_WIFIUSART,
		.NVIC_IRQChannelSubPriority = 0
	};
	NVIC_Init(&NVIC_InitStructure);
	/* Configuring active interrupt flags for USART - Receive Ready */
	USART_ITConfig(WIFI_USART, USART_IT_RXNE, ENABLE);
	/* Enabling clock for DMA */
	RCC_AHB1PeriphClockCmd(WIFI_DMA_CLOCK, ENABLE);
	/*
	 * Configuring DMA stream for transmission over UART
	 * This stream should upload data from memory buffer to Tx peripheral
	 * Memory burst and FIFO are enabled to minimize DMA events
	 */
	DMA_InitStructure = (DMA_InitTypeDef){
		.DMA_FIFOMode = DMA_FIFOMode_Enable,
		.DMA_FIFOThreshold = DMA_FIFOThreshold_Full,
		.DMA_MemoryBurst = DMA_MemoryBurst_INC16,
		.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte,
		.DMA_MemoryInc = DMA_MemoryInc_Enable,
		.DMA_Memory0BaseAddr = 0, // temporary
		.DMA_Mode = DMA_Mode_Normal,
		.DMA_PeripheralBaseAddr = (uint32_t) (&WIFI_USART -> DR),
		.DMA_PeripheralBurst = DMA_PeripheralBurst_Single,
		.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte,
		.DMA_PeripheralInc = DMA_PeripheralInc_Disable,
		.DMA_Priority = DMA_Priority_Low,
		.DMA_Channel = WIFI_TX_DMA_CHANNEL,
		.DMA_DIR = DMA_DIR_MemoryToPeripheral,
		.DMA_BufferSize = 1 // temporary
	};
	DMA_Init(WIFI_TX_DMA_STREAM, &DMA_InitStructure);
	/* Disabling double buffer mode */
	DMA_DoubleBufferModeCmd(WIFI_TX_DMA_STREAM, DISABLE);
	/* Enabling interrupt after finished transmission */
	NVIC_InitStructure = (NVIC_InitTypeDef){
		.NVIC_IRQChannel = WIFI_TX_DMA_NVIC_IRQn,
		.NVIC_IRQChannelCmd = ENABLE,
		.NVIC_IRQChannelPreemptionPriority = PRIORITY_ISR_WIFIDMATX,
		.NVIC_IRQChannelSubPriority = 0
	};
	NVIC_Init(&NVIC_InitStructure);
	DMA_ITConfig(WIFI_TX_DMA_STREAM, DMA_IT_TC, ENABLE);
	/*
	 * Configuring DMA stream for reception over USART
	 * Stream is configured to download data from Rx to memory
	 * Data is received by USART as 8-bit BYTE, it is stored in FIFO
	 * and transmitted in burst of 16 bytes
	 * Data is saved to memory as 32-bit WORD, so each burst moves 4x WORD items
	 */
	DMA_InitStructure = (DMA_InitTypeDef){
		.DMA_FIFOMode = DMA_FIFOMode_Enable,
		.DMA_FIFOThreshold = DMA_FIFOThreshold_Full,
		.DMA_MemoryBurst = DMA_MemoryBurst_INC4,
		.DMA_MemoryDataSize = DMA_MemoryDataSize_Word,
		.DMA_MemoryInc = DMA_MemoryInc_Enable,
		.DMA_Memory0BaseAddr = 0, // temporary
		.DMA_Mode = DMA_Mode_Normal,
		.DMA_PeripheralBaseAddr = (uint32_t)(&WIFI_USART -> DR),
		.DMA_PeripheralBurst = DMA_PeripheralBurst_Single,
		.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte,
		.DMA_PeripheralInc = DMA_PeripheralInc_Disable,
		.DMA_Priority = DMA_Priority_High,
		.DMA_Channel = WIFI_RX_DMA_CHANNEL,
		.DMA_DIR = DMA_DIR_PeripheralToMemory,
		.DMA_BufferSize = 1 // temporary
	},
	DMA_Init(WIFI_RX_DMA_STREAM, &DMA_InitStructure);
	DMA_FlowControllerConfig(WIFI_RX_DMA_STREAM, DMA_FlowCtrl_Memory);
	/* Enabling interrupt after receiving */
	NVIC_InitStructure = (NVIC_InitTypeDef){
		.NVIC_IRQChannel = WIFI_RX_DMA_NVIC_IRQn,
		.NVIC_IRQChannelCmd = ENABLE,
		.NVIC_IRQChannelPreemptionPriority = PRIORITY_ISR_WIFIDMARX,
		.NVIC_IRQChannelSubPriority = 0
	};
	NVIC_Init(&NVIC_InitStructure);
	DMA_ITConfig(WIFI_RX_DMA_STREAM, DMA_IT_TC, ENABLE);
	/* Enabling UART */
	USART_Cmd(WIFI_USART, ENABLE);


	/* Configuring GPIOs for motors */
	/* Clocks */
	RCC_AHB1PeriphClockCmd(MOTORL_GPIO_CLOCK, ENABLE);
	RCC_AHB1PeriphClockCmd(MOTORR_GPIO_CLOCK, ENABLE);
	/* Pins */
	GPIO_InitStructure = (GPIO_InitTypeDef){
		.GPIO_Mode = GPIO_Mode_OUT,
		.GPIO_OType = GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_NOPULL,
		.GPIO_Speed = GPIO_Speed_2MHz,
		.GPIO_Pin = MOTORL_GPIO_INA_PIN | MOTORL_GPIO_INB_PIN | MOTORL_GPIO_ENA_PIN | MOTORL_GPIO_ENB_PIN
	};
	GPIO_Init(MOTORL_GPIO, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = MOTORR_GPIO_INA_PIN | MOTORR_GPIO_INB_PIN | MOTORR_GPIO_ENA_PIN | MOTORR_GPIO_ENB_PIN;
	GPIO_Init(MOTORR_GPIO, &GPIO_InitStructure);
	/* Configuring PWMs for motors */
	/* Clock for GPIOs */
	RCC_AHB1PeriphClockCmd(MOTOR_PWM_GPIO_CLOCK, ENABLE);
	/* Clock for timer */
	MOTOR_PWM_TIM_CLOCK_FUN(MOTOR_PWM_TIM_CLOCK, ENABLE);
	/* Pins as alternate function out */
	GPIO_InitStructure = (GPIO_InitTypeDef){
		.GPIO_Mode = GPIO_Mode_AF,
		.GPIO_OType = GPIO_OType_OD,
		.GPIO_Pin = MOTOR_PWM_GPIO_LEFT_PIN | MOTOR_PWM_GPIO_RIGHT_PIN,
		.GPIO_PuPd = GPIO_PuPd_NOPULL,
		.GPIO_Speed = GPIO_Speed_50MHz
	};
	GPIO_Init(MOTOR_PWM_GPIO, &GPIO_InitStructure);
	/* Redirect pins */
	GPIO_PinAFConfig(MOTOR_PWM_GPIO, MOTOR_PWM_GPIO_LEFT_PINSOURCE, MOTOR_PWM_GPIO_AF);
	GPIO_PinAFConfig(MOTOR_PWM_GPIO, MOTOR_PWM_GPIO_RIGHT_PINSOURCE, MOTOR_PWM_GPIO_AF);
	/* Timer configuration */
	TIM_TimeBaseStructure = (TIM_TimeBaseInitTypeDef){
		.TIM_Prescaler = MOTOR_PWM_TIM_PRESCALER,			// 84MHz
		.TIM_Period = MOTOR_PWM_TIM_PERIOD,
		.TIM_ClockDivision = TIM_CKD_DIV1,
		.TIM_CounterMode = TIM_CounterMode_Up,
		.TIM_RepetitionCounter = 0
	};
	TIM_TimeBaseInit(MOTOR_PWM_TIM, &TIM_TimeBaseStructure);
	/* Outputs for timer */
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC1Init(MOTOR_PWM_TIM, &TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC2Init(MOTOR_PWM_TIM, &TIM_OCInitStructure);
	/* Buffer writing to registers */
	TIM_OC1PreloadConfig(MOTOR_PWM_TIM, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(MOTOR_PWM_TIM, TIM_OCPreload_Enable);
	/* Enable timer */
	TIM_Cmd(MOTOR_PWM_TIM, ENABLE);


	/* Configuring LEDs */
	/* GPIO clocks */
	RCC_AHB1PeriphClockCmd(LEDS_GPIO_CLOCK, ENABLE);
	/* GPIO pins */
	GPIO_InitStructure = (GPIO_InitTypeDef){
		.GPIO_Mode = GPIO_Mode_OUT,
		.GPIO_OType = GPIO_OType_PP,
		.GPIO_PuPd = GPIO_PuPd_NOPULL,
		.GPIO_Speed = GPIO_Speed_25MHz,
		.GPIO_Pin = LEDS_GPIO_1_PIN
	};
	GPIO_Init(LEDS_GPIO_1, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = LEDS_GPIO_2_PIN | LEDS_GPIO_3_PIN | LEDS_GPIO_4_PIN | LEDS_GPIO_5_PIN | LEDS_GPIO_6_PIN;
	GPIO_Init(LEDS_GPIO_26, &GPIO_InitStructure);


	/* Configuring encoders */
	/* Enabling GPIO clock */
	RCC_AHB1PeriphClockCmd(ENCODERS_GPIO_CLOCK, ENABLE);
	/* Enabling encoders timers */
	ENCODERS_TIM_L_CLOCK_FUN(ENCODERS_TIM_L_CLOCK, ENABLE);
	ENCODERS_TIM_R_CLOCK_FUN(ENCODERS_TIM_R_CLOCK, ENABLE);
	/* Configuring pins as inputs alternate function */
	GPIO_InitStructure = (GPIO_InitTypeDef){
		.GPIO_Mode = GPIO_Mode_AF,
		.GPIO_PuPd = GPIO_PuPd_UP,
		.GPIO_Speed = GPIO_Speed_50MHz,
		.GPIO_OType = GPIO_OType_PP,
		.GPIO_Pin = ENCODERS_GPIO_LA_PIN
	};
	GPIO_Init(ENCODERS_GPIO_LA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = ENCODERS_GPIO_LB_PIN;
	GPIO_Init(ENCODERS_GPIO_LB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = ENCODERS_GPIO_RA_PIN;
	GPIO_Init(ENCODERS_GPIO_RA, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = ENCODERS_GPIO_RB_PIN;
	GPIO_Init(ENCODERS_GPIO_RB, &GPIO_InitStructure);
	/* Redirecting GPIO lines to timers */
	GPIO_PinAFConfig(ENCODERS_GPIO_LA, ENCODERS_GPIO_LA_PINSOURCE, ENCODERS_GPIO_L_AF);
	GPIO_PinAFConfig(ENCODERS_GPIO_LB, ENCODERS_GPIO_LB_PINSOURCE, ENCODERS_GPIO_L_AF);
	GPIO_PinAFConfig(ENCODERS_GPIO_RA, ENCODERS_GPIO_RA_PINSOURCE, ENCODERS_GPIO_R_AF);
	GPIO_PinAFConfig(ENCODERS_GPIO_RB, ENCODERS_GPIO_RB_PINSOURCE, ENCODERS_GPIO_R_AF);
	/* Timers configuration */
	TIM_TimeBaseStructure = (TIM_TimeBaseInitTypeDef){
		.TIM_Prescaler = (uint16_t)0x0000,
		.TIM_Period = (uint32_t)0xFFFFFFFFL,
		.TIM_ClockDivision = TIM_CKD_DIV1,
		.TIM_CounterMode = TIM_CounterMode_Up,
		.TIM_RepetitionCounter = 0
	};
	TIM_TimeBaseInit(ENCODERS_TIM_L, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(ENCODERS_TIM_R, &TIM_TimeBaseStructure);
	/* Encoder interface */
	TIM_EncoderInterfaceConfig(ENCODERS_TIM_L, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_EncoderInterfaceConfig(ENCODERS_TIM_R, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	/* Set start value in between max and min value */
	TIM_SetCounter(ENCODERS_TIM_L, (uint32_t)0x80000000L);
	TIM_SetCounter(ENCODERS_TIM_R, (uint32_t)0x80000000L);
	/* Enable timers */
	TIM_Cmd(ENCODERS_TIM_L, ENABLE);
	TIM_Cmd(ENCODERS_TIM_R, ENABLE);


	/* Configuring ADC for battery level check */
	/* Enabling clocks for GPIO & ADC */
	RCC_AHB1PeriphClockCmd(BATTLVL_GPIO_CLOCK, ENABLE);
	BATTLVL_ADC_CLOCK_FUN(BATTLVL_ADC_CLOCK, ENABLE);
	/* Configuring GPIO Pin */
	GPIO_InitStructure = (GPIO_InitTypeDef){
		.GPIO_Mode = GPIO_Mode_AN,
		.GPIO_Pin = BATTLVL_GPIO_PIN,
		.GPIO_PuPd = GPIO_PuPd_NOPULL,
		.GPIO_Speed = GPIO_Speed_25MHz,
		.GPIO_OType = GPIO_OType_PP
	};
	GPIO_Init(BATTLVL_GPIO, &GPIO_InitStructure);
	/* ADC common init */
	ADC_CommonInitStructure = (ADC_CommonInitTypeDef){
		.ADC_Mode = ADC_Mode_Independent,
		.ADC_Prescaler = ADC_Prescaler_Div8,
		.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles,
		.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled
	};
	ADC_CommonInit(&ADC_CommonInitStructure);
	/* Configuring ADC line, continuous mode, 12 bit resolution */
	ADC_InitStructure = (ADC_InitTypeDef){
		.ADC_ContinuousConvMode = ENABLE,
		.ADC_DataAlign = ADC_DataAlign_Right,
		.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1,
		.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None,
		.ADC_NbrOfConversion = 1,
		.ADC_Resolution = ADC_Resolution_12b,
		.ADC_ScanConvMode = DISABLE
	};
	ADC_Init(BATTLVL_ADC, &ADC_InitStructure);
	/* Configuring channel */
	ADC_RegularChannelConfig(BATTLVL_ADC, BATTLVL_ADC_CHANNEL, 1, ADC_SampleTime_480Cycles);
	/* Analog watchdog configuration */
	ADC_AnalogWatchdogCmd(BATTLVL_ADC, ADC_AnalogWatchdog_SingleRegEnable);
	ADC_AnalogWatchdogSingleChannelConfig(BATTLVL_ADC, BATTLVL_ADC_CHANNEL);
	ADC_AnalogWatchdogThresholdsConfig(BATTLVL_ADC, 0x0FFF, (uint16_t)(BATTLVL_THRESHOLD_VOLTAGE/BATTLVL_CONV_2_VOLTAGE));
	/* Interrupt for analog watchdog */
	NVIC_InitStructure = (NVIC_InitTypeDef){
		.NVIC_IRQChannel = ADC_IRQn,
		.NVIC_IRQChannelCmd = ENABLE,
		.NVIC_IRQChannelPreemptionPriority = PRIORITY_ISR_ADC,
		.NVIC_IRQChannelSubPriority = 0
	};
	NVIC_Init(&NVIC_InitStructure);
	ADC_ITConfig(BATTLVL_ADC, ADC_IT_AWD, ENABLE);
	/* Enabling ADC and starting first conversion */
	ADC_Cmd(BATTLVL_ADC, ENABLE);
	ADC_SoftwareStartConv(BATTLVL_ADC);


	/* Configuring ADC to read value from potentiometer */
	/* Enabling clocks */
	RCC_AHB1PeriphClockCmd(POT_GPIO_CLOCK, ENABLE);
	POT_ADC_CLOCK_FUN(POT_ADC_CLOCK, ENABLE);
	/* Configuring GPIO pin */
	GPIO_InitStructure = (GPIO_InitTypeDef){
		.GPIO_Mode = GPIO_Mode_AN,
		.GPIO_Pin = POT_GPIO_PIN,
		.GPIO_PuPd = GPIO_PuPd_NOPULL,
		.GPIO_Speed = GPIO_Speed_25MHz,
		.GPIO_OType = GPIO_OType_PP
	};
	GPIO_Init(POT_GPIO, &GPIO_InitStructure);
	/* ADC common init was done before */
	/* ADC configuring, 12-bit single conversion triggered by TIM8 CC1 */
	ADC_InitStructure = (ADC_InitTypeDef){
		.ADC_ContinuousConvMode = DISABLE,
		.ADC_DataAlign = ADC_DataAlign_Right,
		.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T8_CC1,
		.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising,
		.ADC_NbrOfConversion = 1,
		.ADC_Resolution = ADC_Resolution_12b,
		.ADC_ScanConvMode = DISABLE
	};
	ADC_Init(POT_ADC, &ADC_InitStructure);
	/* Configuring channel */
	ADC_RegularChannelConfig(POT_ADC, POT_ADC_CHANNEL, 1, ADC_SampleTime_480Cycles);
	/* Interrupt on finished conversion */
	/* NVIC already configured */
	ADC_ITConfig(POT_ADC, ADC_IT_EOC, ENABLE);
	/* Configuring timer which will trigger conversion */
	/* Clock */
	HELPER_TIM_CLOCK_FUN(HELPER_TIM_CLOCK, ENABLE);
	/* Timer configuration */
	TIM_TimeBaseStructure = (TIM_TimeBaseInitTypeDef){
		.TIM_RepetitionCounter = 0,
		.TIM_Prescaler = HELPER_TIM_PRESCALER,
		.TIM_Period = HELPER_TIM_PERIOD,
		.TIM_ClockDivision = TIM_CKD_DIV1,
		.TIM_CounterMode = TIM_CounterMode_Up
	};
	TIM_TimeBaseInit(HELPER_TIM, &TIM_TimeBaseStructure);
	/* Capture Compare channel 1 */
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = HELPER_TIM_PERIOD / 2 + 1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init(HELPER_TIM, &TIM_OCInitStructure);
	TIM_CtrlPWMOutputs(HELPER_TIM, ENABLE);
	/* Enable timer */
	TIM_Cmd(HELPER_TIM, ENABLE);
	/* Enabling ADC */
	ADC_Cmd(POT_ADC, ENABLE);
	/* Force ADC conversion */
	ADC_SoftwareStartConv(POT_ADC);


	/* Configuring servo */
	/* Clock for GPIO */
	RCC_AHB1PeriphClockCmd(SERVO_GPIO_CLOCK, ENABLE);
	/* Clock for timer */
	SERVO_TIM_CLOCK_FUN(SERVO_TIM_CLOCK, ENABLE);
	/* Pins as alternate function out */
	GPIO_InitStructure = (GPIO_InitTypeDef){
		.GPIO_Mode = GPIO_Mode_AF,
		.GPIO_OType = GPIO_OType_PP,
		.GPIO_Pin = SERVO_GPIO_PIN,
		.GPIO_PuPd = GPIO_PuPd_NOPULL,
		.GPIO_Speed = GPIO_Speed_2MHz
	};
	GPIO_Init(SERVO_GPIO, &GPIO_InitStructure);
	/* Redirect pins */
	GPIO_PinAFConfig(SERVO_GPIO, SERVO_GPIO_PINSOURCE, SERVO_GPIO_AF);
	/* Timer configuration */
	TIM_TimeBaseStructure = (TIM_TimeBaseInitTypeDef){
		.TIM_RepetitionCounter = 0,
		.TIM_Prescaler = SERVO_TIM_PRESCALER,
		.TIM_Period = SERVO_TIM_PERIOD,
		.TIM_ClockDivision = TIM_CKD_DIV1,
		.TIM_CounterMode = TIM_CounterMode_Up
	};
	TIM_TimeBaseInit(SERVO_TIM, &TIM_TimeBaseStructure);
	/* Outputs for timer */
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC2Init(SERVO_TIM, &TIM_OCInitStructure);
	/* Set correct initial value for the servo */
	setPenUp();
	/* Enable preload */
	TIM_OC2PreloadConfig(SERVO_TIM, TIM_OCPreload_Enable);
	/* DO NOT enable timer - will be enabled using enablePen(ENABLE) from CLI or RC5 */


	/* Configure independent watchdog */
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_16);
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	/* It should give 2s */
	IWDG_SetReload(4005);
	/* Set halt mode in debug */
	DBGMCU_APB1PeriphConfig(DBGMCU_IWDG_STOP, ENABLE);
	/* Not started here, but in TaskLED */


	/* Configure two timers to time OS busy percent */
	CPUUSAGE_TIM_CLOCK_FUN(CPUUSAGE_TIM_CLOCKS, ENABLE);
	TIM_TimeBaseStructure = (TIM_TimeBaseInitTypeDef){
		.TIM_Prescaler = CPUUSAGE_TIM_PRESCALER,
		.TIM_Period = CPUUSAGE_TIM_PERIOD,
		.TIM_ClockDivision = TIM_CKD_DIV1,
		.TIM_CounterMode = TIM_CounterMode_Up,
		.TIM_RepetitionCounter = 0
	};
	TIM_TimeBaseInit(CPUUSAGE_BASE_TIM, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(CPUUSAGE_CNT_TIM, &TIM_TimeBaseStructure);
	TIM_ITConfig(CPUUSAGE_BASE_TIM, TIM_IT_Update, ENABLE);
	NVIC_InitStructure = (NVIC_InitTypeDef){
		.NVIC_IRQChannel = CPUUSAGE_BASE_TIM_IRQn,
		.NVIC_IRQChannelCmd = ENABLE,
		.NVIC_IRQChannelPreemptionPriority = PRIORITY_ISR_OSBUSY,
		.NVIC_IRQChannelSubPriority = 0
	};
	NVIC_Init(&NVIC_InitStructure);
	TIM_Cmd(CPUUSAGE_BASE_TIM, ENABLE);


	/* Configuring switches and external interrupts - last of all*/
	/* GPIO clock */
	RCC_AHB1PeriphClockCmd(SWITCHES_GPIO_CLOCK, ENABLE);
	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	/* Pins as inputs with pullup */
	GPIO_InitStructure = (GPIO_InitTypeDef){
		.GPIO_Mode = GPIO_Mode_IN,
		.GPIO_Pin = SWITCHES_GPIO_1_PIN | SWITCHES_GPIO_2_PIN | SWITCHES_GPIO_3_PIN |
					SWITCHES_GPIO_4_PIN | SWITCHES_GPIO_5_PIN | SWITCHES_GPIO_6_PIN,
		.GPIO_PuPd = GPIO_PuPd_UP,
		.GPIO_Speed = GPIO_Speed_2MHz,
		.GPIO_OType = GPIO_OType_PP
	};
	GPIO_Init(SWITCHES_GPIO, &GPIO_InitStructure);
	/* Connect EXTI line to GPIO */
	SYSCFG_EXTILineConfig(SWITCHES_EXTI_PORTSOURCE, SWITCHES_EXTI_1_PINSOURCE);
	SYSCFG_EXTILineConfig(SWITCHES_EXTI_PORTSOURCE, SWITCHES_EXTI_2_PINSOURCE);
	SYSCFG_EXTILineConfig(SWITCHES_EXTI_PORTSOURCE, SWITCHES_EXTI_3_PINSOURCE);
	SYSCFG_EXTILineConfig(SWITCHES_EXTI_PORTSOURCE, SWITCHES_EXTI_4_PINSOURCE);
	SYSCFG_EXTILineConfig(SWITCHES_EXTI_PORTSOURCE, SWITCHES_EXTI_5_PINSOURCE);
	SYSCFG_EXTILineConfig(SWITCHES_EXTI_PORTSOURCE, SWITCHES_EXTI_6_PINSOURCE);
	/* EXTI config */
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	/* Switch 1 */
	EXTI_InitStructure.EXTI_Line = SWITCHES_EXTI_1_LINE;
	EXTI_Init(&EXTI_InitStructure);
	if (getSwitchStatus(1) == OFF) enableUSB(DISABLE);
	/* Switch 2 */
	EXTI_InitStructure.EXTI_Line = SWITCHES_EXTI_2_LINE;
	EXTI_Init(&EXTI_InitStructure);
	if (getSwitchStatus(2) == OFF) enableWiFi(DISABLE);
	/* Switch 3 */
	EXTI_InitStructure.EXTI_Line = SWITCHES_EXTI_3_LINE;
	EXTI_Init(&EXTI_InitStructure);
	if (getSwitchStatus(3) == ON) {
		AsyncCall_Type call = {
			.CallVoid = TaskUSB2WiFiBridgeConstructor,
			.Type = AsyncCallProc_Void
		};
		xQueueSend(AsyncCallHandlerQueue, &call, 0);
	}
	/* Switch 4 */
	EXTI_InitStructure.EXTI_Line = SWITCHES_EXTI_4_LINE;
	EXTI_Init(&EXTI_InitStructure);
	/* Switch 5 */
	EXTI_InitStructure.EXTI_Line = SWITCHES_EXTI_5_LINE;
	EXTI_Init(&EXTI_InitStructure);
	/* Switch 6 */
	EXTI_InitStructure.EXTI_Line = SWITCHES_EXTI_6_LINE;
	EXTI_Init(&EXTI_InitStructure);
	// current lantern state is checked and set during lantern initialization
	/* NVIC config */
	NVIC_InitStructure = (NVIC_InitTypeDef){
		.NVIC_IRQChannel = EXTI9_5_IRQn,
		.NVIC_IRQChannelCmd = ENABLE,
		.NVIC_IRQChannelPreemptionPriority = PRIORITY_ISR_SWITCHES,
		.NVIC_IRQChannelSubPriority = 0
	};
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	/* Configure timer used to debounce switches */
	/* Clock enable */
	SWITCHES_TIM_CLOCK_FUN(SWITCHES_TIM_CLOCK, ENABLE);
	/* Frequency configuration */
	TIM_TimeBaseStructure = (TIM_TimeBaseInitTypeDef){
		.TIM_Prescaler = SWITCHES_TIM_PRESCALER,
		.TIM_CounterMode = TIM_CounterMode_Up,
		.TIM_Period = SWITCHES_TIM_PERIOD,
		.TIM_ClockDivision = TIM_CKD_DIV1,
		.TIM_RepetitionCounter = 0
	};
	TIM_TimeBaseInit(SWITCHES_TIM, &TIM_TimeBaseStructure);
	/* Interrupt generation */
	TIM_ITConfig(SWITCHES_TIM, TIM_IT_Update, ENABLE);
	/* Interrupt configuration */
	NVIC_InitStructure = (NVIC_InitTypeDef){
		.NVIC_IRQChannel = SWITCHES_NVIC_TIM_IRQn,
		.NVIC_IRQChannelCmd = ENABLE,
		.NVIC_IRQChannelPreemptionPriority = PRIORITY_ISR_SWITCHES_TIM,
		.NVIC_IRQChannelSubPriority = 0
	};
	NVIC_Init(&NVIC_InitStructure);
	/* DO NOT START TIMER HERE */


	/* Configuring lantern timer to generate pattern */
	/* Enable clocks */
	LANTERN_TIM_CLOCK_FUN(LANTERN_TIM_CLOCK, ENABLE);
	RCC_AHB1PeriphClockCmd(LANTERN_GPIO_CLOCK, ENABLE);
	/* Do not configure GPIO pins, will be done by enableLantern call */
	/* Connect TIM Channels to GPIOs */
	GPIO_PinAFConfig(LANTERN_GPIO, LANTERN_GPIO_PINSOURCE1, LANTERN_GPIO_AF);
	GPIO_PinAFConfig(LANTERN_GPIO, LANTERN_GPIO_PINSOURCE2, LANTERN_GPIO_AF);
	GPIO_PinAFConfig(LANTERN_GPIO, LANTERN_GPIO_PINSOURCE3, LANTERN_GPIO_AF);
	GPIO_PinAFConfig(LANTERN_GPIO, LANTERN_GPIO_PINSOURCE4, LANTERN_GPIO_AF);
	/* Time base configuration */
	TIM_TimeBaseStructure = (TIM_TimeBaseInitTypeDef){
		.TIM_RepetitionCounter = 0,
		.TIM_Prescaler = LANTERN_TIM_PRESCALER,
		.TIM_Period = LANTERN_TIM_PERIOD,
		.TIM_ClockDivision = TIM_CKD_DIV1,
		.TIM_CounterMode = TIM_CounterMode_Up
	};
	TIM_TimeBaseInit(LANTERN_TIM, &TIM_TimeBaseStructure);
	/* Output Compare Toggle Mode configuration - all powered down; this simplifies turning it off initially and is shorter */
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = LANTERN_TIM_PERIOD;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC1Init(LANTERN_TIM, &TIM_OCInitStructure);
	TIM_OC2Init(LANTERN_TIM, &TIM_OCInitStructure);
	TIM_OC3Init(LANTERN_TIM, &TIM_OCInitStructure);
	TIM_OC4Init(LANTERN_TIM, &TIM_OCInitStructure);
	/* Configure registers to allow immediate writing without shadow register */
	TIM_OC1PreloadConfig(LANTERN_TIM, TIM_OCPreload_Disable);
	TIM_OC2PreloadConfig(LANTERN_TIM, TIM_OCPreload_Disable);
	TIM_OC3PreloadConfig(LANTERN_TIM, TIM_OCPreload_Disable);
	TIM_OC4PreloadConfig(LANTERN_TIM, TIM_OCPreload_Disable);
	/* Enable outputs */
	TIM_CtrlPWMOutputs(LANTERN_TIM, ENABLE);
	/* Configuring interrupt on TIM Update */
	NVIC_InitStructure = (NVIC_InitTypeDef){
		.NVIC_IRQChannel = LANTERN_NVIC_IRQn,
		.NVIC_IRQChannelPreemptionPriority = PRIORITY_ISR_LANTERN,
		.NVIC_IRQChannelCmd = ENABLE,
		.NVIC_IRQChannelSubPriority = 0
	};
	NVIC_Init(&NVIC_InitStructure);
	TIM_ITConfig(LANTERN_TIM, TIM_IT_Update, ENABLE);
	/* Do not enable timer - will be done in enableLantern call */
	/* Init lantern properly to be on or off at startup */
	enableLantern(getSwitchStatus(6) == OFF ? DISABLE : ENABLE);
}

void InitializeWiFiUART(uint32_t speed) {
	/* Configuring UART, no parity, 1 stop bit */
	 USART_InitTypeDef USART_InitStructure = {
		.USART_BaudRate = speed,
		.USART_HardwareFlowControl = USART_HardwareFlowControl_None,
		.USART_Mode = USART_Mode_Rx | USART_Mode_Tx,
		.USART_Parity = USART_Parity_No,
		.USART_StopBits = USART_StopBits_1,
		.USART_WordLength = USART_WordLength_8b
	};
	USART_Init(WIFI_USART, &USART_InitStructure);
}
