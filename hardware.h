#ifndef __HARDWARE_H__
#define __HARDWARE_H__

/*
 * Hardware using
 * TIM1			lantern LEDs pattern generation
 * TIM2			encoder input on channels CH1 (PA15) & CH2 (PB3)
 * TIM3			PWM for motors on channels CH1 (PB4) & CH2 (PB5)
 * TIM4			CH3 connected to RC5 input, but not used that way. Period counter for RC5
 * TIM5			encoder input on channels CH1 (PA0) & CH2 (PA1)
 * TIM6			used to debounce switches
 * TIM8			ADC2 trigger @ 10Hz
 * TIM9			servo PWM on CH2 (PE6)
 * TIM14		OS busy-time calculation base time
 * TIM13		OS busy-time calculation busy time
 *
 * I2C1			IMU interface SCL (PB8) & SDA (PB9)
 * USART1		WiFi output TX(PA9), RX(PA10), CTS(PA11), RTS(PA12)
 * UART4		UART-to-USB interface, TX (PC10), RX (PC11)
 * ADC1			battery voltage measurement on Channel 10 (PC0)
 * ADC2			potentiometer voltage reading on Channel 3 (PA3)
 * DMA1
 * 		- CH4ST4	UART-to-USB transfer using safePrint
 * 		- CH4ST2	UART-to-USB transfer, reading huge blocks of data
 * EXTI
 * 		- Line0		IMU gyroscope data ready
 * 		- Line8-13	switches
 * 		- Line14	used by RC5 decoder
 *
 *
 * Important remarks
 * 	- SWITCHES are default configured in Initialize()
 *
 *
 * Switches:
 * 	1: USB enable
 * 	2: WIFI enable
 * 	3: USB-WIFI bridge enable
 * 	4:
 * 	5:
 * 	6: lantern enable
 */


#define COM_GPIO						GPIOC
#define COM_GPIO_CLOCK					RCC_AHB1Periph_GPIOC
#define COM_GPIO_PIN_TX					GPIO_Pin_10
#define COM_GPIO_PINSOURCE_TX			GPIO_PinSource10
#define COM_GPIO_PIN_RX					GPIO_Pin_11
#define COM_GPIO_PINSOURCE_RX			GPIO_PinSource11
#define COM_GPIO_AF						GPIO_AF_UART4
#define COM_USART						UART4
#define COM_USART_SPEED					460800				// Rx ISR is too slow for more
#define COM_USART_CLOCK_FUN				RCC_APB1PeriphClockCmd
#define COM_USART_CLOCK					RCC_APB1Periph_UART4
#define COM_NVIC_IRQn					UART4_IRQn
#define COM_USART_IRQHANDLER			UART4_IRQHandler
#define COM_DMA							DMA1
#define COM_DMA_CLOCK					RCC_AHB1Periph_DMA1
#define COM_TX_DMA_CHANNEL				DMA_Channel_4
#define COM_TX_DMA_STREAM				DMA1_Stream4
#define COM_TX_DMA_FLAG_TCIF			DMA_FLAG_TCIF4
#define COM_TX_DMA_NVIC_IRQn			DMA1_Stream4_IRQn
#define COM_TX_DMA_IRQHANDLER			DMA1_Stream4_IRQHandler
#define COM_RX_DMA_CHANNEL				DMA_Channel_4
#define COM_RX_DMA_STREAM				DMA1_Stream2
#define COM_RX_DMA_FLAG_TCIF			DMA_FLAG_TCIF2
#define COM_RX_DMA_NVIC_IRQn			DMA1_Stream2_IRQn
#define COM_RX_DMA_IRQHANDLER			DMA1_Stream2_IRQHandler


#define LANTERN_GPIO					GPIOE
#define LANTERN_GPIO_CLOCK				RCC_AHB1Periph_GPIOE
#define LANTERN_GPIO_PIN1				GPIO_Pin_9
#define LANTERN_GPIO_PIN2				GPIO_Pin_11
#define LANTERN_GPIO_PIN3				GPIO_Pin_13
#define LANTERN_GPIO_PIN4				GPIO_Pin_14
#define LANTERN_GPIO_PINSOURCE1			GPIO_PinSource9
#define LANTERN_GPIO_PINSOURCE2			GPIO_PinSource11
#define LANTERN_GPIO_PINSOURCE3			GPIO_PinSource13
#define LANTERN_GPIO_PINSOURCE4			GPIO_PinSource14
#define LANTERN_GPIO_AF					GPIO_AF_TIM1
#define LANTERN_TIM						TIM1
#define LANTERN_TIM_CLOCK				RCC_APB2Periph_TIM1
#define LANTERN_TIM_CLOCK_FUN			RCC_APB2PeriphClockCmd
#define LANTERN_NVIC_IRQn				TIM1_UP_TIM10_IRQn
#define LANTERN_IRQHANDLER				TIM1_UP_TIM10_IRQHandler
#define LANTERN_TIM_BASE_FREQ			50000
#define LANTERN_FREQ					5
#define LANTERN_TIM_PRESCALER			((uint16_t)((SystemCoreClock) / LANTERN_TIM_BASE_FREQ) - 1)
#define LANTERN_TIM_PERIOD				(LANTERN_TIM_BASE_FREQ / LANTERN_FREQ - 1)


#define IMU_I2C							I2C1
#define IMU_I2C_CLOCK_FUN				RCC_APB1PeriphClockCmd
#define IMU_I2C_CLOCK					RCC_APB1Periph_I2C1
#define IMU_I2C_SPEED					300000//150000
#define IMU_GPIO						GPIOB
#define IMU_GPIO_CLOCK					RCC_AHB1Periph_GPIOB
#define IMU_GPIO_SCL_PIN				GPIO_Pin_8
#define IMU_GPIO_SCL_PINSOURCE			GPIO_PinSource8
#define IMU_GPIO_SDA_PIN				GPIO_Pin_9
#define IMU_GPIO_SDA_PINSOURCE			GPIO_PinSource9
#define IMU_AF							GPIO_AF_I2C1
#define IMU_GPIO_INT					GPIOD
#define IMU_GPIO_INT_CLOCK				RCC_AHB1Periph_GPIOD
#define IMU_GPIO_GINT_PIN				GPIO_Pin_1
#define IMU_GPIO_AINT_PIN				GPIO_Pin_2
#define IMU_GPIO_MINT_PIN				GPIO_Pin_3
#define IMU_EXTI_GINT_LINE				EXTI_Line1
#define IMU_EXTI_AINT_LINE				EXTI_Line2
#define IMU_EXTI_MINT_LINE				EXTI_Line3
#define IMU_EXTI_GINT_TRIGGER			EXTI_Trigger_Rising
#define IMU_EXTI_AINT_TRIGGER			EXTI_Trigger_Rising
#define IMU_EXTI_MINT_TRIGGER			EXTI_Trigger_Rising
#define IMU_EXTI_GINT_IRQn				EXTI1_IRQn
#define IMU_EXTI_AINT_IRQn				EXTI2_IRQn
#define IMU_EXTI_MINT_IRQn				EXTI3_IRQn
#define IMU_EXTI_INT_PORTSOURCE			EXTI_PortSourceGPIOD
#define IMU_EXTI_GINT_PINSOURCE			EXTI_PinSource1
#define IMU_EXTI_AINT_PINSOURCE 		EXTI_PinSource2
#define IMU_EXTI_MINT_PINSOURCE			EXTI_PinSource3
#define IMU_GINT_IRQHANDLER				EXTI1_IRQHandler
#define IMU_AINT_IRQHANDLER				EXTI2_IRQHandler
#define IMU_MINT_IRQHANDLER				EXTI3_IRQHandler
#define IMU_I2C_EVENT_IRQn				I2C1_EV_IRQn
#define IMU_I2C_EVENT_IRQHANDLER		I2C1_EV_IRQHandler


#define MOTORL_GPIO						GPIOE
#define MOTORL_GPIO_CLOCK				RCC_AHB1Periph_GPIOE
#define MOTORL_GPIO_INA_PIN				GPIO_Pin_5
#define MOTORL_GPIO_INB_PIN				GPIO_Pin_2
#define MOTORL_GPIO_ENA_PIN				GPIO_Pin_4
#define MOTORL_GPIO_ENB_PIN				GPIO_Pin_3

#define MOTORR_GPIO						GPIOD
#define MOTORR_GPIO_CLOCK				RCC_AHB1Periph_GPIOD
#define MOTORR_GPIO_INA_PIN				GPIO_Pin_7
#define MOTORR_GPIO_INB_PIN				GPIO_Pin_4
#define MOTORR_GPIO_ENA_PIN				GPIO_Pin_6
#define MOTORR_GPIO_ENB_PIN				GPIO_Pin_5

#define MOTOR_PWM_GPIO					GPIOB
#define MOTOR_PWM_GPIO_CLOCK			RCC_AHB1Periph_GPIOB
#define MOTOR_PWM_GPIO_LEFT_PIN			GPIO_Pin_4
#define MOTOR_PWM_GPIO_RIGHT_PIN		GPIO_Pin_5
#define MOTOR_PWM_GPIO_LEFT_PINSOURCE	GPIO_PinSource4
#define MOTOR_PWM_GPIO_RIGHT_PINSOURCE	GPIO_PinSource5
#define MOTOR_PWM_GPIO_AF				GPIO_AF_TIM3
#define MOTOR_PWM_TIM					TIM3
#define MOTOR_PWM_TIM_CLOCK				RCC_APB1Periph_TIM3
#define MOTOR_PWM_TIM_CLOCK_FUN			RCC_APB1PeriphClockCmd
#define MOTOR_PWM_LEFT_SET_FUN			TIM_SetCompare1
#define MOTOR_PWM_RIGHT_SET_FUN			TIM_SetCompare2
#define MOTOR_PWM_FREQ					15000
#define MOTOR_PWM_TIM_PRESCALER			0
#define MOTOR_PWM_TIM_PERIOD			((SystemCoreClock / 2) / MOTOR_PWM_FREQ - 1)


#define SWITCHES_GPIO					GPIOD
#define SWITCHES_GPIO_CLOCK				RCC_AHB1Periph_GPIOD
#define SWITCHES_GPIO_6_PIN				GPIO_Pin_13
#define SWITCHES_GPIO_5_PIN				GPIO_Pin_12
#define SWITCHES_GPIO_4_PIN				GPIO_Pin_11
#define SWITCHES_GPIO_3_PIN				GPIO_Pin_10
#define SWITCHES_GPIO_2_PIN				GPIO_Pin_9
#define SWITCHES_GPIO_1_PIN				GPIO_Pin_8
#define SWITCHES_EXTI_6_LINE			EXTI_Line13
#define SWITCHES_EXTI_5_LINE			EXTI_Line12
#define SWITCHES_EXTI_4_LINE			EXTI_Line11
#define SWITCHES_EXTI_3_LINE			EXTI_Line10
#define SWITCHES_EXTI_2_LINE			EXTI_Line9
#define SWITCHES_EXTI_1_LINE			EXTI_Line8
#define SWITCHES_EXTI_PORTSOURCE		EXTI_PortSourceGPIOD
#define SWITCHES_EXTI_6_PINSOURCE		EXTI_PinSource13
#define SWITCHES_EXTI_5_PINSOURCE		EXTI_PinSource12
#define SWITCHES_EXTI_4_PINSOURCE		EXTI_PinSource11
#define SWITCHES_EXTI_3_PINSOURCE		EXTI_PinSource10
#define SWITCHES_EXTI_2_PINSOURCE		EXTI_PinSource9
#define SWITCHES_EXTI_1_PINSOURCE		EXTI_PinSource8
#define SWITCHES_TIM					TIM6
#define SWITCHES_TIM_CLOCK				RCC_APB1Periph_TIM6
#define SWITCHES_TIM_CLOCK_FUN			RCC_APB1PeriphClockCmd
#define SWITCHES_TIM_DELAY_MS			5
#define SWITCHES_TIM_PRESCALER			((uint16_t)((SystemCoreClock / 2) / 1000000) - 1)
#define SWITCHES_TIM_PERIOD				(1000000 / (1000 / SWITCHES_TIM_DELAY_MS) - 1)
#define SWITCHES_NVIC_TIM_IRQn			TIM6_DAC_IRQn
#define SWITCHES_TIM_IRQ_HANDLER		TIM6_DAC_IRQHandler


#define SERVO_GPIO						GPIOE
#define SERVO_GPIO_CLOCK				RCC_AHB1Periph_GPIOE
#define SERVO_GPIO_PIN					GPIO_Pin_6
#define SERVO_GPIO_PINSOURCE			GPIO_PinSource6
#define SERVO_GPIO_AF					GPIO_AF_TIM9
#define SERVO_TIM						TIM9
#define SERVO_TIM_CLOCK					RCC_APB2Periph_TIM9
#define SERVO_TIM_CLOCK_FUN				RCC_APB2PeriphClockCmd
#define SERVO_TIM_PWM_SPEED				50
#define SERVO_TIM_BASE_FREQ				100000
#define SERVO_TIM_PRESCALER				((uint16_t)((SystemCoreClock) / SERVO_TIM_BASE_FREQ) - 1)
#define SERVO_TIM_PERIOD				(SERVO_TIM_BASE_FREQ / SERVO_TIM_PWM_SPEED - 1)


#define LEDS_GPIO_1						GPIOB
#define LEDS_GPIO_26					GPIOE
#define LEDS_GPIO_CLOCK					RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOE
#define LEDS_GPIO_1_PIN					GPIO_Pin_1
#define LEDS_GPIO_2_PIN					GPIO_Pin_7
#define LEDS_GPIO_3_PIN					GPIO_Pin_8
#define LEDS_GPIO_4_PIN					GPIO_Pin_10
#define LEDS_GPIO_5_PIN					GPIO_Pin_12
#define LEDS_GPIO_6_PIN					GPIO_Pin_15


#define ENCODERS_GPIO_LA				GPIOA
#define ENCODERS_GPIO_LB				GPIOA
#define ENCODERS_GPIO_RA				GPIOB
#define ENCODERS_GPIO_RB				GPIOA
#define ENCODERS_GPIO_CLOCK				RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB
#define ENCODERS_GPIO_LA_PIN			GPIO_Pin_0
#define ENCODERS_GPIO_LB_PIN			GPIO_Pin_1
#define ENCODERS_GPIO_RA_PIN			GPIO_Pin_3
#define ENCODERS_GPIO_RB_PIN			GPIO_Pin_15
#define ENCODERS_GPIO_LA_PINSOURCE		GPIO_PinSource0
#define ENCODERS_GPIO_LB_PINSOURCE		GPIO_PinSource1
#define ENCODERS_GPIO_RA_PINSOURCE		GPIO_PinSource3
#define ENCODERS_GPIO_RB_PINSOURCE		GPIO_PinSource15
#define ENCODERS_GPIO_L_AF				GPIO_AF_TIM5
#define ENCODERS_GPIO_R_AF				GPIO_AF_TIM2
#define ENCODERS_TIM_L					TIM5
#define ENCODERS_TIM_L_CLOCK			RCC_APB1Periph_TIM5
#define ENCODERS_TIM_L_CLOCK_FUN		RCC_APB1PeriphClockCmd
#define ENCODERS_TIM_R					TIM2
#define ENCODERS_TIM_R_CLOCK			RCC_APB1Periph_TIM2
#define ENCODERS_TIM_R_CLOCK_FUN		RCC_APB1PeriphClockCmd


#define BATTLVL_GPIO					GPIOC
#define BATTLVL_GPIO_CLOCK				RCC_AHB1Periph_GPIOC
#define BATTLVL_GPIO_PIN				GPIO_Pin_0
#define BATTLVL_ADC						ADC1
#define BATTLVL_ADC_CLOCK				RCC_APB2Periph_ADC1
#define BATTLVL_ADC_CLOCK_FUN			RCC_APB2PeriphClockCmd
#define BATTLVL_ADC_CHANNEL				ADC_Channel_10
#define BATTLVL_IRQ_FLAG				ADC_FLAG_AWD
#define BATTLVL_THRESHOLD_VOLTAGE		6.8f
#define BATTLVL_CONV_2_VOLTAGE			3.4677e-3f


#define POT_GPIO						GPIOA
#define POT_GPIO_CLOCK					RCC_AHB1Periph_GPIOA
#define POT_GPIO_PIN					GPIO_Pin_3
#define POT_ADC							ADC2
#define POT_ADC_CLOCK					RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2
#define POT_ADC_CLOCK_FUN				RCC_APB2PeriphClockCmd
#define POT_ADC_CHANNEL					ADC_Channel_3
#define POT_IRQ_FLAG					ADC_FLAG_EOC


#define HELPER_TIM						TIM8
#define HELPER_TIM_CLOCK				RCC_APB2Periph_TIM8
#define HELPER_TIM_CLOCK_FUN			RCC_APB2PeriphClockCmd
#define HELPER_TIM_BASE_FREQ			20000
#define HELPER_TIM_FREQ					10
#define HELPER_TIM_PRESCALER			((uint16_t)((SystemCoreClock) / HELPER_TIM_BASE_FREQ) - 1)
#define HELPER_TIM_PERIOD				(HELPER_TIM_BASE_FREQ / HELPER_TIM_FREQ - 1)


#define WIFI_GPIO_USART					GPIOA
#define WIFI_GPIO_SIG					GPIOC
#define WIFI_GPIO_CLOCK					RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC
#define WIFI_GPIO_SIG_RESET_PIN			GPIO_Pin_6
#define WIFI_GPIO_SIG_LMTFRES_PIN		GPIO_Pin_7
#define WIFI_GPIO_SIG_CMDDATA_PIN		GPIO_Pin_8
#define WIFI_GPIO_SIG_ALARM_PIN			GPIO_Pin_9
#define WIFI_GPIO_USART_TX_PIN			GPIO_Pin_9   	// WiFi_Rx
#define WIFI_GPIO_USART_RX_PIN			GPIO_Pin_10  	// WiFi_Tx
#define WIFI_GPIO_USART_CTS_PIN			GPIO_Pin_11		// WiFi_RTS
#define WIFI_GPIO_USART_RTS_PIN			GPIO_Pin_12		// WiFi_CTS
#define WIFI_GPIO_USART_TX_PINSOURCE	GPIO_PinSource9
#define WIFI_GPIO_USART_RX_PINSOURCE	GPIO_PinSource10
#define WIFI_GPIO_USART_CTS_PINSOURCE	GPIO_PinSource11
#define WIFI_GPIO_USART_RTS_PINSOURCE	GPIO_PinSource12
#define WIFI_GPIO_USART_AF				GPIO_AF_USART1
#define WIFI_USART						USART1
#define WIFI_USART_CLOCK				RCC_APB2Periph_USART1
#define WIFI_USART_CLOCK_FUN			RCC_APB2PeriphClockCmd
#define WIFI_USART_SPEED				115200
#define WIFI_NVIC_IRQn					USART1_IRQn
#define WIFI_USART_IRQHANDLER			USART1_IRQHandler
#define WIFI_DMA						DMA2
#define WIFI_DMA_CLOCK					RCC_AHB1Periph_DMA2
#define WIFI_TX_DMA_CHANNEL				DMA_Channel_4
#define WIFI_TX_DMA_STREAM				DMA2_Stream7
#define WIFI_TX_DMA_FLAG_TCIF			DMA_FLAG_TCIF7
#define WIFI_TX_DMA_NVIC_IRQn			DMA2_Stream7_IRQn
#define WIFI_TX_DMA_IRQHANDLER			DMA2_Stream7_IRQHandler
#define WIFI_RX_DMA_CHANNEL				DMA_Channel_4
#define WIFI_RX_DMA_STREAM				DMA2_Stream5
#define WIFI_RX_DMA_FLAG_TCIF			DMA_FLAG_TCIF5
#define WIFI_RX_DMA_NVIC_IRQn			DMA2_Stream5_IRQn
#define WIFI_RX_DMA_IRQHANDLER			DMA2_Stream5_IRQHandler

#define CPUUSAGE_BASE_TIM				TIM14
#define CPUUSAGE_CNT_TIM				TIM13
#define CPUUSAGE_TIM_CLOCKS				RCC_APB1Periph_TIM13 | RCC_APB1Periph_TIM14
#define CPUUSAGE_TIM_CLOCK_FUN			RCC_APB1PeriphClockCmd
#define CPUUSAGE_TIM_PRESCALER			((SystemCoreClock / 2) / 60000 - 1)
#define CPUUSAGE_TIM_PERIOD				(60000 - 1)
#define CPUUSAGE_BASE_TIM_IRQn			TIM8_TRG_COM_TIM14_IRQn
#define CPUUSAGE_BASE_TIM_IRQHANDLER	TIM8_TRG_COM_TIM14_IRQHandler

#endif /* __HARDWARE_H__ */
