#include <stm32f4xx.h>
#include <stdbool.h>
#include "hardware.h"
#include "hwinterface.h"
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"

#define SERVO_MAX_POS 230 			/* Raw value to CC register */
#define SERVO_MIN_POS 50  			/* Raw value to CC register */
#define SERVO_UP_TO_DOWN 0.08f 		/* Percent of allowable motion space - 0.1 means 10% of (SERVO_MAX_POS - SERVO_MIN_POS) */

static volatile float SERVO_DOWN_POS = 0.5f;
static volatile float SERVO_LAST_POS = 0.5f;

static volatile OnOff WIFI_STATUS = ON;
static volatile OnOff USB_STATUS = ON;
static volatile OnOff WIFI2USBBRIDGE_STATUS = OFF;
static volatile FunctionalState LANTER_STATE = ENABLE;

void lightLED(uint8_t no, OnOff state) {
	/* GPIOs may vary */
	GPIO_TypeDef * gpio = (no == 1 ? LEDS_GPIO_1 : LEDS_GPIO_26);
	uint16_t pin;
	switch (no) {
	case 1:
		pin = LEDS_GPIO_1_PIN;
		break;
	case 2:
		pin = LEDS_GPIO_2_PIN;
		break;
	case 3:
		pin = LEDS_GPIO_3_PIN;
		break;
	case 4:
		pin = LEDS_GPIO_4_PIN;
		break;
	case 5:
		pin = LEDS_GPIO_5_PIN;
		break;
	case 6:
		pin = LEDS_GPIO_6_PIN;
		break;
	default:
		return;
	}

	/* Change bits in critical section to ensure registers coherence */
	taskENTER_CRITICAL();
	if (state == ON) {
		GPIO_SetBits(gpio, pin);
	}
	else {
		GPIO_ResetBits(gpio, pin);
	}
	taskEXIT_CRITICAL();
}

void setServoPos(float prc) {
	if (prc < 0.0f) prc = 0.0f;
	else if (prc > 1.0f) prc = 1.0f;
	TIM_SetCompare2(SERVO_TIM, roundf(prc * (SERVO_MAX_POS-SERVO_MIN_POS) + SERVO_MIN_POS));
	SERVO_LAST_POS = prc;
}

void setServoDownPosValue(float val) {
	assert_param(val >= 0.0f && val <= 1.0f);
	/* This function is not re-entrant because it updates global variables; need to provide mutual exclusion */
	taskENTER_CRITICAL();
	setServoPos(SERVO_LAST_POS + val - SERVO_DOWN_POS);
	SERVO_DOWN_POS = val;
	taskEXIT_CRITICAL();
}

void setPenDown(void) {
	setServoPos(SERVO_DOWN_POS);
}

void setPenUp(void) {
	setServoPos(SERVO_DOWN_POS + SERVO_UP_TO_DOWN);
}

void enablePen(FunctionalState newstate) {
	TIM_Cmd(SERVO_TIM, newstate);
}

OnOff getPenState() {
	return (uint16_t)(SERVO_TIM->CR1 & TIM_CR1_CEN) ? ON : OFF;
}

void setMotorLSignals(BitVal ENA, BitVal ENB, BitVal INA, BitVal INB) {
	taskENTER_CRITICAL();
	if (ENA == HIGH) GPIO_SetBits(MOTORL_GPIO, MOTORL_GPIO_ENA_PIN);
	else GPIO_ResetBits(MOTORL_GPIO, MOTORL_GPIO_ENA_PIN);
	if (ENB == HIGH) GPIO_SetBits(MOTORL_GPIO, MOTORL_GPIO_ENB_PIN);
	else GPIO_ResetBits(MOTORL_GPIO, MOTORL_GPIO_ENB_PIN);
	if (INA == HIGH) GPIO_SetBits(MOTORL_GPIO, MOTORL_GPIO_INA_PIN);
	else GPIO_ResetBits(MOTORL_GPIO, MOTORL_GPIO_INA_PIN);
	if (INB == HIGH) GPIO_SetBits(MOTORL_GPIO, MOTORL_GPIO_INB_PIN);
	else GPIO_ResetBits(MOTORL_GPIO, MOTORL_GPIO_INB_PIN);
	taskEXIT_CRITICAL();
}

void setMotorRSignals(BitVal ENA, BitVal ENB, BitVal INA, BitVal INB) {
	taskENTER_CRITICAL();
	if (ENA == HIGH) GPIO_SetBits(MOTORR_GPIO, MOTORR_GPIO_ENA_PIN);
	else GPIO_ResetBits(MOTORR_GPIO, MOTORR_GPIO_ENA_PIN);
	if (ENB == HIGH) GPIO_SetBits(MOTORR_GPIO, MOTORR_GPIO_ENB_PIN);
	else GPIO_ResetBits(MOTORR_GPIO, MOTORR_GPIO_ENB_PIN);
	if (INA == HIGH) GPIO_SetBits(MOTORR_GPIO, MOTORR_GPIO_INA_PIN);
	else GPIO_ResetBits(MOTORR_GPIO, MOTORR_GPIO_INA_PIN);
	if (INB == HIGH) GPIO_SetBits(MOTORR_GPIO, MOTORR_GPIO_INB_PIN);
	else GPIO_ResetBits(MOTORR_GPIO, MOTORR_GPIO_INB_PIN);
	taskEXIT_CRITICAL();
}

void enableMotors(FunctionalState state) {
	assert_param(IS_FUNCTIONAL_STATE(state));

	/* This function is re-entrant, but need to make sure that both motors' have correct signals set together */
	taskENTER_CRITICAL();
	if (state == DISABLE) {
		setMotorLSignals(LOW, LOW, LOW, LOW);
		setMotorRSignals(LOW, LOW, LOW, LOW);
	}
	else {
		setMotorLSignals(HIGH, HIGH, LOW, LOW);
		setMotorRSignals(HIGH, HIGH, LOW, LOW);
	}
	taskEXIT_CRITICAL();
}

void setMotorLFwd(void) {
	taskENTER_CRITICAL();
	GPIO_ResetBits(MOTORL_GPIO, MOTORL_GPIO_INA_PIN);
	GPIO_SetBits(MOTORL_GPIO, MOTORL_GPIO_INB_PIN);
	taskEXIT_CRITICAL();
}

void setMotorRFwd(void) {
	taskENTER_CRITICAL();
	GPIO_SetBits(MOTORR_GPIO, MOTORR_GPIO_INA_PIN);
	GPIO_ResetBits(MOTORR_GPIO, MOTORR_GPIO_INB_PIN);
	taskEXIT_CRITICAL();
}

void setMotorLRev(void) {
	taskENTER_CRITICAL();
	GPIO_SetBits(MOTORL_GPIO, MOTORL_GPIO_INA_PIN);
	GPIO_ResetBits(MOTORL_GPIO, MOTORL_GPIO_INB_PIN);
	taskEXIT_CRITICAL();
}

void setMotorRRev(void) {
	taskENTER_CRITICAL();
	GPIO_ResetBits(MOTORR_GPIO, MOTORR_GPIO_INA_PIN);
	GPIO_SetBits(MOTORR_GPIO, MOTORR_GPIO_INB_PIN);
	taskEXIT_CRITICAL();
}

void setMotorLBrake(void) {
	taskENTER_CRITICAL();
	GPIO_ResetBits(MOTORL_GPIO, MOTORL_GPIO_INA_PIN);
	GPIO_ResetBits(MOTORL_GPIO, MOTORL_GPIO_INB_PIN);
	taskEXIT_CRITICAL();
}

void setMotorRBrake(void) {
	taskENTER_CRITICAL();
	GPIO_ResetBits(MOTORR_GPIO, MOTORR_GPIO_INA_PIN);
	GPIO_ResetBits(MOTORR_GPIO, MOTORR_GPIO_INB_PIN);
	taskEXIT_CRITICAL();
}

void setMotorLSpeed(float prc) {
	if (prc < -1.0f) prc = -1.0f;
	else if (prc > 1.0f) prc = 1.0f;

	/* To ensure that PWM period matches direction there need to be critical section */
	taskENTER_CRITICAL();
	MOTOR_PWM_LEFT_SET_FUN(MOTOR_PWM_TIM, roundf(fabsf(prc) * MOTOR_PWM_TIM_PERIOD));
	if (prc > 0.0f)
		setMotorLFwd();
	else if (prc < 0.0f)
		setMotorLRev();
	taskEXIT_CRITICAL();
}

void setMotorRSpeed(float prc) {
	if (prc < -1.0f) prc = -1.0f;
	else if (prc > 1.0f) prc = 1.0f;

	/* To ensure that PWM period matches direction there need to be critical section */
	taskENTER_CRITICAL();
	MOTOR_PWM_RIGHT_SET_FUN(MOTOR_PWM_TIM, roundf((1.0f - fabsf(prc)) * MOTOR_PWM_TIM_PERIOD));
	if (prc > 0.0f)
		setMotorRFwd();
	else if (prc < 0.0f)
		setMotorRRev();
	taskEXIT_CRITICAL();
}

int32_t getEncoderL(void) {
	return (int32_t)((int64_t)TIM_GetCounter(ENCODERS_TIM_L) - (int64_t)0x80000000L);
}

int32_t getEncoderR(void) {
	return (int32_t)((int64_t)TIM_GetCounter(ENCODERS_TIM_R) - (int64_t)0x80000000L);
}

float getBatteryVoltage(void) {
	return (float)(ADC_GetConversionValue(BATTLVL_ADC) & 0x0FFF) * BATTLVL_CONV_2_VOLTAGE;
}

float getAvgBatteryVoltage(void) {
	return 8.0f;
	//TODO
}

void enableLantern(FunctionalState state) {
	assert_param(IS_FUNCTIONAL_STATE(state));
	GPIO_InitTypeDef GPIO_InitStructure = {
			.GPIO_OType = GPIO_OType_OD,
			.GPIO_Pin = LANTERN_GPIO_PIN1 | LANTERN_GPIO_PIN2 | LANTERN_GPIO_PIN3 | LANTERN_GPIO_PIN4,
			.GPIO_PuPd = GPIO_PuPd_NOPULL,
			.GPIO_Speed = GPIO_Speed_2MHz
	};

	/* To ensure that GPIOs settings matches TIM state there is critical section */
	taskENTER_CRITICAL();
	if (state == ENABLE) {
		TIM_Cmd(LANTERN_TIM, ENABLE);
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	}
	else {
		TIM_Cmd(LANTERN_TIM, DISABLE);
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_SetBits(LANTERN_GPIO, LANTERN_GPIO_PIN1 | LANTERN_GPIO_PIN2 | LANTERN_GPIO_PIN3 | LANTERN_GPIO_PIN4);
	}
	GPIO_Init(LANTERN_GPIO, &GPIO_InitStructure);
	LANTER_STATE = state;
	taskEXIT_CRITICAL();
}

FunctionalState getLanternState() {
	return LANTER_STATE;
}

void enableWiFi(FunctionalState state) {
	assert_param(IS_FUNCTIONAL_STATE(state));

	/* This function is not re-entrant */
	taskENTER_CRITICAL();
	WIFI_STATUS = (state == ENABLE ? ON : OFF);

	if (WIFI_STATUS == ON) {
		USART_ITConfig(WIFI_USART, USART_IT_RXNE, ENABLE);
	}
	else {
		USART_ITConfig(WIFI_USART, USART_IT_RXNE, DISABLE);
	}
	taskEXIT_CRITICAL();
}

void enableUSB(FunctionalState state) {
	assert_param(IS_FUNCTIONAL_STATE(state));

	/* This function is not re-entrant */
	taskENTER_CRITICAL();
	USB_STATUS = (state == ENABLE ? ON : OFF);

	if (USB_STATUS == ON) {
		USART_ITConfig(COM_USART, USART_IT_RXNE, ENABLE);
	}
	else {
		USART_ITConfig(COM_USART, USART_IT_RXNE, DISABLE);
	}
	taskEXIT_CRITICAL();
}

OnOff getWiFiStatus(void) {
	return WIFI_STATUS;
}

OnOff getUSBStatus(void) {
	return USB_STATUS;
}

void enableWiFi2USBBridge(FunctionalState state) {
	assert_param(IS_FUNCTIONAL_STATE(state));

	WIFI2USBBRIDGE_STATUS = (state == ENABLE ? ON : OFF);
}

OnOff getWiFi2USBBridgeStatus(void) {
	return WIFI2USBBRIDGE_STATUS == ON && WIFI_STATUS == ON && USB_STATUS == ON;
}

OnOff getSwitchStatus(uint8_t no) {
	assert_param(no >= 1 && no <= 6);
	switch (no) {
	case 1:
		if (GPIO_ReadInputDataBit(SWITCHES_GPIO, SWITCHES_GPIO_1_PIN) == Bit_SET) return OFF;
		else return ON;
	case 2:
		if (GPIO_ReadInputDataBit(SWITCHES_GPIO, SWITCHES_GPIO_2_PIN) == Bit_SET) return OFF;
		else return ON;
	case 3:
		if (GPIO_ReadInputDataBit(SWITCHES_GPIO, SWITCHES_GPIO_3_PIN) == Bit_SET) return OFF;
		else return ON;
	case 4:
		if (GPIO_ReadInputDataBit(SWITCHES_GPIO, SWITCHES_GPIO_4_PIN) == Bit_SET) return OFF;
		else return ON;
	case 5:
		if (GPIO_ReadInputDataBit(SWITCHES_GPIO, SWITCHES_GPIO_5_PIN) == Bit_SET) return OFF;
		else return ON;
	case 6:
		if (GPIO_ReadInputDataBit(SWITCHES_GPIO, SWITCHES_GPIO_6_PIN) == Bit_SET) return OFF;
		else return ON;
	default:
		return OFF;
	}
	return OFF;
}

void setWiFiReset(FunctionalState state) {
	assert_param(IS_FUNCTIONAL_STATE(state));
	taskENTER_CRITICAL();
	if (state == DISABLE) GPIO_SetBits(WIFI_GPIO_SIG, WIFI_GPIO_SIG_RESET_PIN);
	else GPIO_ResetBits(WIFI_GPIO_SIG, WIFI_GPIO_SIG_RESET_PIN);
	taskEXIT_CRITICAL();
}

FunctionalState getWiFiReset() {
	if (GPIO_ReadOutputDataBit(WIFI_GPIO_SIG, WIFI_GPIO_SIG_CMDDATA_PIN) == Bit_SET) return ENABLE;
	else return DISABLE;
}

void setWiFiAlarm(FunctionalState state) {
	assert_param(IS_FUNCTIONAL_STATE(state));
	taskENTER_CRITICAL();
	if (state == DISABLE) GPIO_SetBits(WIFI_GPIO_SIG, WIFI_GPIO_SIG_ALARM_PIN);
	else GPIO_ResetBits(WIFI_GPIO_SIG, WIFI_GPIO_SIG_ALARM_PIN);
	taskEXIT_CRITICAL();
}

void setWiFiMode(WiFiMode mode) {
	taskENTER_CRITICAL();
	if (mode == WiFiMode_Command) GPIO_SetBits(WIFI_GPIO_SIG, WIFI_GPIO_SIG_CMDDATA_PIN);
	else GPIO_ResetBits(WIFI_GPIO_SIG, WIFI_GPIO_SIG_CMDDATA_PIN);
	taskEXIT_CRITICAL();
}

WiFiMode getWiFiMode() {
	if (GPIO_ReadOutputDataBit(WIFI_GPIO_SIG, WIFI_GPIO_SIG_CMDDATA_PIN) == Bit_SET) return WiFiMode_Command;
	else return WiFiMode_Data;
}

void setWiFiFactoryDefault(FunctionalState state) {
	assert_param(IS_FUNCTIONAL_STATE(state));
	taskENTER_CRITICAL();
	if (state == DISABLE) GPIO_SetBits(WIFI_GPIO_SIG, WIFI_GPIO_SIG_LMTFRES_PIN);
	else GPIO_ResetBits(WIFI_GPIO_SIG, WIFI_GPIO_SIG_LMTFRES_PIN);
	taskEXIT_CRITICAL();
}

void sendInterfaceBlocking(uint8_t byte, Interface_Type interface) {
	bool throughWiFi = ((interface & Interface_WiFi) != 0);
	bool throughUSB = ((interface & Interface_USB) != 0);
	if (interface & 0x04) { // active flag mask
		throughWiFi = (throughWiFi && (getWiFiStatus() == ON));
		throughUSB = (throughUSB && (getUSBStatus() == ON));
	}

	if (throughWiFi)
		USART_SendData(WIFI_USART, byte);
	if (throughUSB)
		USART_SendData(COM_USART, byte);

	if (throughWiFi)
		while (USART_GetFlagStatus(WIFI_USART, USART_FLAG_TXE) == RESET);
	if (throughUSB)
		while (USART_GetFlagStatus(COM_USART, USART_FLAG_TXE) == RESET);
}

int printInterfaceBlocking(const char *str, int length, Interface_Type interface) {
	if (getWiFi2USBBridgeStatus() != ON) {
		int counter = length;
		for (; counter > 0; counter--) {
			if (*str == 0) break;
			sendInterfaceBlocking(*str, interface);
			str++;
		}
		return length;
	}
	else
		return 0;
}

void systemReset() {
	/* Set too low preload value causing reset to occur */
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetReload(1);
	while(1);
}
