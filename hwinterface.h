#ifndef _HWINTERFACE_H_
#define _HWINTERFACE_H_

#include <stm32f4xx.h>

typedef enum {LOW = 0, HIGH = !LOW} BitVal;
typedef enum {OFF = 0, ON = !OFF} OnOff;
typedef enum {WiFiMode_Command = 0, WiFiMode_Data = 1} WiFiMode;

/**
 * Interface configuration
 * bit 0   - WiFi
 * bit 1   - USB
 * bit 2   - Active flag modifier
 */
typedef enum {
	Interface_None = 0,
	Interface_WiFi = 1,
	Interface_USB = 2,
	Interface_All = 3,
	Interface_WiFi_Active = 5,
	Interface_USB_Active = 6,
	Interface_All_Active = 7
} Interface_Type;

#define IS_BITVAL(x) (x == LOW || x == HIGH)
#define IS_ONOFF(x) (x == ON || x == OFF)
#define IS_WIFIMODE(x) (x == WiFiMode_Command || x == WiFiMode_Data)

/*
 * This file provides basic hardware interface functions
 * such as lighting a led or setting PWM duty cycle of a motor
 */

/*
 * @brief Set specified led to specified state
 * @param no: LED number, from 1 to 6
 * @param state: ON or OFF
 * @retval None
 */
void lightLED(uint8_t no, OnOff state);

/*
 * @brief Sets servo position
 * @param prc: Position in percent 0 - 1.0 inclusive of the full allowable motion space
 * @retval None
 */
void setServoPos(float prc);

/*
 * @brief Sets servo position with respect to changed down position
 * @param None
 * @retval None
 */
void updateServoPos(void);

/*
 * @brief Sets a down position value as a reference to other servo functions
 * @param val New down position in percent of allowable motion space
 * @retval None
 */
void setServoDownPosValue(float val);

/*
 * @brief Sets servo down so that the pen touches the ground
 * @param None
 * @retval None
 */
void setPenDown(void);

/*
 * @brief Sets servo up so that the pen do not touches the ground
 * @param None
 * @retval None
 */
void setPenUp(void);

/*
 * @brief Sets four signals that control H-bridge of each motor
 * @param ENA EN_A signal
 * @param ENB EN_B signal
 * @param INA IN_A signal
 * @param INB IN_B signal
 * @retval None
 */
void setMotorLSignals(BitVal ENA, BitVal ENB, BitVal INA, BitVal INB);
void setMotorRSignals(BitVal ENA, BitVal ENB, BitVal INA, BitVal INB);

/*
 * @brief Enable or disable both motors
 * @param enable ENABLE or DISABLE
 * @retval None
 */
void enableMotors(FunctionalState state);

/*
 * @brief Sets IN_A and IN_B signals to set motor direction forward
 * @param None
 * @retval None
 */
void setMotorLFwd(void);
void setMotorRFwd(void);

/*
 * @brief Sets IN_A and IN_B signals to set motor direction to reverse
 */
void setMotorLRev(void);
void setMotorRRev(void);

/* Brakes the motor */
void setMotorLBrake(void);
void setMotorRBrake(void);

/*
 * @brief Sets motor speed to a value being a percent of max speed
 * @param prc -1.0 to 1.0 inclusive, percent of max speed to set
 * @retval None
 */
void setMotorLSpeed(float prc);
void setMotorRSpeed(float prc);

/*
 * @brief Returns encoders position regarding to start position
 * @param None
 * @retval Encoder position
 */
int32_t getEncoderL(void);
int32_t getEncoderR(void);

/* @brief Returns battery voltage
 * @param None
 * @retval Battery voltage in volts
 */
float getBatteryVoltage(void);
float getAvgBatteryVoltage(void);

void enableLantern(FunctionalState state);
FunctionalState getLanternState();

void enableWiFi(FunctionalState state);
void enableUSB(FunctionalState state);

OnOff getWiFiStatus(void);
OnOff getUSBStatus(void);

void enableWiFi2USBBridge(FunctionalState state);

OnOff getWiFi2USBBridgeStatus(void);

OnOff getSwitchStatus(uint8_t no);

void setWiFiReset(FunctionalState state);
FunctionalState getWiFiReset();

void setWiFiAlarm(FunctionalState state);

void setWiFiMode(WiFiMode mode);
WiFiMode getWiFiMode();

void setWiFiFactoryDefault(FunctionalState state);

void sendInterfaceBlocking(uint8_t byte, Interface_Type interface);

#endif
