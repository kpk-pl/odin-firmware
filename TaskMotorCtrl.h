#ifndef _TASKMOTORCTRL_H_
#define _TASKMOTORCTRL_H_

#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"

extern xTaskHandle motorCtrlTask;						/*!< Export this task's handle */
extern xSemaphoreHandle motorControllerMutex;			/*!< Export mutex for trajectory regulators */
extern xQueueHandle motorCtrlQueue;						/*!< Export queue's handle to which motor commands should be passed */
extern volatile uint32_t globalLogSpeedCounter;			/*!< Export counter useful when logging only a few speed entries */
extern volatile FunctionalState globalSpeedRegulatorOn;	/*!< Export on/off regulator setting flag */

#ifdef USE_CUSTOM_MOTOR_CONTROLLER
#include "motorController.h"
	extern volatile MotorControllerState_Struct globalLeftMotorParams;		/*!< Export left wheel's custom regulator params */
	extern volatile MotorControllerState_Struct globalRightMotorParams;		/*!< Export right wheel's custom regulator params */
#else
#include "arm_math.h"
	extern volatile float globalMotorPidKp;				/*!< Export Kp for the PID controller */
	extern volatile float globalMotorPidKi;				/*!< Export Ki for the PID controller */
	extern volatile float globalMotorPidKd;				/*!< Export Kd for the PID controller */
#endif

/**
 *  \brief Struct holding motors' speeds in rad/sec.
 */
typedef struct {
	float LeftSpeed;				/*<< Left motor's speed */
	float RightSpeed;				/*<< Right motor's speed */
} MotorSpeed_Struct;

/**
 * \brief Controller for motors
 */
void TaskMotorCtrl(void *);				// Motors' speed regulator

/**
 * \brief Sets all OS-related objects for use. It should be called on startup, only once, before scheduler starts
 */
void TaskMotorCtrlConstructor();

/**
 * \brief Send speed update to motor controller
 * @param left Left wheel speed in rad/s
 * @param right Right wheel speed in rad/s
 * @param delay Ticks to wait until speeds are sent
 */
void sendSpeeds(float left, float right, unsigned portLONG delay);

bool isCurrentlyDriving();

#endif /* _TASKMOTORCTRL_H_ */
