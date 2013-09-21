#ifndef _TASKMOTORCTRL_H_
#define _TASKMOTORCTRL_H_

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

extern xTaskHandle motorCtrlTask;						/*!< Export this task's handle */
extern xQueueHandle motorCtrlQueue;						/*!< Export queue's handle to which motor commands should be passed */
extern volatile uint32_t globalLogSpeedCounter;			/*!< Export counter useful when logging only a few speed entries */
extern volatile FunctionalState globalSpeedRegulatorOn;	/*!< Export on/off regulator setting flag */

#ifdef USE_CUSTOM_MOTOR_CONTROLLER
	extern volatile FunctionalState globalControllerVoltageCorrection;	/*!< Export setting if voltage correction is used in custom speed regulator */
#endif

#ifdef USE_CUSTOM_MOTOR_CONTROLLER
#include "motorController.h"
	extern MotorControllerParameters_Struct globalLeftMotorParams;		/*!< Export left wheel's custom regulator params */
	extern MotorControllerParameters_Struct globalRightMotorParams;		/*!< Export right wheel's custom regulator params */
#else
#include "arm_math.h"
	extern arm_pid_instance_f32 globalPidLeft;		/*!< Export left wheel's PID regulator params */
	extern arm_pid_instance_f32 globalPidRight;		/*!< Export right wheel's PID regulator params */
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

#endif /* _TASKMOTORCTRL_H_ */
