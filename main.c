#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>

#include "compilation.h"

#ifdef FOLLOW_TRAJECTORY
#include "pointsBuffer.h"
#endif

#include "TaskTelemetry.h"
#include "TaskRC5.h"
#include "TaskMotorCtrl.h"
#include "TaskPrintfConsumer.h"
#include "TaskCommandHandler.h"
#include "TaskLED.h"
#ifdef USE_IMU_TELEMETRY
#include "TaskIMU.h"
#include "TaskIMUMagScaling.h"
#endif

#include "main.h"
#include "priorities.h"
#include "hardware.h"
#include "hwinterface.h"


/* Useful defines for motor control */
#define BUF_RX_LEN 20											/*<< Maximum length of UART command */

#ifdef FOLLOW_TRAJECTORY
/* Typedef for gains used by trajectory controller */
typedef struct {
	float k_x;
	float k;
	float k_s;
} TrajectoryControlerGains_Struct;
#else
/* Type of drive command to perform */
typedef enum {
	DriveCommand_Type_Line =  'l',		/*<< Drive straight line; need one parameter - length */
	DriveCommand_Type_Point = 'p',		/*<< Drive to point; need two parameters - X and Y coordinates */
	DriveCommand_Type_Arc =   'a',		/*<< Drive around an arc; need two parameters - radius and arc length in degrees */
	DriveCommand_Type_Angle = 'd'		/*<< Turn by an angle; need two parameters - one indicating wheather it is relative (0) or absolute (1) angle and second being an angle in degrees */
} DriveCommand_Type;

/* Struct to hold driving information needed by trajectory controller */
typedef struct {
	DriveCommand_Type Type;			/*<< Command type, one of DriveCommand_Type */
	bool UsePen;					/*<< If true then pen will be held down, up otherwise */
	float Speed;					/*<< Robot's speed, only positive values */
	float Param1;					/*<< Command param #1 */
	float Param2;					/*<< Command param #2 */
} DriveCommand_Struct;
#endif

/* Types of input's character source */
typedef enum {
	PrintSource_Type_USB = 1,
	PrintSource_Type_WiFi = 2
} PrintSource_Type;

/* Struct to hold one input char with it's source */
typedef struct {
	char Input;
	PrintSource_Type Source;
} PrintInput_Struct;

/* Initialize all hardware. THIS FUNCTION DISABLES INTERRUPTS AND DO NOT ENABLES THEM AGAIN */
static void Initialize();

#ifdef FOLLOW_TRAJECTORY
/*
 * @brief Function for calculating motors speed using trajectory control
 * @param currentPosition Current position from telemetry
 * @param trajectoryPoint Target position and velocities
 * @param outputSpeeds Calculated speeds
 * @retval none
 */
void calculateTrajectoryControll(const TelemetryData_Struct * currentPosition,
								 TrajectoryPoint_Ptr trajectoryPoint,
								 MotorSpeed_Struct * outputSpeeds);
#endif

#ifndef FOLLOW_TRAJECTORY
void TaskDrive(void *);					// Task controlling trajectory. Issues wheel's speed commands, checks if target is reached, calculates best route
#else
void TaskTrajectory(void *);			// Task for controlling trajectory using Ferdek's regulator
#endif
void TaskUSBWiFiBridge(void *);			// Task for USB-WiFi bridge to transfer commands between two UARTs
void TaskInputBuffer(void *);			// Task to handle input characters from any source - it makes possible to set very high transfer speeds

xQueueHandle printfQueue;
xQueueHandle commandQueue;
#ifndef FOLLOW_TRAJECTORY
xQueueHandle driveQueue;				// Queue for storing driving commands (probably send in huge blocks like 100 commands at once
#endif
xQueueHandle motorCtrlQueue;
xQueueHandle WiFi2USBBufferQueue;		// Buffer for WiFi to USB characters
xQueueHandle USB2WiFiBufferQueue;		// Buffer for USB to WiFi characters
xQueueHandle commInputBufferQueue;		// Buffer for input characters
xQueueHandle telemetryQueue;
#ifdef USE_IMU_TELEMETRY
xQueueHandle I2CEVFlagQueue;
xQueueHandle magnetometerScalingQueue = NULL;
#endif

xTaskHandle printfConsumerTask;
xTaskHandle commandHandlerTask;
#ifdef USE_IMU_TELEMETRY
xTaskHandle imuMagScalingTask;
xTaskHandle imuTask;
#endif
#ifndef FOLLOW_TRAJECTORY
xTaskHandle driveTask;
#else
xTaskHandle trajectoryTask;
#endif
xTaskHandle motorCtrlTask;
xTaskHandle RC5Task;
xTaskHandle telemetryTask;
xTaskHandle USBWiFiBridgeTask;
xTaskHandle commInputBufferTask;

xSemaphoreHandle comUSARTTCSemaphore;			// USART_TC flag set for USB-USART
xSemaphoreHandle comDMATCSemaphore;				// DMA TC flag set for USB-USART
xSemaphoreHandle wifiUSARTTCSemaphore;			// USART TC flag set for WIFI-USART
xSemaphoreHandle wifiDMATCSemaphore;			// DMA TC flag set for WIFI-USART
xSemaphoreHandle rc5CommandReadySemaphore;		// used by RC5 API to inform about new finished transmission
#ifdef USE_IMU_TELEMETRY
xSemaphoreHandle imuPrintRequest;				// request for IMU to print most up-to-date reading
xSemaphoreHandle imuGyroReady;					// gyro ready flag set in interrupt
xSemaphoreHandle imuAccReady;					// accelerometer ready flag set in interrupt
xSemaphoreHandle imuMagReady;					// magnetometer ready flag set in interrupt
xSemaphoreHandle imuI2CEV;						// semaphore to indicate correct event in I2C protocol
xSemaphoreHandle imuMagScalingReq;				// request to perform magnetometer scaling
#endif

#ifdef USE_IMU_TELEMETRY
xTimerHandle imuWatchdogTimer;					// used by software watchdog, if it expires then I2C is reset
#endif

/*
 * Global variable that holds current up-to-date telemetry data.
 * Only telemetryTask should write to it.
 * To read from this one should use getTelemetry function than provides mutual exclusion to ensure data coherency
 */
volatile TelemetryData_Struct globalTelemetryData = {0.0f, 0.0f, 0.0f};

volatile FunctionalState globalLogTelemetry = DISABLE;
volatile FunctionalState globalLogSpeed = DISABLE;
volatile FunctionalState globalLogEvents = ENABLE;
volatile FunctionalState globalSpeedRegulatorOn = ENABLE;
#ifdef USE_CUSTOM_MOTOR_CONTROLLER
volatile FunctionalState globalControllerVoltageCorrection = DISABLE;
#endif
volatile uint32_t globalLogSpeedCounter = 0;
volatile float globalCPUUsage = 0.0f;
#ifdef USE_IMU_TELEMETRY
volatile bool globalIMUHang = false;
volatile bool globalDoneIMUScaling = false;
volatile FunctionalState globalMagnetometerScalingInProgress = DISABLE;
float globalMagnetometerImprovData[721];
arm_linear_interp_instance_f32 globalMagnetometerImprov = {		/*<< used by IMU to correctly scale magnetometer readings*/
	.nValues = 721,
	.xSpacing = M_PI / 360.0f,
	.pYData = globalMagnetometerImprovData
};
#endif

#ifdef USE_CUSTOM_MOTOR_CONTROLLER
	MotorControllerParameters_Struct globalLeftMotorParams = {
		.threshold = 1.7164f,
		.A  = 1.2885f,
		.B  = 0.9323f,
		.C  = 0.1427f,
		.KP = 0.05f, // TODO: value TBD experimentally
		.A_t  = 19.6755f,
		.B_t  = 14.2360f,
		.KP_t = 0.05f//TODO: value TBD experimentally
	};
	MotorControllerParameters_Struct globalRightMotorParams = {
		.threshold = 1.5510f,
		.A  = 1.3758f,
		.B  = 1.4037f,
		.C  = 0.1150f,
		.KP = 0.05f, // TODO: value TBD experimentally
		.A_t  = 17.2120f,
		.B_t  = 17.5611f,
		.KP_t = 0.05f //TODO: value TBD experimentally
	};
#else
	arm_pid_instance_f32 globalPidLeft = {
		.Kp = 0.08f,
		.Ki = 0.005f,
		.Kd = 0.0f
	};
	arm_pid_instance_f32 globalPidRight = {
		.Kp = 0.08f,
		.Ki = 0.005f,
		.Kd = 0.0f
	};
#endif
#ifdef FOLLOW_TRAJECTORY
	TrajectoryControlerGains_Struct globalTrajectoryControlGains = {
		.k_x = 10.0f,
		.k = 10.0f,
		.k_s = 10.0f
	};
#endif

int main(void)
{
	/* Ensure all priority bits are assigned as preemption priority bits. */
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	SystemInit();
	SystemCoreClockUpdate();
	//RCC_HSEConfig(RCC_HSE_ON);

	Initialize();
	if (globalLogEvents) printf("Reset!\n");

	printfQueue 		 = xQueueCreate(50, 	sizeof(char*)					);
	commandQueue 		 = xQueueCreate(15, 	sizeof(char*)					);
#ifndef FOLLOW_TRAJECTORY
	driveQueue 			 = xQueueCreate(100, 	sizeof(DriveCommand_Struct*)	);		// holding pointers because there's a lot of big structures that are processed rather slowly. Memory is allocated dynamically
#endif
	motorCtrlQueue 		 = xQueueCreate(1,		sizeof(MotorSpeed_Struct)		);
	telemetryQueue 		 = xQueueCreate(30, 	sizeof(TelemetryUpdate_Struct)	);
#ifdef USE_IMU_TELEMETRY
	I2CEVFlagQueue		 = xQueueCreate(1,		sizeof(uint32_t)				);
#endif
	commInputBufferQueue = xQueueCreate(100,	sizeof(PrintInput_Struct)		);

	vSemaphoreCreateBinary(	comUSARTTCSemaphore			);
	vSemaphoreCreateBinary(	comDMATCSemaphore			);
	vSemaphoreCreateBinary(	wifiUSARTTCSemaphore		);
	vSemaphoreCreateBinary(	wifiDMATCSemaphore			);
#ifdef USE_IMU_TELEMETRY
	vSemaphoreCreateBinary(	imuPrintRequest				);
	vSemaphoreCreateBinary(	imuGyroReady				);
	vSemaphoreCreateBinary(	imuAccReady					);
	vSemaphoreCreateBinary(	imuMagReady					);
	vSemaphoreCreateBinary( imuI2CEV					);
	vSemaphoreCreateBinary( imuMagScalingReq			);
#endif
	vSemaphoreCreateBinary(	rc5CommandReadySemaphore	);

#ifdef USE_IMU_TELEMETRY
	imuWatchdogTimer = xTimerCreate(NULL, 500/portTICK_RATE_MS, pdFALSE, NULL, imuWatchdogOverrun);
#endif

	xTaskCreate(TaskCommandHandler, NULL, 	300, 						NULL, 		PRIOTITY_TASK_COMMANDHANDLER, 	&commandHandlerTask		);
#ifdef USE_IMU_TELEMETRY
	xTaskCreate(TaskIMU, 			NULL, 	400, 						NULL, 		PRIORITY_TASK_IMU, 				&imuTask				);
	xTaskCreate(TaskIMUMagScaling, 	NULL,	300,						NULL,		PRIORITY_TASK_IMUMAGSCALING,	&imuMagScalingTask		);
#endif
	xTaskCreate(TaskRC5, 			NULL, 	300, 						NULL, 		PRIORITY_TASK_RC5, 				&RC5Task				);
	xTaskCreate(TaskPrintfConsumer, NULL, 	500, 						NULL, 		PRIORITY_TASK_PRINTFCONSUMER, 	&printfConsumerTask		);
	xTaskCreate(TaskLED, 			NULL, 	configMINIMAL_STACK_SIZE, 	NULL, 		PRIORITY_TASK_LED,				NULL					);
#ifndef FOLLOW_TRAJECTORY
	xTaskCreate(TaskDrive, 			NULL, 	1000, 						NULL, 		PRIORITY_TASK_DRIVE,			&driveTask				);
#else
	xTaskCreate(TaskTrajectory,		NULL,	1000,						NULL,		PRIORITY_TASK_TRAJECTORY,		&trajectoryTask			);
#endif
	xTaskCreate(TaskMotorCtrl, 		NULL, 	300, 						NULL, 		PRIORITY_TASK_MOTORCTRL,		&motorCtrlTask			);
	xTaskCreate(TaskTelemetry, 		NULL, 	300, 						NULL,		PRIORITY_TASK_TELEMETRY,		&telemetryTask			);
	xTaskCreate(TaskUSBWiFiBridge, 	NULL,	300,						NULL,		PRIOTITY_TASK_BRIDGE,			&USBWiFiBridgeTask		);
	xTaskCreate(TaskInputBuffer,	NULL,	500, 						NULL, 		PRIOTITY_TASK_INPUTBUFFER,		&commInputBufferTask	);

	vTaskStartScheduler();
    while(1);
}

void TaskInputBuffer(void * p) {
	PrintInput_Struct newInput;

	bool incoming[2] = {false, false};
	char RXBUF[2][BUF_RX_LEN+1];
	uint8_t RXBUFPOS[2] = {0, 0};
	uint8_t i;

	while(1) {
		/* Block until new command is available */
		xQueueReceive(commInputBufferQueue, &newInput, portMAX_DELAY);

		/* Check source */
		switch (newInput.Source) {
		case PrintSource_Type_USB:
			i = 0;
			break;
		case PrintSource_Type_WiFi:
			i = 1;
			break;
		default: break;
		}

		switch (newInput.Input) {
		case '<':	// msg begin character
			RXBUFPOS[i] = 0;
			incoming[i] = true;
			break;
		case '>':	// msg end character
			if (incoming[i]) {
				RXBUF[i][RXBUFPOS[i]++] = '\0';
				incoming[i] = false;

				char * ptr = (char *)pvPortMalloc(RXBUFPOS[i]*sizeof(char));
				strcpy(ptr, (const char*)RXBUF[i]);

				RXBUFPOS[i] = 0;

				portBASE_TYPE contextSwitch = pdFALSE;
				if (xQueueSendToBackFromISR(commandQueue, &ptr, &contextSwitch) == errQUEUE_FULL) {
					vPortFree(ptr);
				}
				portEND_SWITCHING_ISR(contextSwitch);
			}
			break;
		default:
			if (incoming[i]) {
				if (RXBUFPOS[i] < BUF_RX_LEN)
					RXBUF[i][RXBUFPOS[i]++] = newInput.Input;
			}
			break;
		}
	}
}

void getTelemetry(TelemetryData_Struct *data) {
	/* Ensure that data is coherent and nothing is changed in between */
	taskENTER_CRITICAL();
	{
		data->X = globalTelemetryData.X;
		data->Y = globalTelemetryData.Y;
		data->O = normalizeOrientation(globalTelemetryData.O);
	}
	taskEXIT_CRITICAL();
}

void getTelemetryRaw(TelemetryData_Struct *data) {
	/* Ensure that data is coherent and nothing is changed in between */
	taskENTER_CRITICAL();
	{
		data->X = globalTelemetryData.X;
		data->Y = globalTelemetryData.Y;
		data->O = globalTelemetryData.O;
	}
	taskEXIT_CRITICAL();
}

#ifndef FOLLOW_TRAJECTORY
void TaskDrive(void * p) {
	portTickType wakeTime = xTaskGetTickCount();
	DriveCommand_Struct * command;
	TelemetryData_Struct telemetryData, begTelData;
	MotorSpeed_Struct motorsSpeed;

	/* Base time period for motors regulators */
	const uint16_t baseDelayMs = 10;

	while(1) {
		/* Take one command from queue */
		xQueueReceive(driveQueue, &command, portMAX_DELAY);

		/* Check if speed is not less than zero */
		if (command->Speed >= 0.0f) {
			/* Handle pen */
			if (command->UsePen) setPenDown();
			else setPenUp();

			/* Recalculate speed from m/s to rad/s */
			command->Speed = command->Speed * 1000.0f / RAD_TO_MM_TRAVELED;

			if (command->Type == DriveCommand_Type_Line) {
				if (globalLogEvents) safePrint(20, "Driving %.0fmm\n", command->Param1);

				const float breakingDistance = 100.0f;
				const float dist = fabsf(command->Param1);
				const float maxSpeed = copysignf(1.0f, (command->Param1)) * command->Speed;

				/* Get starting point telemetry data */
				getTelemetry(&begTelData);

				/* Start driving at max speed if far away from target */
				if (dist > breakingDistance) {
					/* Set motors speed allowing negative speed */
					motorsSpeed.RightSpeed = motorsSpeed.LeftSpeed = maxSpeed;
					xQueueSendToBack(motorCtrlQueue, &motorsSpeed, portMAX_DELAY);

					/* Wait in periods until position is too close to target position */
					while(1) {
						getTelemetry(&telemetryData);
						if (hypotf(telemetryData.X - begTelData.X, telemetryData.Y - begTelData.Y) >= dist - breakingDistance)
							break;
						vTaskDelayUntil(&wakeTime, 3*baseDelayMs/portTICK_RATE_MS);
					}
				}

				/* Regulate speed to gently approach target with desired accuracy */
				while(1) {
					/* Read current position */
					getTelemetry(&telemetryData);

					/* Calculate remaining distance */
					float rem = dist - hypotf(telemetryData.X - begTelData.X, telemetryData.Y - begTelData.Y);

					/* Finish if distance is very very small */
					if (rem < 0.5f) break;

					/* Set motors speed constantly as robot approaches target distance */
					motorsSpeed.RightSpeed = motorsSpeed.LeftSpeed = maxSpeed * (0.8f * rem / breakingDistance + 0.2f);
					xQueueSendToBack(motorCtrlQueue, &motorsSpeed, portMAX_DELAY);

					/* Wait for a bit */
					vTaskDelayUntil(&wakeTime, baseDelayMs/portTICK_RATE_MS);
				}
			}
			else if (command->Type == DriveCommand_Type_Angle || command->Type == DriveCommand_Type_Arc) {
				if (globalLogEvents) {
					if (command->Type == DriveCommand_Type_Angle) {
						safePrint(30, "Turning by %.2f %s\n", command->Param2, (command->Param1 < 0.5f ? "relative" : "absolute"));
					}
					else {
						safePrint(58, "Turning with radius %.2fmm and %.1f degrees length\n", command->Param1, command->Param2);
					}
				}

				const float breakingAngle = 45.0f * DEGREES_TO_RAD;

				/* Get starting point telemetry data */
				getTelemetry(&begTelData);

				/* Calculate target orientation */
				float targetO = command->Param2 * DEGREES_TO_RAD;
				if (command->Param1 < 0.5f || command->Type == DriveCommand_Type_Arc)
					targetO += begTelData.O;
				targetO = normalizeOrientation(targetO);

				/* Calculate direction; dir == 1 - turning left */
				int8_t dir = ((targetO > begTelData.O && fabsf(targetO - begTelData.O) < M_PI) || (targetO < begTelData.O && fabsf(begTelData.O - targetO) > M_PI) ? 1 : -1);

				/* Compute maximal speeds for both wheels */
				const float maxLeft = command->Speed * (command->Type == DriveCommand_Type_Angle ?
						(float)dir * -1.0f :
						1.0f - (float)dir*ROBOT_DIAM/(2.0f*command->Param1) );
				const float maxRight = command->Speed * (command->Type == DriveCommand_Type_Angle ?
						(float)dir :
						1.0f + (float)dir*ROBOT_DIAM/(2.0f*command->Param1) );

				/* Turn with maximum speed as long as turning angle is big */
				if (fabsf(normalizeOrientation(targetO - begTelData.O)) > breakingAngle) {
					/* Set maximum speed */
					motorsSpeed.LeftSpeed = maxLeft;
					motorsSpeed.RightSpeed = maxRight;
					/* Order speeds */
					xQueueSendToBack(motorCtrlQueue, &motorsSpeed, portMAX_DELAY);

					/* Wait for reaching close proximity of target angle */
					while(1) {
						getTelemetry(&telemetryData);
						if (fabsf(normalizeOrientation(targetO - telemetryData.O)) < breakingAngle) break;
						vTaskDelayUntil(&wakeTime, 3*baseDelayMs/portTICK_RATE_MS);
					}
				}

				/* Regulate speed to allow gentle target angle approaching with desired accuracy */
				while(1) {
					/* Read current orientation */
					getTelemetry(&telemetryData);
					/* Compute angle distance to target */
					float dist = fabsf(normalizeOrientation(telemetryData.O - targetO));

					/* End if distance is very small or direction changes */
					if (dist < 0.25f * DEGREES_TO_RAD ||
							((targetO > telemetryData.O && targetO - telemetryData.O < M_PI) ||
							 (targetO < telemetryData.O && telemetryData.O - targetO > M_PI) ? 1 : -1) != dir)
						break;

					/* Calculate speeds */
					motorsSpeed.LeftSpeed = maxLeft * (0.9f * dist / breakingAngle + 0.1f);
					motorsSpeed.RightSpeed = maxRight * (0.9f * dist / breakingAngle + 0.1f);

					/* Set new speeds */
					xQueueSendToBack(motorCtrlQueue, &motorsSpeed, portMAX_DELAY);

					/* Wait a little */
					vTaskDelayUntil(&wakeTime, baseDelayMs/portTICK_RATE_MS);
				}
				xQueueSendToBack(motorCtrlQueue, &motorsSpeed, portMAX_DELAY);
			}
			else if (command->Type == DriveCommand_Type_Point) {
				if (globalLogEvents) safePrint(44, "Driving to point X:%.1fmm Y:%.1fmm\n", command->Param1, command->Param2);

				/* First of all, turn to target point with desired accuracy */
				/* Get starting point telemetry data */
				getTelemetry(&begTelData);

				/* Calculate target orientation */
				float targetO = normalizeOrientation(atan2f(command->Param2 - begTelData.Y, command->Param1 - begTelData.X));
				/* Calculate direction; dir == 1 - turning left */
				int8_t dir = ((targetO > begTelData.O && fabsf(targetO - begTelData.O) < M_PI) || (targetO < begTelData.O && fabsf(begTelData.O - targetO) > M_PI) ? 1 : -1);

				/* Start turning if targetO is bigger than threshold */
				if (fabsf(targetO) > 20.0f * DEGREES_TO_RAD) {
					/* Compute speeds */
					motorsSpeed.LeftSpeed = (float)dir * (-1.0f) * command->Speed;
					motorsSpeed.RightSpeed = -motorsSpeed.LeftSpeed;
					/* Order speeds */
					xQueueSendToBack(motorCtrlQueue, &motorsSpeed, portMAX_DELAY);

					/* Wait for angle distance to become small enough */
					while(1) {
						/* Read current telemetry data */
						getTelemetry(&telemetryData);

						/* End turning if close enough to target angle */
						if (fabsf(normalizeOrientation(atan2f(command->Param2 - telemetryData.Y, command->Param1 - telemetryData.X) - telemetryData.O)) < 20.0f * DEGREES_TO_RAD)
							break;

						/* Wait for a while */
						vTaskDelayUntil(&wakeTime, 3*baseDelayMs/portTICK_RATE_MS);
					}
				}

				/* Start regulator - driving to point requires constant speeds updates */
				while(1) {
					/* Read current position */
					getTelemetry(&telemetryData);

					/* Finish up if target is really close or robot's missing the target */
					float d = hypotf(telemetryData.X - command->Param1, telemetryData.Y - command->Param2);
					if (d < 1.0f || fabsf(normalizeOrientation(atan2f(command->Param2 - telemetryData.Y, command->Param1 - telemetryData.X) - telemetryData.O)) > 60.0f * DEGREES_TO_RAD)
						break;

					/*
					 * Compute regulator values based on Papers-RAS_Blazic_2011.pdf ignoring position regulation
					 * Applying slight modifications to 'v' and 'w', saturating
					 */
					const float k = 0.0003f;						/*<< Gain for turning */
					float dx = command->Param1 - telemetryData.X;	/*<< Distance to target perpendicular to global coordinate X */
					float dy = command->Param2 - telemetryData.Y;	/*<< Distance to target parallel to global coordinate Y */
					float cosfi = cosf(telemetryData.O);			/*<< Cos of robot orientation angle */
					float sinfi = sinf(telemetryData.O);			/*<< Sin of robot orientation angle */
					//float ex = cosfi * dx + sinfi * dy;				/*<< Distance to target perpendicular to robot wheel axis */
					float ey = -sinfi * dx + cosfi * dy;			/*<< Distance to target parallel to robot wheel axis */
					float v = command->Speed;						/*<< Linear speed - always maximum */
					float w = k * command->Speed * ey;				/*<< Rotational speed - depending on ey; the bigger turn to make the bigger the speed is */

					/* Begin to slow down if closer than 5cm from target */
					if (d < 50.0f) {
						v = v * (0.7f * d / 50.0f + 0.3f);
					}

					/* Limit maximal rotational speed to half of maximum speed */
					if (fabsf(w) > command->Speed / 2.0f) {
						w = copysignf(command->Speed / 2.0f, w);
					}

					/* Set wheels speeds */
					motorsSpeed.LeftSpeed =  v - w * ROBOT_DIAM / 2.0f;
					motorsSpeed.RightSpeed = v + w * ROBOT_DIAM / 2.0f;
					xQueueSendToBack(motorCtrlQueue, &motorsSpeed, portMAX_DELAY);

					/* Wait a moment */
					vTaskDelayUntil(&wakeTime, baseDelayMs/portTICK_RATE_MS);
				}
			}
		}
		else { /* command->Speed < 0.0f */
			if (globalLogEvents) safePrint(34, "Speed cannot be less than zero!\n");
		}

		/* Stop motors if no other command available */
		if (uxQueueMessagesWaiting(driveQueue) == 0) {
			motorsSpeed.LeftSpeed = motorsSpeed.RightSpeed = 0.0f;
			xQueueSendToBack(motorCtrlQueue, &motorsSpeed, portMAX_DELAY);
		}

		/* Free space where the command was held */
		vPortFree(command);
	}
}
#endif /* FOLLOW_TRAJECTORY */

#ifdef FOLLOW_TRAJECTORY
void TaskTrajectory(void *p) {
	TelemetryData_Struct telemetry;
	MotorSpeed_Struct motorSpeed;
	portTickType wakeTime = xTaskGetTickCount();
	uint8_t requestNumber = 2;

	while(1) {
		/* Wait for next sampling period */
		vTaskDelayUntil(&wakeTime, 10/portTICK_RATE_MS);

		float usedSpace = TBgetUsedSpace();
		if (usedSpace < 0.1f && requestNumber == 2) { safePrint(35, "<#Please send %d more points#>\n", 7*TBgetSize()/8); requestNumber = 3;}
		else if (usedSpace < 0.2f && requestNumber == 1) { safePrint(35, "<#Please send %d more points#>\n", 3*TBgetSize()/4); requestNumber = 2;}
		else if (usedSpace < 0.45f && requestNumber == 0) { safePrint(35, "<#Please send %d more points#>\n", TBgetSize()/2); requestNumber = 1;}
		else if (usedSpace >= 0.45f) requestNumber = 0;

		TrajectoryPoint_Ptr point = TBgetNextPoint();
		if (point != NULL) {
			getTelemetry(&telemetry);
			calculateTrajectoryControll(&telemetry, point, &motorSpeed);
		}
		else {
			motorSpeed.LeftSpeed = motorSpeed.RightSpeed = 0.0f;
		}

		//xQueueSendToBack(motorCtrlQueue, &motorSpeed, portMAX_DELAY); // order motors to drive with different speed, wait for them to accept
	}
}
#endif /* FOLLOW_TRAJECTORY */

void TaskUSBWiFiBridge(void *p) {
	WiFi2USBBufferQueue = xQueueCreate(50, sizeof(char));
	USB2WiFiBufferQueue = xQueueCreate(50, sizeof(char));

	xQueueSetHandle queueSet = xQueueCreateSet(100);
	xQueueAddToSet((xQueueSetMemberHandle)WiFi2USBBufferQueue, queueSet);
	xQueueAddToSet((xQueueSetMemberHandle)USB2WiFiBufferQueue, queueSet);

	char byte;

	while(1) {
		xQueueSetMemberHandle activeQueue = xQueueSelectFromSet(queueSet, portMAX_DELAY);
		if (activeQueue == WiFi2USBBufferQueue) {
			xQueueReceive(WiFi2USBBufferQueue, &byte, 0);
			while (USART_GetFlagStatus(COM_USART, USART_FLAG_TXE) == RESET);
			USART_SendData(COM_USART, byte);
		}
		else {
			xQueueReceive(USB2WiFiBufferQueue, &byte, 0);
			while (USART_GetFlagStatus(WIFI_USART, USART_FLAG_TXE) == RESET);
			USART_SendData(WIFI_USART, byte);
		}
	}
}

#ifdef FOLLOW_TRAJECTORY
void calculateTrajectoryControll(const TelemetryData_Struct * currentPosition,
								 TrajectoryPoint_Ptr trajectoryPoint,
								 MotorSpeed_Struct * outputSpeeds)
{
	float s, c, s_r, c_r;
	float e_x, e_y, e_s, e_c, e_c_term;
	float v_b, w_b;
	float v, w;

	float diff_x  = trajectoryPoint->X * 1e4f - currentPosition->X; // trajectoryPoint is in metres but here mm are needed
	float diff_y  = trajectoryPoint->Y * 1e4f - currentPosition->Y;
	//float diff_fi = trajectoryPoint->O - currentPosition->O; // unused?

	// buehuehuehue :D anyway, is float equivalent to float32_t???
	arm_sin_cos_f32(currentPosition->O, &s, &c);
	arm_sin_cos_f32(trajectoryPoint->O, &s_r, &c_r);

	//calculate error model
	e_x =  c * diff_x + s * diff_y;
	e_y = -s * diff_x + c * diff_y;
	e_s = s_r * c - c_r * s;
	e_c = c_r * c + s_r * s - 1.0f;

	//calculate feedback signal for n=2 and a=7 (see whitepaper)

	e_c_term = 1.0f + e_c / 7.0f; //a = 7;
	e_c_term = powf(e_c_term, 2.0f);

	v_b = globalTrajectoryControlGains.k_x * e_x;
	w_b = globalTrajectoryControlGains.k * trajectoryPoint->V * e_y * e_c_term + globalTrajectoryControlGains.k_s * e_s * powf(e_c_term, 2.0f); //n = 2

	//[m/s]
	v = trajectoryPoint->V * (e_c + 1.0f) + v_b;
	w = trajectoryPoint->W + w_b;

	// [rad/s]
	outputSpeeds->LeftSpeed  = (v - w * ROBOT_DIAM / (1000.0f * 2.0f)) / WHEEL_DIAM;
	outputSpeeds->RightSpeed = (v + w * ROBOT_DIAM / (1000.0f * 2.0f)) / WHEEL_DIAM;
}
#endif

void reportStackUsage() {
	safePrint(45, "High water mark of stack usage (free space)\n");
	safePrint(28, "printfConsumerTask: %d\n", uxTaskGetStackHighWaterMark(printfConsumerTask));
	safePrint(28, "commandHandlerTask: %d\n", uxTaskGetStackHighWaterMark(commandHandlerTask));
	safePrint(23, "motorCtrlTask: %d\n", uxTaskGetStackHighWaterMark(motorCtrlTask));
	safePrint(17, "RC5Task: %d\n", uxTaskGetStackHighWaterMark(RC5Task));
	safePrint(23, "telemetryTask: %d\n", uxTaskGetStackHighWaterMark(telemetryTask));
	safePrint(27, "USBWiFiBridgeTask: %d\n", uxTaskGetStackHighWaterMark(USBWiFiBridgeTask));
	safePrint(29, "commInputBufferTask: %d\n", uxTaskGetStackHighWaterMark(commInputBufferTask));
#ifdef USE_IMU_TELEMETRY
	safePrint(17, "imuTask: %d\n", uxTaskGetStackHighWaterMark(imuTask));
#endif
#ifdef FOLLOW_TRAJECTORY
	safePrint(24, "trajectoryTask: %d\n", uxTaskGetStackHighWaterMark(trajectoryTask));
#else
	safePrint(19, "driveTask: %d\n", uxTaskGetStackHighWaterMark(driveTask));
#endif
}

/* ISR for COM USART - must be called in USARTx_IRQHandler */
void COMAction(void) {
	portBASE_TYPE contextSwitch = pdFALSE;
	PrintInput_Struct in = {.Source = PrintSource_Type_USB};

	if (USART_GetITStatus(COM_USART, USART_IT_RXNE) == SET) {
		in.Input = USART_ReceiveData(COM_USART);
		if (getWiFi2USBBridgeStatus() == ON) {
			xQueueSendToBackFromISR(USB2WiFiBufferQueue, &in.Input, &contextSwitch);
		}
		else {
			xQueueSendToBackFromISR(commInputBufferQueue, &in, &contextSwitch);
		}
		USART_ClearFlag(COM_USART, USART_FLAG_RXNE);
	}
	else if (USART_GetITStatus(COM_USART, USART_IT_TC) == SET) {
		xSemaphoreGiveFromISR(comUSARTTCSemaphore, &contextSwitch);
		USART_ITConfig(COM_USART, USART_IT_TC, DISABLE);
		USART_ClearFlag(COM_USART, USART_FLAG_TC);
	}

	portEND_SWITCHING_ISR(contextSwitch);
}

/* ISR for WIFI Rx interrupt */
void WIFIAction(void) {
	portBASE_TYPE contextSwitch = pdFALSE;
	PrintInput_Struct in = {.Source = PrintSource_Type_WiFi};

	if (USART_GetITStatus(WIFI_USART, USART_IT_RXNE) == SET) {
		in.Input = USART_ReceiveData(WIFI_USART);
		if (getWiFi2USBBridgeStatus() == ON) {
			xQueueSendToBackFromISR(WiFi2USBBufferQueue, &in.Input, &contextSwitch);
		}
		else {
			xQueueSendToBackFromISR(commInputBufferQueue, &in, &contextSwitch);
		}
		USART_ClearFlag(WIFI_USART, USART_FLAG_RXNE);
	}
	else if (USART_GetITStatus(WIFI_USART, USART_IT_TC) == SET) {
		xSemaphoreGiveFromISR(wifiUSARTTCSemaphore, &contextSwitch);
		USART_ITConfig(WIFI_USART, USART_IT_TC, DISABLE);
		USART_ClearFlag(WIFI_USART, USART_FLAG_TC);
	}

	portEND_SWITCHING_ISR(contextSwitch);
}

/* ISR for COM DMA Tx */
void COMDMANotify(void) {
	portBASE_TYPE contextSwitch = pdFALSE;
	xSemaphoreGiveFromISR(comDMATCSemaphore, &contextSwitch);
	DMA_ClearFlag(COM_TX_DMA_STREAM, COM_TX_DMA_FLAG_TCIF);
	portEND_SWITCHING_ISR(contextSwitch);
}

/* ISR for WIFI DMA Tx */
void WiFiDMANotify() {
	portBASE_TYPE contextSwitch = pdFALSE;
	xSemaphoreGiveFromISR(wifiDMATCSemaphore, &contextSwitch);
	DMA_ClearFlag(WIFI_TX_DMA_STREAM, WIFI_TX_DMA_FLAG_TCIF);
	portEND_SWITCHING_ISR(contextSwitch);
}

#ifdef FOLLOW_TRAJECTORY
/* ISR for WiFi DMA Rx */
void RawStreamDMAIncoming(void) {
	TBDMATransferCompletedSlot();
}
#endif

#ifdef USE_IMU_TELEMETRY
/* ISR for EXTI Gyro */
void IMUGyroReady() {
	portBASE_TYPE contextSwitch = pdFALSE;
	xSemaphoreGiveFromISR(imuGyroReady, &contextSwitch);
	portEND_SWITCHING_ISR(contextSwitch);
}

/* ISR for EXTI Accelerometer */
void IMUAccReady() {
	portBASE_TYPE contextSwitch = pdFALSE;
	xSemaphoreGiveFromISR(imuAccReady, &contextSwitch);
	portEND_SWITCHING_ISR(contextSwitch);
}

/* ISR for EXTI Magnetometer */
void IMUMagReady() {
	portBASE_TYPE contextSwitch = pdFALSE;
	xSemaphoreGiveFromISR(imuMagReady, &contextSwitch);
	portEND_SWITCHING_ISR(contextSwitch);
}
#endif /* USE_IMU_TELEMETRY */

int safePrint(const size_t length, const char *format, ...) {
	va_list arglist;
	va_start(arglist, format);
	char *pbuf = (char*)pvPortMalloc(length*sizeof(char));
	int ret = vsnprintf(pbuf, length, format, arglist);
	if (xQueueSendToBack(printfQueue, &pbuf, 0) == errQUEUE_FULL) {
		lightLED(5, ON);
		vPortFree(pbuf);
	}
	va_end(arglist);
	return ret;
}

int safePrintFromISR(const size_t length, const char *format, ...) {
	va_list arglist;
	va_start(arglist, format);
	char *pbuf = (char*)pvPortMalloc(length*sizeof(char));
	int ret = vsnprintf(pbuf, length, format, arglist);
	portBASE_TYPE contextSwitch = pdFALSE;
	if (xQueueSendToBackFromISR(printfQueue, &pbuf, &contextSwitch) == errQUEUE_FULL) {
		lightLED(5, ON);
		vPortFree(pbuf);
	}
	va_end(arglist);
	portEND_SWITCHING_ISR(contextSwitch);
	return ret;
}

/* Called from ISR */
void Switch1Changed() {
	if (getSwitchStatus(1) == ON) {
		enableUSB(ENABLE);
	}
	else {
		enableUSB(DISABLE);
	}
}

/* Called from ISR */
void Switch2Changed() {
	if (getSwitchStatus(2) == ON) {
		enableWiFi(ENABLE);
	}
	else {
		enableWiFi(DISABLE);
	}
}

/* Called from ISR */
void Switch3Changed() {
	if (getSwitchStatus(3) == ON) {
		enableWiFi2USBBridge(ENABLE);
	}
	else {
		enableWiFi2USBBridge(DISABLE);
	}
}

/* Called from ISR */
void Switch4Changed() {
	safePrint(16, "Four changed\n");
}

/* Called from ISR */
void Switch5Changed() {
	safePrint(16, "Five changed\n");
}

/* Called from ISR */
void Switch6Changed() {
	if (getSwitchStatus(6) == ON) {
		enableLantern(ENABLE);
	}
	else {
		enableLantern(DISABLE);
	}
}

/* Analog watchdog interrupt handler */
void BatteryTooLow() {
	/* Stop all tasks */
	vTaskEndScheduler();
	/* Inform about low battery level */
	printf("Battery Low!\nShutting Down\n");
	/* Turn off motors and lantern */
	enableMotors(DISABLE);
	enableLantern(DISABLE);
	/* Light 2 red LEDs */
	lightLED(6, ON);
	lightLED(1, ON);
	lightLED(2, OFF);
	lightLED(3, OFF);
	lightLED(4, OFF);
	lightLED(5, OFF);
	/* Infinite loop */
	while(1) {
		/* Reload watchdog to prevent from reset */
		IWDG_ReloadCounter();
	}
}

/* ISR for OS busy timer update */
void OSBusyTimerHandler() {
	uint16_t p = TIM_GetCounter(CPUUSAGE_CNT_TIM);
	TIM_SetCounter(CPUUSAGE_CNT_TIM, 0);
	globalCPUUsage = (float)p / (float)(CPUUSAGE_TIM_PERIOD + 1);
}

#ifdef USE_IMU_TELEMETRY
void IMUI2CEVHandler(void) {
	static uint32_t requiredFlag = 0;
	portBASE_TYPE contextSwitch = pdFALSE;

	// pdTRUE if really received
	xQueueReceiveFromISR(I2CEVFlagQueue, &requiredFlag, &contextSwitch);

	if (requiredFlag != 0) {
		if (I2C_CheckEvent(IMU_I2C, requiredFlag)) {
			xSemaphoreGiveFromISR(imuI2CEV, &contextSwitch);
			I2C_ITConfig(IMU_I2C, I2C_IT_EVT, DISABLE);
			requiredFlag = 0;
		}
	}

	portEND_SWITCHING_ISR(contextSwitch);
}
#endif /* USE_IMU_TELEMETRY */

float normalizeOrientation(float in) {
	return (in > M_PI ? in - 2.0f*M_PI : (in <= -M_PI ? in + 2.0f*M_PI : in));
}

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

	/* COM UART configuration */
	/* Enabling clock for GPIO */
	RCC_AHB1PeriphClockCmd(COM_GPIO_CLOCK, ENABLE);
	/* Enabling clock for USART */
	COM_USART_CLOCK_FUN(COM_USART_CLOCK, ENABLE);
	/* Redirecting port lines to USART */
	GPIO_PinAFConfig(COM_GPIO, COM_GPIO_PINSOURCE_TX, COM_GPIO_AF);
	GPIO_PinAFConfig(COM_GPIO, COM_GPIO_PINSOURCE_RX, COM_GPIO_AF);
	/* Configuring GPIO pins */
	GPIO_InitStructure.GPIO_Pin = COM_GPIO_PIN_TX | COM_GPIO_PIN_RX;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(COM_GPIO, &GPIO_InitStructure);
	/* Oversampling by 8 enable for higher speeds */
	USART_OverSampling8Cmd(COM_USART, ENABLE);
	/* Configuring UART 230400bits/s, no parity, 1 stop bit */
	USART_InitStructure.USART_BaudRate = COM_USART_SPEED;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(COM_USART, &USART_InitStructure);
	/* Configuring interrupt for USART */
	NVIC_InitStructure.NVIC_IRQChannel = COM_NVIC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PRIORITY_ISR_COMUSART;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
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
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC16;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) (&COM_USART -> DR);
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	DMA_InitStructure.DMA_Channel = COM_TX_DMA_CHANNEL;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = 1;											// Number of data items to transfer
	DMA_Init(COM_TX_DMA_STREAM, &DMA_InitStructure);
	/* Disabling double buffer mode */
	DMA_DoubleBufferModeCmd(COM_TX_DMA_STREAM, DISABLE);
	/* Enabling interrupt after finished transmission */
	NVIC_InitStructure.NVIC_IRQChannel = COM_TX_DMA_NVIC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PRIORITY_ISR_COMDMATX;
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
	GPIO_InitStructure.GPIO_Pin = WIFI_GPIO_USART_TX_PIN | WIFI_GPIO_USART_RX_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(WIFI_GPIO_USART, &GPIO_InitStructure);
	/* And logic pins for reset etc */
	GPIO_InitStructure.GPIO_Pin = WIFI_GPIO_SIG_RESET_PIN | WIFI_GPIO_SIG_LMTFRES_PIN | WIFI_GPIO_SIG_CMDDATA_PIN | WIFI_GPIO_SIG_ALARM_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	/* Remember to set it default high */
	GPIO_SetBits(WIFI_GPIO_SIG, WIFI_GPIO_SIG_RESET_PIN | WIFI_GPIO_SIG_LMTFRES_PIN | WIFI_GPIO_SIG_ALARM_PIN);
	GPIO_ResetBits(WIFI_GPIO_SIG, WIFI_GPIO_SIG_CMDDATA_PIN);
	GPIO_Init(WIFI_GPIO_SIG, &GPIO_InitStructure);
	/* Configuring UART, no parity, 1 stop bit */
	USART_InitStructure.USART_BaudRate = WIFI_USART_SPEED;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(WIFI_USART, &USART_InitStructure);
	/* Configuring interrupt for USART */
	NVIC_InitStructure.NVIC_IRQChannel = WIFI_NVIC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PRIORITY_ISR_WIFIUSART;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
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
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC16;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) (&WIFI_USART -> DR);
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	DMA_InitStructure.DMA_Channel = WIFI_TX_DMA_CHANNEL;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_Init(WIFI_TX_DMA_STREAM, &DMA_InitStructure);
	/* Disabling double buffer mode */
	DMA_DoubleBufferModeCmd(WIFI_TX_DMA_STREAM, DISABLE);
	/* Enabling interrupt after finished transmission */
	NVIC_InitStructure.NVIC_IRQChannel = WIFI_TX_DMA_NVIC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PRIORITY_ISR_WIFIDMATX;
	NVIC_Init(&NVIC_InitStructure);
	DMA_ITConfig(WIFI_TX_DMA_STREAM, DMA_IT_TC, ENABLE);
#ifdef FOLLOW_TRAJECTORY
	/*
	 * Configuring DMA stream for reception over USART
	 * Stream is configured to download data from Rx to memory
	 * Data is received by USART as 8-bit BYTE, it is stored in FIFO
	 * and transmitted in burst of 16 bytes
	 * Data is saved to memory as 32-bit WORD, so each burst moves 4x WORD items
	 */
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	//DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)tab1;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&WIFI_USART -> DR);
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_Channel = WIFI_RX_DMA_CHANNEL;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_Init(WIFI_RX_DMA_STREAM, &DMA_InitStructure);
	DMA_FlowControllerConfig(WIFI_RX_DMA_STREAM, DMA_FlowCtrl_Memory);
	/* Enabling double buffer mode */
	//DMA_DoubleBufferModeConfig(COM_RX_DMA_STREAM, (uint32_t)tab2, DMA_Memory_0);
	//DMA_DoubleBufferModeCmd(COM_RX_DMA_STREAM, ENABLE);
	/* Enabling interrupt after receiving */
	NVIC_InitStructure.NVIC_IRQChannel = WIFI_RX_DMA_NVIC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PRIORITY_ISR_WIFIDMARX;
	NVIC_Init(&NVIC_InitStructure);
	DMA_ITConfig(WIFI_RX_DMA_STREAM, DMA_IT_TC, ENABLE);
	/* Enabling UART */
#endif
	USART_Cmd(WIFI_USART, ENABLE);


	/* Configuring lantern timer to generate pattern */
	/* Enable clocks */
	LANTERN_TIM_CLOCK_FUN(LANTERN_TIM_CLOCK, ENABLE);
	RCC_AHB1PeriphClockCmd(LANTERN_GPIO_CLOCK, ENABLE);
	/* Configuring pins as alternate function */
	GPIO_InitStructure.GPIO_Pin = LANTERN_GPIO_PIN1 | LANTERN_GPIO_PIN2 | LANTERN_GPIO_PIN3 | LANTERN_GPIO_PIN4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(LANTERN_GPIO, &GPIO_InitStructure);
	/* Connect TIM Channels to GPIOs */
	GPIO_PinAFConfig(LANTERN_GPIO, LANTERN_GPIO_PINSOURCE1, LANTERN_GPIO_AF);
	GPIO_PinAFConfig(LANTERN_GPIO, LANTERN_GPIO_PINSOURCE2, LANTERN_GPIO_AF);
	GPIO_PinAFConfig(LANTERN_GPIO, LANTERN_GPIO_PINSOURCE3, LANTERN_GPIO_AF);
	GPIO_PinAFConfig(LANTERN_GPIO, LANTERN_GPIO_PINSOURCE4, LANTERN_GPIO_AF);
	/* Time base configuration */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = LANTERN_TIM_PRESCALER;
	TIM_TimeBaseStructure.TIM_Period = LANTERN_TIM_PERIOD;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
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
	NVIC_InitStructure.NVIC_IRQChannel = LANTERN_NVIC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PRIORITY_ISR_LANTERN;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	TIM_ITConfig(LANTERN_TIM, TIM_IT_Update, ENABLE);
	/* TIM enable counter */
	TIM_Cmd(LANTERN_TIM, ENABLE);


	/* Configuring GPIOs for motors */
	/* Clocks */
	RCC_AHB1PeriphClockCmd(MOTORL_GPIO_CLOCK, ENABLE);
	RCC_AHB1PeriphClockCmd(MOTORR_GPIO_CLOCK, ENABLE);
	/* Pins */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Pin = MOTORL_GPIO_INA_PIN | MOTORL_GPIO_INB_PIN | MOTORL_GPIO_ENA_PIN | MOTORL_GPIO_ENB_PIN;
	GPIO_Init(MOTORL_GPIO, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = MOTORR_GPIO_INA_PIN | MOTORR_GPIO_INB_PIN | MOTORR_GPIO_ENA_PIN | MOTORR_GPIO_ENB_PIN;
	GPIO_Init(MOTORR_GPIO, &GPIO_InitStructure);
	/* Configuring PWMs for motors */
	/* Clock for GPIOs */
	RCC_AHB1PeriphClockCmd(MOTOR_PWM_GPIO_CLOCK, ENABLE);
	/* Clock for timer */
	MOTOR_PWM_TIM_CLOCK_FUN(MOTOR_PWM_TIM_CLOCK, ENABLE);
	/* Pins as alternate function out */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Pin = MOTOR_PWM_GPIO_LEFT_PIN | MOTOR_PWM_GPIO_RIGHT_PIN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MOTOR_PWM_GPIO, &GPIO_InitStructure);
	/* Redirect pins */
	GPIO_PinAFConfig(MOTOR_PWM_GPIO, MOTOR_PWM_GPIO_LEFT_PINSOURCE, MOTOR_PWM_GPIO_AF);
	GPIO_PinAFConfig(MOTOR_PWM_GPIO, MOTOR_PWM_GPIO_RIGHT_PINSOURCE, MOTOR_PWM_GPIO_AF);
	/* Timer configuration */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = MOTOR_PWM_TIM_PRESCALER;									// 84MHz
	TIM_TimeBaseStructure.TIM_Period = MOTOR_PWM_TIM_PERIOD;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(MOTOR_PWM_TIM, &TIM_TimeBaseStructure);
	/* Outputs for timer */
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC1Init(MOTOR_PWM_TIM, &TIM_OCInitStructure);
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
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_InitStructure.GPIO_Pin = LEDS_GPIO_1_PIN;
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
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = ENCODERS_GPIO_LA_PIN;
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
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)0x0000;
	TIM_TimeBaseStructure.TIM_Period = (uint32_t)0xFFFFFFFFL;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
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
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Pin = BATTLVL_GPIO_PIN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_Init(BATTLVL_GPIO, &GPIO_InitStructure);
	/* ADC common init */
	ADC_CommonStructInit(&ADC_CommonInitStructure);
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div8;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInit(&ADC_CommonInitStructure);
	/* Configuring ADC line, continuous mode, 12 bit resolution */
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_Init(BATTLVL_ADC, &ADC_InitStructure);
	/* Configuring channel */
	ADC_RegularChannelConfig(BATTLVL_ADC, BATTLVL_ADC_CHANNEL, 1, ADC_SampleTime_480Cycles);
	/* Analog watchdog configuration */
	ADC_AnalogWatchdogCmd(BATTLVL_ADC, ADC_AnalogWatchdog_SingleRegEnable);
	ADC_AnalogWatchdogSingleChannelConfig(BATTLVL_ADC, BATTLVL_ADC_CHANNEL);
	ADC_AnalogWatchdogThresholdsConfig(BATTLVL_ADC, 0x0FFF, (uint16_t)(BATTLVL_THRESHOLD_VOLTAGE/BATTLVL_CONV_2_VOLTAGE));
	/* Interrupt for analog watchdog */
	NVIC_InitStructure.NVIC_IRQChannel = ADC_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PRIORITY_ISR_ADC;
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
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Pin = POT_GPIO_PIN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
	GPIO_Init(POT_GPIO, &GPIO_InitStructure);
	/* ADC common init was done before */
	/* ADC configuring, 12-bit single conversion triggered by TIM8 CC1 */
	ADC_StructInit(&ADC_InitStructure);
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T8_CC1;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
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
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = HELPER_TIM_PRESCALER;
	TIM_TimeBaseStructure.TIM_Period = HELPER_TIM_PERIOD;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
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
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = SERVO_GPIO_PIN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(SERVO_GPIO, &GPIO_InitStructure);
	/* Redirect pins */
	GPIO_PinAFConfig(SERVO_GPIO, SERVO_GPIO_PINSOURCE, SERVO_GPIO_AF);
	/* Timer configuration */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = SERVO_TIM_PRESCALER;
	TIM_TimeBaseStructure.TIM_Period = SERVO_TIM_PERIOD;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(SERVO_TIM, &TIM_TimeBaseStructure);
	/* Outputs for timer */
	TIM_OCStructInit(&TIM_OCInitStructure);
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC2Init(SERVO_TIM, &TIM_OCInitStructure);
	setPenUp();
	/* Enable preload */
	TIM_OC2PreloadConfig(SERVO_TIM, TIM_OCPreload_Enable);
	/* Enable timer */
	TIM_Cmd(SERVO_TIM, ENABLE);


	/* Configure independent watchdog */
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler(IWDG_Prescaler_4);
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	/* It should give 200ms */
	IWDG_SetReload(1600);
	/* Set halt mode in debug */
	DBGMCU_APB1PeriphConfig(DBGMCU_IWDG_STOP, ENABLE);
	/* Not started here */


	/* Configure two timers to time OS busy percent */
	CPUUSAGE_TIM_CLOCK_FUN(CPUUSAGE_TIM_CLOCKS, ENABLE);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = CPUUSAGE_TIM_PRESCALER;
	TIM_TimeBaseStructure.TIM_Period = CPUUSAGE_TIM_PERIOD;
	TIM_TimeBaseStructure.TIM_ClockDivision - TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(CPUUSAGE_BASE_TIM, &TIM_TimeBaseStructure);
	TIM_TimeBaseInit(CPUUSAGE_CNT_TIM, &TIM_TimeBaseStructure);
	TIM_ITConfig(CPUUSAGE_BASE_TIM, TIM_IT_Update, ENABLE);
	NVIC_InitStructure.NVIC_IRQChannel = CPUUSAGE_BASE_TIM_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PRIORITY_ISR_OSBUSY;
	NVIC_Init(&NVIC_InitStructure);
	TIM_Cmd(CPUUSAGE_BASE_TIM, ENABLE);


	/* Configuring switches and external interrupts - last of all*/
	/* GPIO clock */
	RCC_AHB1PeriphClockCmd(SWITCHES_GPIO_CLOCK, ENABLE);
	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	/* Pins as inputs with pullup */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Pin = SWITCHES_GPIO_1_PIN | SWITCHES_GPIO_2_PIN | SWITCHES_GPIO_3_PIN | SWITCHES_GPIO_4_PIN | SWITCHES_GPIO_5_PIN | SWITCHES_GPIO_6_PIN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
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
	if (getSwitchStatus(3) == ON) enableWiFi2USBBridge(ENABLE);
	/* Switch 4 */
	EXTI_InitStructure.EXTI_Line = SWITCHES_EXTI_4_LINE;
	EXTI_Init(&EXTI_InitStructure);
	/* Switch 5 */
	EXTI_InitStructure.EXTI_Line = SWITCHES_EXTI_5_LINE;
	EXTI_Init(&EXTI_InitStructure);
	/* Switch 6 */
	EXTI_InitStructure.EXTI_Line = SWITCHES_EXTI_6_LINE;
	EXTI_Init(&EXTI_InitStructure);
	if (getSwitchStatus(6) == OFF) enableLantern(DISABLE);
	/* NVIC config */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PRIORITY_ISR_SWITCHES;
	NVIC_Init(&NVIC_InitStructure);
	NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_Init(&NVIC_InitStructure);
	/* Configure timer used to debounce switches */
	/* Clock enable */
	SWITCHES_TIM_CLOCK_FUN(SWITCHES_TIM_CLOCK, ENABLE);
	/* Frequency configuration */
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = SWITCHES_TIM_PRESCALER;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = SWITCHES_TIM_PERIOD;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(SWITCHES_TIM, &TIM_TimeBaseStructure);
	/* Interrupt generation */
	TIM_ITConfig(SWITCHES_TIM, TIM_IT_Update, ENABLE);
	/* Interrupt configuration */
	NVIC_InitStructure.NVIC_IRQChannel = SWITCHES_NVIC_TIM_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PRIORITY_ISR_SWITCHES_TIM;
	NVIC_Init(&NVIC_InitStructure);
	/* DO NOT START HERE */
}

void vApplicationMallocFailedHook( void )
{
	while(1);
}

void vApplicationStackOverflowHook( void )
{
	while(1);
}

/*void vApplicationIdleHook( void )
{

}

void vApplicationTickHook( void )
{

}*/

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	vTaskEndScheduler();
	printf("Wrong parameters value: file %s on line %ld\r\n", file, line);
	while (1);
}

#endif
