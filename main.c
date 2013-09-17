#include <stm32f4xx.h>

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>

#include "arm_math.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "portmacro.h"

#include "compilation.h"

#ifdef USE_CUSTOM_MOTOR_CONTROLLER
#include "motorController.h"
#endif

#ifdef FOLLOW_TRAJECTORY
#include "pointsBuffer.h"
#endif

#ifdef USE_IMU_TELEMETRY
#include "i2chelpers.h"
#include "imu.h"
#include "vector.h"
#include "complementary.h"
#endif

#include "main.h"
#include "commands.h"
#include "priorities.h"
#include "hardware.h"
#include "rc5_tim_exti.h"
#include "hwinterface.h"


/* Useful defines for motor control */
#define M_PI 				(3.14159265358979323846f)			/*<< PI */
#define IMPS_PER_REV 		(3592.0f)							/*<< Number of encoder impulses per wheel revolution */
#define IMPS_TO_RAD 		(2.0f * M_PI / IMPS_PER_REV)		/*<< Coefficient to convert encoder impulses to radians */
#define ROBOT_DIAM			(187.3f)//(187.0f)//(184.9f)		/*<< Distance between two wheels */
#define WHEEL_DIAM 			(69.76f)//(70.0f)					/*<< Wheel diameter */
#define IMPS_TO_MM_TRAVELED (WHEEL_DIAM * M_PI / IMPS_PER_REV)	/*<< Coefficient to convert encoder inpulses to distance traveled on wheel */
#define RAD_TO_MM_TRAVELED	(WHEEL_DIAM / 2.0f)					/*<< Coefficient to convert radians to distance traveled on wheel */
#define DEGREES_TO_RAD		(M_PI / 180.0f)						/*<< Coefficient to convert degrees to radians */
#define BUF_RX_LEN 20											/*<< Maximum length of UART command */

#ifndef FOLLOW_TRAJECTORY
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

/* Struct holding motors' speeds. Choosen unit is rad/sec */
typedef struct {
	float LeftSpeed;				/*<< Left motor's speed */
	float RightSpeed;				/*<< Right motor's speed */
} MotorSpeed_Struct;

/* Type of telemetry update */
typedef enum {
	TelemetryUpdate_Source_Odometry = 0,	/*<< Update from odometry - encoders */
#ifdef USE_IMU_TELEMETRY
	TelemetryUpdate_Source_IMU				/*<< Update from IMU */
#endif
} TelemetryUpdate_Source;

/* Struct to hold telemetry updates from various sources. Based on these updates, position and orientation is calculated */
typedef struct {
	TelemetryUpdate_Source Source;			/*<< Update source */
	float dX;								/*<< Change in X position */
	float dY;								/*<< Change in Y position */
	float dO;								/*<< Change of orientation angle (radians) */
} TelemetryUpdate_Struct;

/* Struct for holding position and orientation */
typedef struct {
	float X;				/*<< X coordinate */
	float Y;				/*<< Y coordinate */
	float O;				/*<< Orientation angle coordinate in radians */
} TelemetryData_Struct;

/* Types of different logging commands */
typedef enum {
	Logging_Type_Telemetry = 't',		/*<< Log position and orientation when it changes */
	Logging_Type_Speed = 's',			/*<< Log wheels speed */
	Logging_Type_Events = 'e'			/*<< Log system events */
} Logging_Type;

#define IS_LOGGING_TYPE(x) (x == Logging_Type_Telemetry || x == Logging_Type_Speed || x == Logging_Type_Events)

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

/*
 * @brief Handles various commands from various places
 * @param command Pointer to the beginning of a string containing command
 * @retval None
 */
static void COMHandle(const char * command);

/*
 * @brief Print formated text via USB and WiFi (if enabled) in thread-safe and non-blocking way.
 * This function performs the same logical action as normal printf.
 * This function uses <stdio.h> which is very resource-consuming and takes a lot of stack space
 * @param length Maximum length that will be allocated for printed string. If string is longer, then it will be truncated
 * @param format Refer to printf
 * @retval Refer to printf
 */
static int 	safePrint(const size_t length, const char *format, ...);

/* Implementation of safePrint that is safe to use in interrupts. DO NOT USE NORMAL VERSION IN ISR! */
static int 	safePrintFromISR(const size_t length, const char *format, ...);

#ifdef USE_IMU_TELEMETRY
/* Function used by IMU software watchdog */
static void imuWatchdogOverrun(xTimerHandle xTimer);
#endif

/* Returns current up-to-date telemetry data and saves it in provided structure. This function provides mutual exclusion and data coherency */
static void getTelemetry(TelemetryData_Struct *data);

/* Returns current telemetry data without orientation normalization to +-M_PI */
static void getTelemetryRaw(TelemetryData_Struct *data);

/* Returns normalized orientation angle provided as input in radians, output is [-PI, +PI] */
static float normalizeOrientation(float in) { return (in > M_PI ? in - 2.0f*M_PI : (in <= -M_PI ? in + 2.0f*M_PI : in)); }

#ifdef USE_IMU_TELEMETRY
/* Sets I2C peripheral */
static void initI2CforIMU(void);

/* Initializes globalMagnetometerImprov */
static void initMagnetometerImprovInstance(float x0);
#endif

/* Prints free stack space for each running task */
static void reportStackUsage();

void TaskPrintfConsumer(void *);		// Task handling safePrint invocations and printing everything on active interfaces
void TaskLED(void *);					// Blinking LED task; indication that scheduler is running and no task is hang; watchdog resetting
void TaskCommandHandler(void *);		// Task handling incomming commands
#ifdef USE_IMU_TELEMETRY
void TaskIMU(void *);					// Task for reading from IMU and calculating orientation based on IMU readings
void TaskIMUMagScaling(void *);			// Task for magnetometer scaling. This is not an infinite task.
#endif
void TaskRC5(void *);					// Task for handling commands from RC5 remote
#ifndef FOLLOW_TRAJECTORY
void TaskDrive(void *);					// Task controlling trajectory. Issues wheel's speed commands, checks if target is reached, calculates best route
#else
void TaskTrajectory(void *);			// Task for controlling trajectory using Ferdek's regulator
#endif
void TaskMotorCtrl(void *);				// Motors' speed regulator
void TaskTelemetry(void *);				// Task calculating global position and orientation based on all available sources (IMU, odometry, camera, etc. )
void TaskUSBWiFiBridge(void *);			// Task for USB-WiFi bridge to transfer commands between two UARTs
void TaskInputBuffer(void *);			// Task to handle input characters from any source - it makes possible to set very high transfer speeds

xQueueHandle printfQueue;				// Queue for safePrint strings to send via active interfaces
xQueueHandle commandQueue;				// Queue for storing commands to do
#ifndef FOLLOW_TRAJECTORY
xQueueHandle driveQueue;				// Queue for storing driving commands (probably send in huge blocks like 100 commands at once
#endif
xQueueHandle motorCtrlQueue;			// One-element queue for setting wheel's speed
xQueueHandle telemetryQueue;			// Queue for sending updates to telemetry task. This queue holds updates from all available sources
xQueueHandle WiFi2USBBufferQueue;		// Buffer for WiFi to USB characters
xQueueHandle USB2WiFiBufferQueue;		// Buffer for USB to WiFi characters
#ifdef USE_IMU_TELEMETRY
xQueueHandle I2CEVFlagQueue;			// Buffer for I2C event interrupt that holds new event flag to wait for
xQueueHandle magnetometerScalingQueue = NULL;	// Queue for data from IMU task for magnetometer scaling. Created on demand in scaling task.
#endif
xQueueHandle commInputBufferQueue;		// Buffer for input characters

xTaskHandle printfConsumerTask;
xTaskHandle commandHandlerTask;
#ifdef USE_IMU_TELEMETRY
xTaskHandle imuTask;
xTaskHandle imuMagScalingTask;
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
#ifdef USE_IMU_TELEMETRY
xSemaphoreHandle imuPrintRequest;				// request for IMU to print most up-to-date reading
xSemaphoreHandle imuGyroReady;					// gyro ready flag set in interrupt
xSemaphoreHandle imuAccReady;					// accelerometer ready flag set in interrupt
xSemaphoreHandle imuMagReady;					// magnetometer ready flag set in interrupt
xSemaphoreHandle imuI2CEV;						// semaphore to indicate correct event in I2C protocol
xSemaphoreHandle imuMagScalingReq;				// request to perform magnetometer scaling
#endif
xSemaphoreHandle rc5CommandReadySemaphore;		// used by RC5 API to inform about new finished transmission

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
volatile FunctionalState globalMagnetometerScalingInProgress = DISABLE;
volatile bool globalIMUHang = false;
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

void TaskTelemetry(void * p) {
	TelemetryUpdate_Struct update;
#ifdef USE_IMU_TELEMETRY
	portTickType startTime = xTaskGetTickCount();
	bool useIMU = false;
#endif

	while(1) {
#ifdef USE_IMU_TELEMETRY
		if (!useIMU) { // use IMU data after timeout to let magnetometer scaling kick in
			if ((xTaskGetTickCount() - startTime)/portTICK_RATE_MS > 10000) useIMU = true;
		}
#endif

		/* Wait indefinitely while there is no update */
		xQueueReceive(telemetryQueue, &update, portMAX_DELAY);

		/* Handle multiple updates */
		switch(update.Source) {
		case TelemetryUpdate_Source_Odometry:
			taskENTER_CRITICAL();
			{
				globalTelemetryData.X += update.dX;
				globalTelemetryData.Y += update.dY;
				globalTelemetryData.O += update.dO;
				if (globalLogTelemetry && (fabsf(update.dX) > 0.1f || fabsf(update.dY) > 0.1f || fabsf(update.dO) > 0.001f)) {
					safePrint(52, "Odometry update: X:%.2f Y:%.2f O:%.1f\n", globalTelemetryData.X, globalTelemetryData.Y, globalTelemetryData.O / DEGREES_TO_RAD);
				}
			}
			taskEXIT_CRITICAL();
			break;
#ifdef USE_IMU_TELEMETRY
		case TelemetryUpdate_Source_IMU:
			if (useIMU) {
			}
			break;
#endif
		default:
			if (globalLogEvents) safePrint(37, "Invalid telemetry update type: %d\n", update.Source);
			break;
		}
	}
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
		if (point == NULL) continue;

		// EXAMPLES BELOW

		// this gives telemetry data with orientation cormalized to +-PI radians
		getTelemetry(&telemetry);

		float ty = telemetry.Y;

		// this is how you can get point data
		float x = point->X;

		// note that the two lines below are WRONG and won't compile (point in const pointer to const struct)
		// point->X = 3;
		// point++;

		motorSpeed.LeftSpeed = 3.43f;
		motorSpeed.RightSpeed = 2.0f;
		xQueueSendToBack(motorCtrlQueue, &motorSpeed, portMAX_DELAY); // order motors to drive with different speed, wait for them to accept

	}
}
#endif /* FOLLOW_TRAJECTORY */

void TaskMotorCtrl(void * p) {
	portTickType wakeTime = xTaskGetTickCount();
	/* Speed is given in radians per second */
	MotorSpeed_Struct motorSpeed = {0.0f, 0.0f};

	//const float maxSpeedAllowed = 6300.0f * IMPS_TO_RAD; // 6300 is max ticks per second on both motors
	const float maxSpeedAllowed = 10.0f;  // 10 rad/s - It looks like this is the limit for motor controllers' parameters to hold
	const uint16_t delayMsPerPeriod = 10;

	float errorLeft, errorRight;
	float outLeft, outRight;
	float speedLeft, speedRight;
	int32_t prevPosLeft = getEncoderL(), prevPosRight = getEncoderR();
	int32_t posLeft, posRight;

	TelemetryUpdate_Struct telemetryUpdate = {.Source = TelemetryUpdate_Source_Odometry};
	TelemetryData_Struct telemetryData;

#ifdef USE_CUSTOM_MOTOR_CONTROLLER
	/*
	 * Custom controller
	 * input - speed [rad/s]
	 * output - PWM
	 */
#else
	/*
	 * Input to PID controller is error of rotational velocity in rad/sec
	 * Output is normalized speed scaled linearly to PWM
	 */
	arm_pid_init_f32(&globalPidLeft, 1);
	arm_pid_init_f32(&globalPidRight, 1);
#endif

	enableMotors(ENABLE);

	while(1) {
		/*
		 * Receive from the queue, if it is empty then nothing is saved nowhere
		 * If smth was really received it should be checked.
		 * Limit max absolute value preserving original sign
		 */
		if (xQueueReceive(motorCtrlQueue, &motorSpeed, 0) == pdTRUE) {
			float max = fmaxf(fabsf(motorSpeed.LeftSpeed), fabsf(motorSpeed.RightSpeed));
			if (max > maxSpeedAllowed) {
				motorSpeed.LeftSpeed  *= maxSpeedAllowed / max;
				motorSpeed.RightSpeed *= maxSpeedAllowed / max;
			}
			if (globalLogSpeed) safePrint(34, "Ordered speeds: L:%.2f R:%.2f\n", motorSpeed.LeftSpeed, motorSpeed.RightSpeed);
		}

		/* Read encoders and compute difference in readings. Speeds will be calculated after odometry part */
		posLeft = getEncoderL();
		posRight = getEncoderR();
		speedLeft = (float)(posLeft - prevPosLeft);
		speedRight = (float)(posRight - prevPosRight);

		/* Compute telemetry update */
		getTelemetry(&telemetryData);
		float deltaS = (speedRight + speedLeft) * IMPS_TO_MM_TRAVELED / 2.0f;
		telemetryUpdate.dO = (speedRight - speedLeft) * IMPS_TO_MM_TRAVELED / ROBOT_DIAM;
		telemetryUpdate.dX = deltaS * cosf(telemetryData.O);
		telemetryUpdate.dY = deltaS * sinf(telemetryData.O);
		/*if (speedLeft != speedRight) {
			float a = ROBOT_DIAM / 2.0f * (speedRight + speedLeft) / (speedRight - speedLeft);
			telemetryUpdate.dO = (speedRight - speedLeft) * IMPS_TO_MM_TRAVELED / ROBOT_DIAM;
			telemetryUpdate.dX = a*(sinf(telemetryUpdate.dO + telemetryData.O) - sinf(telemetryData.O));
			telemetryUpdate.dY = -a*(cosf(telemetryUpdate.dO + telemetryData.O) - cosf(telemetryData.O));
		}
		else {
			float deltaS = (speedRight + speedLeft) * IMPS_TO_MM_TRAVELED / 2.0f;
			telemetryUpdate.dX = deltaS * cosf(telemetryData.O);
			telemetryUpdate.dY = deltaS * sinf(telemetryData.O);
			telemetryUpdate.dO = 0.0f;
		}*/
		if (xQueueSendToBack(telemetryQueue, &telemetryUpdate, 0) == errQUEUE_FULL) {
			if (globalLogEvents) safePrint(25, "Telemetry queue full!\n");
		}

		/* Compute speeds in rad/s */
		speedLeft *= IMPS_TO_RAD*1000.0f/(float)(delayMsPerPeriod/portTICK_RATE_MS);
		speedRight *= IMPS_TO_RAD*1000.0f/(float)(delayMsPerPeriod/portTICK_RATE_MS);

		if (globalLogSpeed && globalLogSpeedCounter > 0) { // mod sth to allow changing log period
			safePrint(42, "Speeds: L:%.4frad/s R:%.4frad/s\n", speedLeft, speedRight);
			globalLogSpeedCounter--;
		}

		/* If regulator is on; critical section is to ensure that regulator is not switched after global... is checked */
		taskENTER_CRITICAL();
		{
			if (globalSpeedRegulatorOn) {
				/* Compute error by simple substraction */
				errorLeft = -(speedLeft - motorSpeed.LeftSpeed);
				errorRight = -(speedRight - motorSpeed.RightSpeed);

#ifdef USE_CUSTOM_MOTOR_CONTROLLER
				/* Use Ferdek's controllers */
				float voltage = 8.0f; //calculations were made for voltage normalized to 8V
				if(globalControllerVoltageCorrection) voltage = getAvgBatteryVoltage();
				outLeft = motorController(motorSpeed.LeftSpeed, errorLeft, voltage, &globalLeftMotorParams);
				outRight = motorController(motorSpeed.RightSpeed, errorRight, voltage, &globalRightMotorParams);
#else
				/* Invoke PID functions and compute output speed values, minus is necessary for PID */
				outLeft = arm_pid_f32(&globalPidLeft, errorLeft);
				outRight = arm_pid_f32(&globalPidRight, errorRight);
#endif

				/* Set motors speed; minus is necessary to drive in the right direction */
				if (motorSpeed.LeftSpeed == 0.0f && fabsf(errorLeft) < 0.001f) {
					setMotorLBrake();
#ifndef USE_CUSTOM_MOTOR_CONTROLLER
					arm_pid_reset_f32(&globalPidLeft);
#endif
				}
				else setMotorLSpeed(outLeft);

				//TODO: if motors' speeds are set to 0, change mode to servo-controller
				if (motorSpeed.RightSpeed == 0.0f && fabsf(errorRight) < 0.001f) {
					setMotorRBrake();
#ifndef USE_CUSTOM_MOTOR_CONTROLLER
					arm_pid_reset_f32(&globalPidRight);
#endif
				}
				else setMotorRSpeed(outRight);
			}
			else {
				// if regulator was turned off, keep target speed at 0 so that motors will break after regulator is on again
				motorSpeed.LeftSpeed = motorSpeed.RightSpeed = 0.0f;
			}
		}
		taskEXIT_CRITICAL();

		/* Update current encoders position for next loop pass */
		prevPosLeft = posLeft;
		prevPosRight = posRight;

		/* Wait for next sampling period */
		vTaskDelayUntil(&wakeTime, delayMsPerPeriod/portTICK_RATE_MS);
	}
}

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

void TaskRC5(void * p) {
	/* Timer callback function to switch LED off */
	void rc5LEDOff(xTimerHandle xTimer) {
		lightLED(2, OFF);
	}
	/* Timer for switching led off */
	xTimerHandle rc5LEDTimer = xTimerCreate(NULL, 200/portTICK_RATE_MS, pdFALSE, NULL, rc5LEDOff);

	/* Initialize RC5 hardware */
	RC5_Receiver_Init();
	/* Initial semaphore take so it can be given and waited for */
	xSemaphoreTake(rc5CommandReadySemaphore, 0);

	MotorSpeed_Struct motorSpeed;
	RC5Frame_TypeDef frame;
	uint8_t toggle = 2;
	float maxSpeed = 3.0f;

	while(1) {
		/* Blocking wait - suspend task till semaphore is set */
		xSemaphoreTake(rc5CommandReadySemaphore, portMAX_DELAY);
		/* Decode command from external API */
		RC5_Decode(&frame);
		/* If this is a unique press (ignore continuous pressing) */
		if (toggle != frame.ToggleBit) {
			switch(frame.Command) {
			case 1: /*<< 1 button */
				motorSpeed.LeftSpeed = maxSpeed * 0.5f;
				motorSpeed.RightSpeed = maxSpeed;
				xQueueSendToBack(motorCtrlQueue, &motorSpeed, portMAX_DELAY);
				break;
			case 2: /*<< 2 button */
				motorSpeed.LeftSpeed = motorSpeed.RightSpeed = maxSpeed;
				xQueueSendToBack(motorCtrlQueue, &motorSpeed, portMAX_DELAY);
				break;
			case 3: /*<< 3 button */
				motorSpeed.LeftSpeed = maxSpeed;
				motorSpeed.RightSpeed = maxSpeed * 0.5f;
				xQueueSendToBack(motorCtrlQueue, &motorSpeed, portMAX_DELAY);
				break;
			case 4: /*<< 4 button */
				motorSpeed.LeftSpeed = -maxSpeed * 0.5f;
				motorSpeed.RightSpeed = maxSpeed * 0.5f;
				xQueueSendToBack(motorCtrlQueue, &motorSpeed, portMAX_DELAY);
				break;
			case 5: /*<< 5 button */
				motorSpeed.LeftSpeed = .0f;
				motorSpeed.RightSpeed = .0f;
				xQueueSendToBack(motorCtrlQueue, &motorSpeed, portMAX_DELAY);
				break;
			case 6: /*<< 6 button */
				motorSpeed.LeftSpeed = maxSpeed * 0.5f;
				motorSpeed.RightSpeed = -maxSpeed * 0.5f;
				xQueueSendToBack(motorCtrlQueue, &motorSpeed, portMAX_DELAY);
				break;
			case 7: /*<< 7 button */
				motorSpeed.LeftSpeed = -maxSpeed * 0.5f;
				motorSpeed.RightSpeed = -maxSpeed;
				xQueueSendToBack(motorCtrlQueue, &motorSpeed, portMAX_DELAY);
				break;
			case 8: /*<< 8 button */
				motorSpeed.LeftSpeed = motorSpeed.RightSpeed = -maxSpeed;
				xQueueSendToBack(motorCtrlQueue, &motorSpeed, portMAX_DELAY);
				break;
			case 9: /*<< 9 button */
				motorSpeed.LeftSpeed = -maxSpeed;
				motorSpeed.RightSpeed = -maxSpeed * 0.5f;
				xQueueSendToBack(motorCtrlQueue, &motorSpeed, portMAX_DELAY);
				break;
			case 12: /*<< OFF red button */
				/* Set too low preload value causing reset to occur */
				IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
				IWDG_SetReload(1);
				break;
			case 16: /*<< VOL up button */
				maxSpeed *= 1.2f;
				break;
			case 17: /*<< VOL down button */
				maxSpeed /= 1.2f;
				break;
			case 32: /*<< CH up button */
				setPenUp();
				break;
			case 33: /*<< CH down button */
				setPenDown();
				break;
#ifdef USE_IMU_TELEMETRY
			case 42: /*<< clock button */
				xSemaphoreGive(imuMagScalingReq);
				break;
#endif
			case 43: /*<< screen button above purple button */
				setWiFiMode(getWiFiMode() == WiFiMode_Data ? WiFiMode_Command : WiFiMode_Data);
				break;
			case 52: /*<< purple button */
				enableLantern(!getLanternState());
				break;
			}

			if (globalLogEvents) safePrint(10, "RC5: %d\n", frame.Command);
			/* Light LED and reset software timer; when it expires the LED will be turned off */
			lightLED(2, ON);
			xTimerReset(rc5LEDTimer, 0);
		}
		toggle = frame.ToggleBit;
	}
}

#ifdef USE_IMU_TELEMETRY
void TaskIMUMagScaling(void *p) {
	xSemaphoreTake(imuMagScalingReq, 0);	// initial take

	uint8_t taken = xSemaphoreTake(imuMagScalingReq, 5000/portTICK_RATE_MS);	// wait for max 5s for request
	if (taken == pdFALSE) goto finish;
	// ok, there was a request. Check if robot moved

	TelemetryData_Struct telemetry;
	getTelemetryRaw(&telemetry);
	if (fabsf(telemetry.X) > 0.1f || fabsf(telemetry.Y) > 0.1f || fabsf(telemetry.O) > 0.01f) goto finish;
	// ok, robot is not moving, try to do scaling

#ifdef FOLLOW_TRAJECTORY
	vTaskSuspend(trajectoryTask);
#else
	vTaskSuspend(driveTask);
#endif

	magnetometerScalingQueue = xQueueCreate(10,	sizeof(float));
	globalMagnetometerScalingInProgress = ENABLE;

	float imuAngle;
	MotorSpeed_Struct motorsSpeed;

	taken = xQueueReceive(magnetometerScalingQueue, &imuAngle, 5000/portTICK_RATE_MS);
	if (taken != pdFALSE) { // something really came in, doing scaling
		safePrint(23, "Scaling magnetometer\n");

		motorsSpeed.LeftSpeed = -0.3f;
		motorsSpeed.RightSpeed = 0.3f;
		xQueueSendToBack(motorCtrlQueue, &motorsSpeed, portMAX_DELAY);

		// turning around, save all reading data in orientation intervals
		uint16_t i = 0;
		while(telemetry.O < 2*M_PI) {
			do {
				getTelemetryRaw(&telemetry);
				taken = xQueueReceive(magnetometerScalingQueue, &imuAngle, 0);	// read everything as soon as possible
			} while (telemetry.O < globalMagnetometerImprov.xSpacing * i);

			if (i >= 360) imuAngle -= 2.0f*M_PI;
			globalMagnetometerImprovData[(i+360)%720] = imuAngle;
			i++;
		}

		globalMagnetometerImprovData[720] = globalMagnetometerImprovData[0] + 2.0f*M_PI;

		motorsSpeed.LeftSpeed = motorsSpeed.RightSpeed = 0.0f;
		xQueueSendToBack(motorCtrlQueue, &motorsSpeed, portMAX_DELAY);
	}

#ifdef FOLLOW_TRAJECTORY
	vTaskResume(trajectoryTask);
#else
	vTaskResume(driveTask);
#endif

	globalMagnetometerScalingInProgress = DISABLE;
	vTaskDelay(500/portTICK_RATE_MS);				// delay to make sure other tasks know that global flag is DISABLED
	vQueueDelete(magnetometerScalingQueue);
	magnetometerScalingQueue = NULL;
finish:
	globalMagnetometerScalingInProgress = DISABLE; 	// just to be sure it is disabled (if not done so by default at initialization)
	vTaskDelete(NULL);								// delete this task
}
#endif

#ifdef USE_IMU_TELEMETRY
void TaskIMU(void * p) {
	/* If reset needed then do reset, else setup I2C*/
	if (globalIMUHang) {
		/* Turn off interrupts from IMU */
		NVIC_InitTypeDef NVIC_InitStructure;
		NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannel = IMU_I2C_EVENT_IRQn;
		NVIC_Init(&NVIC_InitStructure);

		I2C_SoftwareResetCmd(IMU_I2C, ENABLE);
		/* Reset I2C - turn off the clock*/
		IMU_I2C_CLOCK_FUN(IMU_I2C_CLOCK, DISABLE);

		// Init SCL to allow bit-bang clocking
		GPIO_InitTypeDef GPIO_InitStructure;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Pin = IMU_GPIO_SCL_PIN;
		GPIO_Init(IMU_GPIO, &GPIO_InitStructure);

		for (uint8_t i = 0; i<40; i++) {
			vTaskDelay(1/portTICK_RATE_MS);
			taskENTER_CRITICAL();
			{
				GPIO_ToggleBits(IMU_GPIO, IMU_GPIO_SCL_PIN);
			}
			taskEXIT_CRITICAL();
		}

		// default-initialize I2C in case it is not after clock is turned off
		I2C_DeInit(IMU_I2C);
		// init I2C once again
		initI2CforIMU();

		globalIMUHang = false;
	}
	else {
		initI2CforIMU();
	}

	VectorF read; 								// one reading from I2C, temporary variable
	VectorF accSum = {0.0f, 0.0f, 0.0f};		// place for 4 readings from accelerometer
	VectorF magSum = {0.0f, 0.0f, 0.0f};		// place for 3 readings from magnetometer
	VectorF gyroSum = {0.0f, 0.0f, 0.0f};		// place for 4 readings from gyro
	const VectorF front = {0.0f, -1.0f, 0.0f};	// front of the robot relative to IMU mounting
	float estDir, angle, cangle, prev_angle;
	Complementary_State cState;					// filter state
	int32_t turn_counter = 0;
	TelemetryData_Struct telemetry;

	uint8_t samplingState = 7;					// state machine's state
	ComplementaryInit(&cState, 0.93f); // 0.5s time constant with 25Hz sampling

	/* Take semaphores initially, so that they can be given later */
	xSemaphoreTake(imuPrintRequest, 0);
	xSemaphoreTake(imuGyroReady, 0);
	xSemaphoreTake(imuAccReady, 0);
	xSemaphoreTake(imuMagReady, 0);
	xSemaphoreTake(imuI2CEV, 0);

	/* Reset watchdog timer to 0 effectively starting it and init all IMU modules; then stop timer */
	xTimerReset(imuWatchdogTimer, 0);
	InitAcc();
	InitMag();
	InitGyro();
	xTimerStop(imuWatchdogTimer, 0);

	/* If timer was successfully stopped then all units must have been correctly initialized, so I2C
	 * works good and IMU task can get higher priority */
	vTaskPrioritySet(imuTask, PRIORITY_TASK_IMU);

	/* Wait a while until IMU stabilizes, TODO calibration here */
	vTaskDelay(2000/portTICK_RATE_MS);

	TelemetryUpdate_Struct update = {.dX = 0.0f, .dY = 0.0f, .Source = TelemetryUpdate_Source_IMU};

	portTickType wakeTime = xTaskGetTickCount();

	while (1) {
		switch (samplingState) {
		case 0: // all data sampled in this common point
			VectorScale(&accSum, 0.25f, &accSum);
			VectorScale(&magSum, 1.0f / 3.0f, &magSum);
			VectorScale(&gyroSum, 0.25f, &gyroSum);

			estDir -= gyroSum.z * 0.04f;

			angle = GetHeading(&accSum, &magSum, &front);

			if (globalMagnetometerScalingInProgress != ENABLE)
				angle = normalizeOrientation(arm_linear_interp_f32(&globalMagnetometerImprov, angle));

			// if abs angle > 90 deg and angle sign changes
			if (fabsf(angle) > M_PI/2.0f && angle*prev_angle < 0.0f) {
				if (angle > 0.0f) { // switching from -180 -> 180
					turn_counter--;
				}
				else {
					turn_counter++;
				}
			}
			prev_angle = angle;

			angle += 2.0f * M_PI * (float)turn_counter;

			if (globalMagnetometerScalingInProgress == DISABLE) {
				cangle = ComplementaryGet(&cState, cangle - gyroSum.z * 0.04f, angle);
				update.dO = cangle;
				if (xQueueSendToBack(telemetryQueue, &update, 0) == errQUEUE_FULL) {
					if (globalLogEvents) safePrint(25, "Telemetry queue full!\n");
				}
			}
			else {
				xQueueSendToBack(magnetometerScalingQueue, &angle, 0); // this queue should exist if globalMagnetometerScaling == ENABLE
			}
//			getTelemetry(&telemetry);
//			safePrint(55, "Mag: %.1f Gyro: %.1f Comp: %.1f Odo: %.1f\n", angle / DEGREES_TO_RAD, estDir / DEGREES_TO_RAD, cangle / DEGREES_TO_RAD, telemetry.O / DEGREES_TO_RAD);

			VectorSet(&gyroSum, 0.0f);
			VectorSet(&magSum, 0.0f);
			VectorSet(&accSum, 0.0f);
			vTaskDelayUntil(&wakeTime, 10/portTICK_RATE_MS);
			samplingState++;
			break;
		case 1:	// first point of 100Hz
		case 3: // second point of 100Hz
		case 5: // third point of 100Hz
			xTimerReset(imuWatchdogTimer, 0);
			ReadGyroScaled(&read);
			xTimerStop(imuWatchdogTimer, 0);
			VectorAdd(&gyroSum, &read, &gyroSum);

			xTimerReset(imuWatchdogTimer, 0);
			ReadAccScaled(&read);
			xTimerStop(imuWatchdogTimer, 0);
			VectorAdd(&accSum, &read, &accSum);

			if (samplingState == 1) vTaskDelayUntil(&wakeTime, 4/portTICK_RATE_MS);
			else if (samplingState == 3) vTaskDelayUntil(&wakeTime, 7/portTICK_RATE_MS);
			else /* 5 */ vTaskDelayUntil(&wakeTime, 10/portTICK_RATE_MS);
			samplingState++;
			break;
		case 2: // first point of 75Hz
		case 4: // second point of 75Hz
			xTimerReset(imuWatchdogTimer, 0);
			ReadMagScaled(&read);
			xTimerStop(imuWatchdogTimer, 0);
			VectorAdd(&magSum, &read, &magSum);

			if (samplingState == 2) vTaskDelayUntil(&wakeTime, 6/portTICK_RATE_MS);
			else /* 4 */ vTaskDelayUntil(&wakeTime, 3/portTICK_RATE_MS);
			samplingState++;
			break;
		case 6: // common point: fourth of 100Hz and third of 75Hz
		case 7: // prepare - enter state
			xTimerReset(imuWatchdogTimer, 0);
			ReadGyroScaled(&read);
			xTimerStop(imuWatchdogTimer, 0);
			VectorAdd(&gyroSum, &read, &gyroSum);

			xTimerReset(imuWatchdogTimer, 0);
			ReadAccScaled(&read);
			xTimerStop(imuWatchdogTimer, 0);
			VectorAdd(&accSum, &read, &accSum);

			xTimerReset(imuWatchdogTimer, 0);
			ReadMagScaled(&read);
			xTimerStop(imuWatchdogTimer, 0);
			VectorAdd(&magSum, &read, &magSum);

			if (samplingState == 7) { 						// initialize all values to first reading
				VectorScale(&accSum, 4.0f, &accSum);		// will be scaled back at state 0
				VectorScale(&magSum, 3.0f, &magSum);
				VectorScale(&gyroSum, 4.0f, &gyroSum);
				cangle = GetHeading(&accSum, &magSum, &front); // initial heading
				getTelemetry(&telemetry);
				initMagnetometerImprovInstance(cangle - telemetry.O);  // scale to be 0 at initial
				cangle = telemetry.O;								   // linear interpolation in this point gives 0
				prev_angle = cangle;
				estDir = cangle;
			}

			samplingState = 0;
			break;
		}
	}
}
#endif /* USE_IMU_TELEMETRY */

#ifdef USE_IMU_TELEMETRY
void imuWatchdogOverrun(xTimerHandle xTimer) {
	vTaskDelete(imuTask);
	if (globalLogEvents) safePrint(11, "IMU hang!\n");

	// indicate reset need
	globalIMUHang = true;

	// create IMU task with the lowest priority
	xTaskCreate(TaskIMU, NULL, 400, NULL, 0, &imuTask);
}
#endif /* USE_IMU_TELEMETRY */

void TaskPrintfConsumer(void * p) {
	char *msg;

	xSemaphoreTake(comUSARTTCSemaphore, 0);
	xSemaphoreTake(comDMATCSemaphore, 0);
	xSemaphoreTake(wifiUSARTTCSemaphore, 0);
	xSemaphoreTake(wifiDMATCSemaphore, 0);

	while(1) {
		/* Turn off printing if bridge is on */
		if (getWiFi2USBBridgeStatus() == ON) {
			vTaskDelay(100/portTICK_RATE_MS);
			continue;
		}

		/* Wait for new message and take it, process any incomming message as quickly as possible */
		xQueueReceive(printfQueue, &msg, portMAX_DELAY);

		/* Set start address at the beginning of the message */
		COM_TX_DMA_STREAM->M0AR = (uint32_t)msg; // base addr
		WIFI_TX_DMA_STREAM->M0AR = (uint32_t)msg;
		/* Set number of bytes to send */
		size_t len = strlen(msg);
		COM_TX_DMA_STREAM->NDTR = len;   // size in bytes
		WIFI_TX_DMA_STREAM->NDTR = len;

		/* Save USB and WiFi status, will be used in two different places and need to be the same */
		OnOff USBS = getUSBStatus();
		OnOff WIFIS = getWiFiStatus();

		/* If USB is enabled, then start DMA transfer */
		if (USBS == ON) {
			/* Enable stream */
			DMA_Cmd(COM_TX_DMA_STREAM, ENABLE);
			/* Enable DMA in USART */
			USART_DMACmd(COM_USART, USART_DMAReq_Tx, ENABLE);
			/* Enabling interrupt from Transmission Complete flag.
			 * Here, because it it set constantly when USART is idle.
			 * Disabled in ISR. */
			USART_ITConfig(COM_USART, USART_IT_TC, ENABLE);
		}
		/* The same for WiFi */
		if (WIFIS == ON) {
			DMA_Cmd(WIFI_TX_DMA_STREAM, ENABLE);
			USART_DMACmd(WIFI_USART, USART_DMAReq_Tx, ENABLE);
			USART_ITConfig(WIFI_USART, USART_IT_TC, ENABLE);
		}

		/* Waiting for transmission complete. One interrupt from DMA and one from USART. Disabling stream at the end */
		if (USBS == ON) {
			xSemaphoreTake(comUSARTTCSemaphore, portMAX_DELAY);
			xSemaphoreTake(comDMATCSemaphore, portMAX_DELAY);
			DMA_Cmd(COM_TX_DMA_STREAM, DISABLE);
		}
		if (WIFIS == ON) {
			xSemaphoreTake(wifiUSARTTCSemaphore, portMAX_DELAY);
			xSemaphoreTake(wifiDMATCSemaphore, portMAX_DELAY);
			DMA_Cmd(WIFI_TX_DMA_STREAM, DISABLE);
		}

		/* Free memory allocated for message */
		vPortFree(msg);
	}
}

void TaskCommandHandler(void * p) {
	char * msg;
	while(1) {
		/* Block till message is available */
		xQueueReceive(commandQueue, &msg, portMAX_DELAY);
		/* Handle message */
		COMHandle(msg);
		/* Free allocated resources */
		vPortFree(msg);
	}
}

void TaskLED(void * p) {
	IWDG_Enable();
	while(1) {
		GPIO_ToggleBits(LEDS_GPIO_26, LEDS_GPIO_3_PIN);
		/* Every 100 ms */
		for (uint8_t t = 0; t < 5; t++) {
			IWDG_ReloadCounter();
			vTaskDelay(100 / portTICK_RATE_MS);
		}
	}
}

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

void COMHandle(const char * command) {
#ifndef FOLLOW_TRAJECTORY
	DriveCommand_Struct *dc;
#endif
	char *last;
	MotorSpeed_Struct ms;
	TelemetryData_Struct td;
	float temp_float;
#ifdef USE_CUSTOM_MOTOR_CONTROLLER
	MotorControllerParameters_Struct* ptrParams;
#endif

	const char wrongComm[] = "Incorrect command\n";

	bool commandCheck(const bool p) {
		if (p != true) {
			if (globalLogEvents) safePrint(20, "%s", wrongComm);
		}
		return p;
	}

	switch(command[0]) {
	case LOW_LEVEL_AUA:
		GPIO_ToggleBits(LEDS_GPIO_1, LEDS_GPIO_1_PIN);
		break;
	case HIGH_LEVEL_AUA:
		safePrint(8, "Hello!\n");
		break;
	case AVAILABLE_MEMORY:
		safePrint(32, "Available memory: %dkB\n", xPortGetFreeHeapSize());
		break;
#ifdef FOLLOW_TRAJECTORY
	case IMPORT_TRAJECTORY_POINTS:
		if (commandCheck( strlen(command) >= 3) ) {
			safePrint(36, "<#Please send only %d points#>\n",
				TBloadNewPoints(strtol((char*)&command[2], NULL, 10)));
		}
		break;
#endif
	case BATTERY_VOLTAGE:
		safePrint(26, "Battery voltage: %.2fV\n", getBatteryVoltage());
		break;
	case PEN_DOWN:
		setPenDown();
		break;
	case PEN_UP:
		setPenUp();
		break;
#ifndef FOLLOW_TRAJECTORY
	case DRIVE_COMMAND:
		if (commandCheck( strlen(command) >= 11) ) {
			dc = (DriveCommand_Struct*)pvPortMalloc(sizeof(DriveCommand_Struct));
			dc->Type = command[2];
			dc->UsePen = (command[4] == '0' ? 0 : 1);
			dc->Speed = strtof((char*)&command[6], &last);
			dc->Param1 = strtof(last+1, &last);
			/* Only 'line' has one parameter */
			if (dc->Type != DriveCommand_Type_Line) {
				dc->Param2 = strtof(last+1, NULL);
			}
			xQueueSendToBack(driveQueue, &dc, portMAX_DELAY);
		}
		break;
#endif
	case MOTOR_LEFT_SPEED:
		if (commandCheck( strlen(command) >= 3 )) {
			globalSpeedRegulatorOn = DISABLE;
			setMotorLSpeed(strtof((char*)&command[2], NULL)/100.0f);
		}
		break;
	case MOTOR_RIGHT_SPEED:
		if (commandCheck( strlen(command) >= 3 )) {
			globalSpeedRegulatorOn = DISABLE;
			setMotorRSpeed(strtof((char*)&command[2], NULL)/100.0f);
		}
		break;
	case MOTOR_LEFT_ENCODER:
		safePrint(22, "Encoder left: %ld\n", getEncoderL());
		break;
	case MOTOR_RIGHT_ENCODER:
		safePrint(23, "Encoder right: %ld\n", getEncoderR());
		break;
	case MOTORS_ENABLE:
		if (commandCheck( strlen(command) >= 3 )) {
			if (command[2] == '0') {
				globalSpeedRegulatorOn = DISABLE;
				enableMotors(DISABLE);
				safePrint(19, "Motors disabled\n");
			}
			else {
				enableMotors(ENABLE);
				globalSpeedRegulatorOn = ENABLE;
				safePrint(18, "Motors enabled\n");
			}
		}
		break;
	case MOTORS_SET_SPEEDS:
		if (commandCheck( strlen(command) >= 3 )) {
			globalSpeedRegulatorOn = ENABLE;
			ms.LeftSpeed = strtof((char*)&command[2], &last);
			ms.RightSpeed = strtof(last+1, NULL);
			xQueueSendToBack(motorCtrlQueue, &ms, portMAX_DELAY);
		}
		break;
	case TELEMETRY_PRINT:
		getTelemetry(&td);
		safePrint(40, "X:%.2f Y:%.2f O:%.1f\n", td.X, td.Y, td.O / DEGREES_TO_RAD);
		break;
	case LOGGING_COMMAND:
		if (commandCheck( strlen(command) >= 5 )) {
			switch(command[2]) {
			case Logging_Type_Telemetry:
				globalLogTelemetry = (command[4] == '1');
				break;
			case Logging_Type_Speed:
				if (command[4] == '1') {
					taskENTER_CRITICAL();
					globalLogSpeed = ENABLE;
					globalLogSpeedCounter = 1<<31;
					taskEXIT_CRITICAL();
				}
				else {
					globalLogSpeed = DISABLE;
				}
				break;
			case Logging_Type_Events:
				globalLogEvents = (command[4] == '1');
				break;
			default:
				safePrint(17, wrongComm);
			}
		}
		break;
	case CPU_RESET:
		IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
		IWDG_SetReload(1);
		break;
	case WIFI_MODE:
		if (commandCheck( strlen(command) >= 3 )) {
			setWiFiMode(command[2] == 'c' ? WiFiMode_Command : WiFiMode_Data);
		}
		break;
	case WIFI_RESET:
		if (commandCheck( strlen(command) >= 3 )) {
			setWiFiReset(command[2] == '1' ? ENABLE : DISABLE);
		}
		break;
	case LANTERN_ENABLE:
		if (commandCheck( strlen(command) >= 3 )) {
			enableLantern(command[2] == '1' ? ENABLE : DISABLE);
		}
		break;
	case SPEED_REGULATOR_ENABLE:
		if (commandCheck( strlen(command) >= 3 )) {
			taskENTER_CRITICAL();
			{
				globalSpeedRegulatorOn = (command[2] != '0');
				setMotorLSpeed(0.0f);
				setMotorRSpeed(0.0f);
			}
			taskEXIT_CRITICAL();
		}
		break;
	case MOTORS_CHARACTERISTIC:
		if (commandCheck( strlen(command) >= 7 && (command[2] == 'l' || command[2] == 'r') )) {
			taskENTER_CRITICAL();
			{
				globalSpeedRegulatorOn = DISABLE;
				globalLogSpeed = ENABLE;
				temp_float = strtof((char*)&command[4], &last) / 100.0f;
				globalLogSpeedCounter = strtol(last+1, NULL, 10);
				command[2] == 'l' ? setMotorLSpeed(temp_float) : setMotorRSpeed(temp_float);
			}
			taskEXIT_CRITICAL();
		}
		break;
	case MOTORS_CHARACTERISTIC2:
		if (commandCheck( strlen(command) >= 7 )) {
			taskENTER_CRITICAL();
			{
				globalSpeedRegulatorOn = DISABLE;
				globalLogSpeed = ENABLE;
				setMotorLSpeed(strtof((char*)&command[4], &last) / 100.0f);
				setMotorRSpeed(strtof(last+1, &last) / 100.0f);
				globalLogSpeedCounter = strtol(last+1, NULL, 10);
			}
			taskEXIT_CRITICAL();
		}
		break;
	case DELAY_COMMANDS:
		if (commandCheck( strlen(command) >= 3) ) {
			vTaskDelay(strtol((char*)&command[2], NULL, 10) / portTICK_RATE_MS);
		}
		break;
	case CPU_USAGE:
		safePrint(19, "CPU Usage: %.1f%%\n", globalCPUUsage*100.0f);
		break;
	case STACK_USAGE:
		reportStackUsage();
		break;
	case '$':
		temp_float = strtof((char*)&command[2], NULL);
		safePrint(60, "Float %f: %x %x %x %x\n", temp_float, *((uint8_t*)(&temp_float)), *(((uint8_t*)(&temp_float)+1)), *(((uint8_t*)(&temp_float)+2)), *(((uint8_t*)(&temp_float))+3));
		break;
#ifdef USE_CUSTOM_MOTOR_CONTROLLER
	case SPEER_REGULATOR_VOLTAGE_CORRECTION:
		if (commandCheck( strlen(command) >= 3 )) {
			globalControllerVoltageCorrection = (command[2] != '0');
		}
		break;
	case SPEED_REGULATOR_CUSTOM_PARAMS:
		if (commandCheck (strlen(command) >= 19 && (command[2] == 'l' || command[2] == 'r') )) {
			if (command[2] == 'l') ptrParams = &globalLeftMotorParams;
			else ptrParams = &globalRightMotorParams;
			taskENTER_CRITICAL();
			{
				ptrParams->threshold = strtof((char*)&command[4], &last);
				ptrParams->A = strtof(last+1, &last);
				ptrParams->B = strtof(last+1, &last);
				ptrParams->C = strtof(last+1, &last);
				ptrParams->KP = strtof(last+1, &last);
				ptrParams->A_t = strtof(last+1, &last);
				ptrParams->B_t = strtof(last+1, &last);
				ptrParams->KP_t = strtof(last+1, &last);
			}
			taskEXIT_CRITICAL();
		}
		break;
#else
	case SPEED_REGULATOR_PID_PARAMS:
		if(commandCheck (strlen(command) >= 7) ) {
			taskENTER_CRITICAL();
			{
				globalPidLeft.Kp = globalPidRight.Kp = strtof((char*)&command[2], &last);
				globalPidLeft.Ki = globalPidRight.Ki = strtof(last+1, &last);
				globalPidLeft.Kd = globalPidRight.Kd = strtof(last+1, &last);
			}
			taskEXIT_CRITICAL();
		}
		break;
#endif
	default:
		if (globalLogEvents) safePrint(18, "No such command\n");
		break;
	}
}

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

#ifdef USE_IMU_TELEMETRY
void initI2CforIMU(void) {
	I2C_InitTypeDef I2C_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Configuring I2C to work with INU */
	/* Enabling timer for GPIOs */
	RCC_AHB1PeriphClockCmd(IMU_GPIO_CLOCK, ENABLE);
	/* Redirecting GPIOs to alternative function */
	GPIO_PinAFConfig(IMU_GPIO, IMU_GPIO_SCL_PINSOURCE, IMU_AF);
	GPIO_PinAFConfig(IMU_GPIO, IMU_GPIO_SDA_PINSOURCE, IMU_AF);
	/* Configuring GPIOs */
	GPIO_InitStructure.GPIO_Pin = IMU_GPIO_SCL_PIN | IMU_GPIO_SDA_PIN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(IMU_GPIO, &GPIO_InitStructure);
	/* Enabling I2C clock */
	IMU_I2C_CLOCK_FUN(IMU_I2C_CLOCK, ENABLE);
	/* Initializing I2C  */
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = IMU_I2C_SPEED;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_OwnAddress1 = 0;
	/* Enabling clock stretching feature for greater stability */
	I2C_StretchClockCmd(IMU_I2C, ENABLE);
	/*
	 * EXTI interrupts moved to appr. Init() function in imu.c
	 * This will be done only if imu task gets started
	 */
	/* Interrupt for I2C event */
	NVIC_InitStructure.NVIC_IRQChannel = IMU_I2C_EVENT_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PRIORITY_ISR_IMUI2CEVENT;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_Init(&NVIC_InitStructure);
	/* Enable I2C */
	I2C_Init(IMU_I2C, &I2C_InitStructure);
}
#endif /* USE_IMU_TELEMETRY */

#ifdef USE_IMU_TELEMETRY
void initMagnetometerImprovInstance(float x0) {
	globalMagnetometerImprov.x1 = -M_PI;
	float offset = -M_PI - x0;

	for (uint16_t i = 0; i<globalMagnetometerImprov.nValues; i++) {
		globalMagnetometerImprovData[i] = globalMagnetometerImprov.xSpacing * i + offset; // Must be continuous, no normalization. Normalize afterwards.
	}
}
#endif /* USE_IMU_TELEMETRY */

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
