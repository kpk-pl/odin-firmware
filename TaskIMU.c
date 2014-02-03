#include <stdbool.h>

#include <stm32f4xx.h>
#include "arm_math.h"

#include "TaskIMU.h"

#include "main.h"
#include "priorities.h"
#include "stackSpace.h"
#include "hardware.h"

#include "imu.h"				// reading IMU data
#include "vector.h"				// operations on 3D vectors

#ifdef USE_GYRO_FOR_IMU
#include "complementary.h"		// complementary filter
#endif

#include "TaskTelemetry.h"		// for typedefs
#include "TaskIMUScaling.h"		// for external declarations
#include "TaskPrintfConsumer.h"	// for printf

/**
 *  \brief Sets I2C peripheral for use with magnetometer and gyro sensors
 *
 *  It configures ports and peripherals for I2C use.
 */
static void initI2CforIMU(void);

/**
 * \brief Resets I2C peripheral after I2C hang
 * It turns the peripheral off, disconnects ports, toggles pins several times to reset devices on line and turns I2C peripheral back on
 */
static inline void resetI2C(void);

/**
 * \brief Performs interpolation and normalization of the angle read from magnetometer
 * @param Input angle
 * @return Interpolated angle
 */
static float interpolateAngle(float angle);

/**
 *  \brief Handler for timer overflow when I2C hangs up
 *  @param xTimer Handle to timer which overflowed
 */
void imuWatchdogOverrun(xTimerHandle xTimer);

static volatile bool globalIMUHang = false;								/*!< Indicates if IMU hanged lately */
volatile float globalMagnetometerImprovData[MAG_IMPROV_DATA_POINTS+1];	/*!< Array of scaling points used by interpolation to correct magnetometer readings */
volatile float globalMagStartOrientation = NAN;							/*!< Start orientation when odometry reports 0 degrees */
#ifdef USE_GYRO_FOR_IMU
volatile float globalGyroDrift = NAN;									/*!< Drift scaled by IMU Scaling task */
#endif

xQueueHandle I2CEVFlagQueue = NULL;		/*!< Queue for I2C events synchronization and handling in interrupts to avoid busy-waits */
xSemaphoreHandle imuI2CEV;				/*!< Semaphore for I2C successful event recognition */
xTaskHandle imuTask;					/*!< Handler for this task */
xTimerHandle imuWatchdogTimer;			/*!< Timer for catching IMU hangs */

void TaskIMU(void * p) {
	/* If reset needed then do reset, else setup I2C*/
	if (globalIMUHang) {
		resetI2C();
		globalIMUHang = false;
	}

	initI2CforIMU();

	VectorF read; 								// one reading from I2C, temporary variable
	VectorF accSum = {0.0f, 0.0f, 0.0f};		// place for 4 readings from accelerometer
	VectorF magSum = {0.0f, 0.0f, 0.0f};		// place for 3 readings from magnetometer
	const VectorF front = {0.0f, -1.0f, 0.0f};	// front of the robot relative to IMU mounting
	float dirFromMag, dirComplementary, prevDirFromMag;
	static int32_t turn_counter = 0;			// static means that it will survive restart
	TelemetryData_Struct telemetry;
#ifdef USE_GYRO_FOR_IMU
	VectorF gyroSum = {0.0f, 0.0f, 0.0f};		// place for 4 readings from gyro
	float dirFromGyro;
	Complementary_State cState;					// filter state
	ComplementaryInit(&cState, 0.93f); 			// 0.5s time constant with 25Hz sampling
#endif

	uint8_t samplingState = 7;					// state machine's state

	/* Take semaphores initially, so that they can be given later */
	xSemaphoreTake(imuI2CEV, 0);

	/* Reset watchdog timer to 0 effectively starting it and init all IMU modules; then stop timer */
	xTimerReset(imuWatchdogTimer, 0);
	InitAcc();
	InitMag();
#ifdef USE_GYRO_FOR_IMU
	InitGyro();
#endif
	xTimerStop(imuWatchdogTimer, 0);

	/* If timer was successfully stopped then all units must have been correctly initialized, so I2C
	 * works good and IMU task can get higher priority; NULL is THIS TASK */
	vTaskPrioritySet(NULL, PRIORITY_TASK_IMU);

	/* Wait a while until IMU stabilizes, TODO calibration here */
	vTaskDelay(1000/portTICK_RATE_MS);

	TelemetryUpdate_Struct update = {.dX = NAN, .dY = NAN, .Source = TelemetryUpdate_Source_IMU};
	portTickType wakeTime = xTaskGetTickCount();

	while (1) {
		switch (samplingState) {
		case 0: // all data sampled in this common point
			VectorScale(&accSum, 0.25f, &accSum);
			VectorScale(&magSum, 1.0f / 3.0f, &magSum);
#ifdef USE_GYRO_FOR_IMU
			VectorScale(&gyroSum, 0.25f, &gyroSum);
			dirFromGyro -= gyroSum.z * 0.04f;
#endif
			dirFromMag = GetHeading(&accSum, &magSum, &front);

			bool interpolated = false;
			if (globalDoneIMUScaling) {
				dirFromMag = interpolateAngle(dirFromMag);
#ifdef USE_GYRO_FOR_IMU
				dirFromGyro -= globalGyroDrift;
#endif
				interpolated = true;
			}

			// if abs angle > 90 deg and angle sign changes
			if (fabsf(dirFromMag) > HALFM_PI && dirFromMag*prevDirFromMag < 0.0f) {
				if (dirFromMag > 0.0f) // switching from -180 -> 180
					turn_counter--;
				else
					turn_counter++;
			}
			prevDirFromMag = dirFromMag;

			dirFromMag += TWOM_PI * (float)turn_counter;

			if (globalDoneIMUScaling && interpolated) {
#ifdef USE_GYRO_FOR_IMU
				dirComplementary = ComplementaryGet(&cState, dirComplementary - gyroSum.z * 0.04f, dirFromMag);
#else
				dirComplementary = dirFromMag;
#endif
				update.dO = dirComplementary;

				if (xQueueSendToBack(telemetryQueue, &update, 0) == errQUEUE_FULL) {
					if (globalLogEvents) safePrint(25, "Telemetry queue full!\n");
				}

				if (globalLogIMU) {
					getTelemetry(&telemetry, TelemetryStyle_Raw);
#ifdef USE_GYRO_FOR_IMU
					safePrint(60, "Mag: %.1f Gyro: %.1f Comp: %.1f Odo: %.1f\n", dirFromMag / DEGREES_TO_RAD, dirFromGyro / DEGREES_TO_RAD, dirComplementary / DEGREES_TO_RAD, telemetry.O / DEGREES_TO_RAD);
#else
					safePrint(60, "Mag: %.1f Odo: %.1f\n", dirFromMag / DEGREES_TO_RAD, telemetry.O / DEGREES_TO_RAD);
#endif
				}
			}
			else if (!globalDoneIMUScaling) {
				IMUAngles_Type angles = {
#ifdef USE_GYRO_FOR_IMU
					.AngleGyro = dirFromGyro,
#else
					.AngleGyro = NAN,
#endif
					.AngleMag = dirFromMag
				};
				xQueueOverwrite(imuScalingQueue, &angles);
			}

#ifdef USE_GYRO_FOR_IMU
			VectorSet(&gyroSum, 0.0f);
#endif
			VectorSet(&magSum, 0.0f);
			VectorSet(&accSum, 0.0f);
			vTaskDelayUntil(&wakeTime, 10/portTICK_RATE_MS);
			samplingState++;
			break;
		case 1:	// first point of 100Hz
		case 3: // second point of 100Hz
		case 5: // third point of 100Hz
#ifdef USE_GYRO_FOR_IMU
			xTimerReset(imuWatchdogTimer, 0);
			ReadGyroScaled(&read);
			xTimerStop(imuWatchdogTimer, 0);
			VectorAdd(&gyroSum, &read, &gyroSum);
#endif

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
#ifdef USE_GYRO_FOR_IMU
			xTimerReset(imuWatchdogTimer, 0);
			ReadGyroScaled(&read);
			xTimerStop(imuWatchdogTimer, 0);
			VectorAdd(&gyroSum, &read, &gyroSum);
#endif

			xTimerReset(imuWatchdogTimer, 0);
			ReadAccScaled(&read);
			xTimerStop(imuWatchdogTimer, 0);
			VectorAdd(&accSum, &read, &accSum);

			xTimerReset(imuWatchdogTimer, 0);
			ReadMagScaled(&read);
			xTimerStop(imuWatchdogTimer, 0);
			VectorAdd(&magSum, &read, &magSum);

			if (samplingState == 7) { 						// initialize all values to first reading
				prevDirFromMag = GetHeading(&accSum, &magSum, &front); // initial heading;
#ifdef USE_GYRO_FOR_IMU
				getTelemetry(&telemetry, TelemetryStyle_Raw);
				dirFromGyro = dirComplementary = telemetry.O;
#endif

				VectorScale(&accSum, 4.0f, &accSum);		// will be scaled back at state 0
				VectorScale(&magSum, 3.0f, &magSum);
#ifdef USE_GYRO_FOR_IMU
				VectorScale(&gyroSum, 4.0f, &gyroSum);
#endif
			}

			samplingState = 0;
			break;
		default:
			samplingState = 7;
			break;
		}
	}
}

void imuWatchdogOverrun(xTimerHandle xTimer) {
	vTaskDelete(imuTask);
	if (globalLogEvents) safePrint(11, "IMU hang!\n");

	// indicate reset need
	globalIMUHang = true;

	// create IMU task with the lowest priority
	xTaskCreate(TaskIMU, NULL, 400, NULL, 0, &imuTask);
}

void TaskIMUConstructor() {
	I2CEVFlagQueue = xQueueCreate(1, sizeof(uint32_t));
	vSemaphoreCreateBinary(imuI2CEV);
	imuWatchdogTimer = xTimerCreate(NULL, 500/portTICK_RATE_MS, pdFALSE, NULL, imuWatchdogOverrun);
	xTaskCreate(TaskIMU, NULL, TASKIMU_STACKSPACE, NULL, PRIORITY_TASK_IMU, &imuTask);
}

void resetI2C(void) {
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
}

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

float interpolateAngle(float angle) {
	/* Make sure that angle is in the range of interpolation data */
	while (angle < globalMagnetometerImprovData[0])
		angle += TWOM_PI;
	while (angle > globalMagnetometerImprovData[MAG_IMPROV_DATA_POINTS])
		angle -= TWOM_PI;

	uint16_t i;
	for (i = 1; i < MAG_IMPROV_DATA_POINTS+1; ++i) {
		if (globalMagnetometerImprovData[i] > angle) break;
	}

	float a = (float)(i-1)*TWOM_PI / MAG_IMPROV_DATA_POINTS - M_PI;
	float b = (float)i*TWOM_PI / MAG_IMPROV_DATA_POINTS - M_PI;

	float prc = (angle - globalMagnetometerImprovData[i-1]) /
			(globalMagnetometerImprovData[i]-globalMagnetometerImprovData[i-1]);

	angle = prc * (b-a) + a;

	return normalizeOrientation(angle - globalMagStartOrientation + globalMagnetometerImprovData[MAG_IMPROV_DATA_POINTS/2]);
}

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

/* ISR for EXTI Gyro */
void IMUGyroReady() {
	//portBASE_TYPE contextSwitch = pdFALSE;
	//xSemaphoreGiveFromISR(imuGyroReady, &contextSwitch);
	//portEND_SWITCHING_ISR(contextSwitch);
}

/* ISR for EXTI Accelerometer */
void IMUAccReady() {
	//portBASE_TYPE contextSwitch = pdFALSE;
	//xSemaphoreGiveFromISR(imuAccReady, &contextSwitch);
	//portEND_SWITCHING_ISR(contextSwitch);
}

/* ISR for EXTI Magnetometer */
void IMUMagReady() {
	//portBASE_TYPE contextSwitch = pdFALSE;
	//xSemaphoreGiveFromISR(imuMagReady, &contextSwitch);
	//
}
