#include <stm32f4xx.h>

#include "arm_math.h"

#include "TaskIMU.h"
#include "main.h"
#include "priorities.h"

#include "i2chelpers.h"
#include "imu.h"
#include "vector.h"
#include "complementary.h"

/* Sets I2C peripheral */
static void initI2CforIMU(void);

/* Initializes globalMagnetometerImprov */
static void initMagnetometerImprovInstance(float x0);

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
				if (!globalDoneIMUScaling)
					initMagnetometerImprovInstance(cangle - telemetry.O);  // scale to be 0 at initial
				cangle = telemetry.O;								   	   // linear interpolation in this point gives 0
				prev_angle = cangle;
				estDir = cangle;
			}

			samplingState = 0;
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

void initMagnetometerImprovInstance(float x0) {
	globalMagnetometerImprov.x1 = -M_PI;
	float offset = -M_PI - x0;

	for (uint16_t i = 0; i<globalMagnetometerImprov.nValues; i++) {
		globalMagnetometerImprovData[i] = globalMagnetometerImprov.xSpacing * i + offset; // Must be continuous, no normalization. Normalize afterwards.
	}
}
