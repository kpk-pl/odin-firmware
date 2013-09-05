#include "imu.h"
#include "i2chelpers.h"
#include "hardware.h"
#include "priorities.h"
#include <math.h>

uint8_t InitAcc() {
	// disable everything
	WriteI2CRegister(IMU_I2C, ACC_ADDR, 0x20, 0x00);

	// CTRL_REG2_A - internally filtered, 0.25Hz high-pass cut off
	//WriteI2CRegister(IMU_I2C, ACC_ADDR, 0x21, 0x13);
	// CTRL_REG3_A - INT active high push-pull on INT1 when data ready
	WriteI2CRegister(IMU_I2C, ACC_ADDR, 0x22, 0x02);
	// CTRL_REG4_A - block update, 2g scale, MSB @ lower address
	WriteI2CRegister(IMU_I2C, ACC_ADDR, 0x23, 0xC0);
	// CTRL_REG1_A - normal mode, 100Hz, all axes enabled
	WriteI2CRegister(IMU_I2C, ACC_ADDR, 0x20, 0x2F);


/*	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	// Configuring interrupt sources from IMU
	// Configuring GPIOs
	// Enabling clock
	RCC_AHB1PeriphClockCmd(IMU_GPIO_INT_CLOCK, ENABLE);
	// Enable SYSCFG clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	// Setting up pins as inputs
	GPIO_InitStructure.GPIO_Pin = IMU_GPIO_AINT_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(IMU_GPIO_INT, &GPIO_InitStructure);
	// Connect EXTI line to GPIO
	SYSCFG_EXTILineConfig(IMU_EXTI_INT_PORTSOURCE, IMU_EXTI_AINT_PINSOURCE);
	// Configuring EXTI interrupts
	EXTI_InitStructure.EXTI_Line = IMU_EXTI_AINT_LINE;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = IMU_EXTI_AINT_TRIGGER;
	EXTI_Init(&EXTI_InitStructure);
	// Configuring NVIC
	NVIC_InitStructure.NVIC_IRQChannel = IMU_EXTI_AINT_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PRIORITY_ISR_IMUREADY;
	NVIC_Init(&NVIC_InitStructure);
*/
	return 1;
}

uint8_t InitMag() {
	// check correct operation reading IRA_REG_M
	if (ReadI2CRegister(IMU_I2C, MAG_ADDR, 0x0A) != 0x48) return 0;

	// CRA_REG_M - normal, 75Hz
	WriteI2CRegister(IMU_I2C, MAG_ADDR, 0x00, 0x18);
	// CRB_REG_M - scale 1.9 Gauss
	WriteI2CRegister(IMU_I2C, MAG_ADDR, 0x01, 0x40);
	// MR_REG_M - continuous operation
	WriteI2CRegister(IMU_I2C, MAG_ADDR, 0x02, 0x00);


/*	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	// Configuring interrupt sources from IMU
	// Configuring GPIOs
	// Enabling clock
	RCC_AHB1PeriphClockCmd(IMU_GPIO_INT_CLOCK, ENABLE);
	// Enable SYSCFG clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	// Setting up pins as inputs
	GPIO_InitStructure.GPIO_Pin = IMU_GPIO_MINT_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(IMU_GPIO_INT, &GPIO_InitStructure);
	// Connect EXTI line to GPIO
	SYSCFG_EXTILineConfig(IMU_EXTI_INT_PORTSOURCE, IMU_EXTI_MINT_PINSOURCE);
	// Configuring EXTI interrupts
	EXTI_InitStructure.EXTI_Line = IMU_EXTI_MINT_LINE;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = IMU_EXTI_MINT_TRIGGER;
	/XTI_Init(&EXTI_InitStructure);
	// Configuring NVIC
	NVIC_InitStructure.NVIC_IRQChannel = IMU_EXTI_MINT_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PRIORITY_ISR_IMUREADY;
	NVIC_Init(&NVIC_InitStructure);
*/
	return 1;
}

uint8_t InitGyro() {
	// check correct operation reading WHO_AM_I register
	if (ReadI2CRegister(IMU_I2C, GYRO_ADDR, 0x0f) != 0xd3) return 0;

	// reboot - power down and disable all axes
	// necessary to ensure that uC reset WILL cause new reading to be taken
	// and interrupt will be raised
	WriteI2CRegister(IMU_I2C, GYRO_ADDR, 0x20, 0x00);

	// CTRL_REG2 - 0.1Hz high pass filter
	WriteI2CRegister(IMU_I2C, GYRO_ADDR, 0x21, 0x06);
	// CTRL_REG3 - data ready, push-pull, DRDY interrupt
	WriteI2CRegister(IMU_I2C, GYRO_ADDR, 0x22, 0x08);
	// CTRL_REG4 - block reading, 250dps, MSB @ lower address
	WriteI2CRegister(IMU_I2C, GYRO_ADDR, 0x23, 0xC0);
	// CTRE_REG5 - FIFO disabled, high pass filter disabled, low pass filter enabled
	WriteI2CRegister(IMU_I2C, GYRO_ADDR, 0x24, 0x0A/*0x1A*/);
	// FIFO_CTRL_REG - FIFO bypass
	WriteI2CRegister(IMU_I2C, GYRO_ADDR, 0x2E, 0x00);
	// CTRL_REG1 - 100Hz, 25Hz cut-off, power-up, all axes enable
	WriteI2CRegister(IMU_I2C, GYRO_ADDR, 0x20, 0x1F);


/*	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	// Configuring interrupt sources from IMU
	// Configuring GPIOs
	// Enabling clock
	RCC_AHB1PeriphClockCmd(IMU_GPIO_INT_CLOCK, ENABLE);
	// Enable SYSCFG clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	// Setting up pins as inputs
	GPIO_InitStructure.GPIO_Pin = IMU_GPIO_GINT_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(IMU_GPIO_INT, &GPIO_InitStructure);
	// Connect EXTI line to GPIO
	SYSCFG_EXTILineConfig(IMU_EXTI_INT_PORTSOURCE, IMU_EXTI_GINT_PINSOURCE);
	// Configuring EXTI interrupts
	EXTI_InitStructure.EXTI_Line = IMU_EXTI_GINT_LINE;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = IMU_EXTI_GINT_TRIGGER;
	//EXTI_Init(&EXTI_InitStructure);
	// Configuring NVIC
	NVIC_InitStructure.NVIC_IRQChannel = IMU_EXTI_GINT_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PRIORITY_ISR_IMUREADY;
	NVIC_Init(&NVIC_InitStructure);
*/
	return 1;
}

static void ReadSensor(uint8_t addr, Vector16 *vec) {
	uint8_t raw[6];
	uint8_t reg = (addr == MAG_ADDR ? 0x03 : 0x28 | 0x80);
	ReadI2CMemory(IMU_I2C, addr, reg, raw, 6);
	vec->x = (int16_t)((uint16_t)raw[0] << 8 | raw[1]);
	vec->y = (int16_t)((uint16_t)raw[2] << 8 | raw[3]);
	vec->z = (int16_t)((uint16_t)raw[4] << 8 | raw[5]);
}

void ReadAccRaw(Vector16 *vec) {
	ReadSensor(ACC_ADDR, vec);
	vec->x >>= 4;
	vec->y >>= 4;
	vec->z >>= 4;
}

void ReadMagRaw(Vector16 *vec) {
	ReadSensor(MAG_ADDR, vec);
}

void ReadGyroRaw(Vector16 *vec) {
	ReadSensor(GYRO_ADDR, vec);
}

void ReadAccScaled(VectorF *vec) {
	Vector16 raw;
	ReadAccRaw(&raw);
	vec->x = ((float)raw.x) * ACC_SCALE_X - ACC_OFFSET_X;
	vec->y = ((float)raw.y) * ACC_SCALE_Y - ACC_OFFSET_Y;
	vec->z = ((float)raw.z) * ACC_SCALE_Z - ACC_OFFSET_Z;
	//vec->x = (float)raw.x;
	//vec->y = (float)raw.y;
	//vec->z = (float)raw.z;
}

void ReadMagScaled(VectorF *vec) {
	Vector16 raw;
	ReadMagRaw(&raw);
	vec->x = ((float)raw.x) * MAG_SCALE_X - MAG_OFFSET_X;
	vec->y = ((float)raw.y) * MAG_SCALE_Y - MAG_OFFSET_Y;
	vec->z = ((float)raw.z) * MAG_SCALE_Z - MAG_OFFSET_Z;
	//vec->x = (float)raw.x;
	//vec->y = (float)raw.y;
	//vec->z = (float)raw.z;
}

void ReadGyroScaled(VectorF *vec) {
	Vector16 raw;
	ReadGyroRaw(&raw);
	vec->x = ((float)raw.x - GYRO_DRIFT_X) * GYRO_SCALE_X;
	vec->y = ((float)raw.y - GYRO_DRIFT_Y) * GYRO_SCALE_Y;
	vec->z = ((float)raw.z - GYRO_DRIFT_Z) * GYRO_SCALE_Z;
	//vec->x = (float)raw.x;
	//vec->y = (float)raw.y;
	//vec->z = (float)raw.z;
}

float GetHeading(const VectorF *acc, const VectorF *mag, const VectorF *front) {
	VectorF east, north;
	VectorF nacc = *acc;
	VectorF nmag = *mag;
	VectorNormalize(&nacc, &nacc);
	VectorCross(&nmag, &nacc, &east);
	VectorNormalize(&east, &east);
	VectorCross(&nacc, &east, &north);
	return -atan2f(VectorDot(&east, front), VectorDot(&north, front));
}
