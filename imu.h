#ifndef __IMU_H__
#define __IMU_H__

#include <stm32f4xx.h>
#include "vector.h"

#define ACC_ADDR 0x30
#define MAG_ADDR 0x3C
#define GYRO_ADDR 0xD3

/*
 * All scales are set to 25Hz sampling and SUMS of signals
 * Accelerometer and gyro are sampling at 100Hz, 4 samples are added together to form one sample
 * Magnetometer is sampling at 75Hz, 3 samples are added together to form one sample
 * These are not avarage signals but SUMS!!!
 */

#define GYRO_DRIFT_X (-35.91f / 4.0f)
#define GYRO_DRIFT_Y (-23.45f / 4.0f)
#define GYRO_DRIFT_Z (-38.18f / 4.0f)

#define GYRO_SCALE_Z (1.5216e-4) // scale to rad/s
#define GYRO_SCALE_X GYRO_SCALE_Z
#define GYRO_SCALE_Y GYRO_SCALE_Z

#define ACC_RAW_MAXX (4232.0f / 4.0f)
#define ACC_RAW_MAXY (4106.0f / 4.0f)
#define ACC_RAW_MAXZ (4073.0f / 4.0f)
#define ACC_RAW_MINX (-4035.0f / 4.0f)
#define ACC_RAW_MINY (-4291.0f / 4.0f)
#define ACC_RAW_MINZ (-4060.0f / 4.0f)

// 2.0 is twice the acceleration value in g's on surface of Earth
#define ACC_SCALE_X	(2.0f / (ACC_RAW_MAXX - ACC_RAW_MINX))
#define ACC_SCALE_Y (2.0f / (ACC_RAW_MAXY - ACC_RAW_MINY))
#define ACC_SCALE_Z (2.0f / (ACC_RAW_MAXZ - ACC_RAW_MINZ))

#define ACC_OFFSET_X ((ACC_RAW_MAXX + ACC_RAW_MINX) / 2.0f * ACC_SCALE_X)
#define ACC_OFFSET_Y ((ACC_RAW_MAXY + ACC_RAW_MINY) / 2.0f * ACC_SCALE_Y)
#define ACC_OFFSET_Z ((ACC_RAW_MAXZ + ACC_RAW_MINZ) / 2.0f * ACC_SCALE_Z)

#define MAG_RAW_MAXX (751.0f/*678.0f*/ / 3.0f)
#define MAG_RAW_MAXY (990.0f/*864.0f*/ / 3.0f)
#define MAG_RAW_MAXZ (724.0f/*664.0f*/ / 3.0f)
#define MAG_RAW_MINX (-1459.0f/*-1390.0f*/ / 3.0f)
#define MAG_RAW_MINY (-1324.0f/*-1204.0f*/ / 3.0f)
#define MAG_RAW_MINZ (-1253.0f/*-1160.0f*/ / 3.0f)

// 28 mGauss is the magnetic field strength where measurements were taken
#define MAG_RELATIVE_MGAUSS (28.0f)

#define MAG_SCALE_X	(2.0f * MAG_RELATIVE_MGAUSS / (MAG_RAW_MAXX - MAG_RAW_MINX))
#define MAG_SCALE_Y (2.0f * MAG_RELATIVE_MGAUSS / (MAG_RAW_MAXY - MAG_RAW_MINY))
#define MAG_SCALE_Z (2.0f * MAG_RELATIVE_MGAUSS / (MAG_RAW_MAXZ - MAG_RAW_MINZ))

#define MAG_OFFSET_X ((MAG_RAW_MAXX + MAG_RAW_MINX) / 2.0f * MAG_SCALE_X)
#define MAG_OFFSET_Y ((MAG_RAW_MAXY + MAG_RAW_MINY) / 2.0f * MAG_SCALE_Y)
#define MAG_OFFSET_Z ((MAG_RAW_MAXZ + MAG_RAW_MINZ) / 2.0f * MAG_SCALE_Z)

uint8_t InitAcc();
uint8_t InitMag();
uint8_t InitGyro();

void ReadAccRaw(Vector16 *vec);
void ReadMagRaw(Vector16 *vec);
void ReadGyroRaw(Vector16 *vec);

void ReadAccScaled(VectorF *vec); // unit is 1g
void ReadMagScaled(VectorF *vec); // unit is 1mG
void ReadGyroScaled(VectorF *vec); // unit is rad/s

float GetHeading(const VectorF *acc, const VectorF *mag, const VectorF *front);

#endif /* __IMU_H__ */
