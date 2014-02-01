#ifndef _ODIN_COMPILATION_H_
#define _ODIN_COMPILATION_H_

#define USE_CUSTOM_MOTOR_CONTROLLER		/*!< If defined then custom controller will be used, else PID */
#define USE_IMU_TELEMETRY				/*!< If program should use IMU data for telemetry, else disable anything IMU-related */
#define FOLLOW_TRAJECTORY				/*!< If defined then robot will use trajectory points as source, otherwise driving commands will be available */
#define DRIVE_COMMANDS					/*!< If defined then drive commands will be available */


// Possibility to minimize program size
#ifdef FOLLOW_TRAJECTORY
#define COMPILE_CIRCULAR_BUFFER			/*!< If defined then circular buffer will be compiled with not zero size */
#endif

// Possibility to exclude Gyroscope usage
#ifdef USE_IMU_TELEMETRY
#define USE_GYRO_FOR_IMU
#endif

// Protect from compiling IMU without drive commands
#ifdef USE_IMU_TELEMETRY
#ifndef DRIVE_COMMANDS
#error "Drive commands must be compiled to use IMU"
#endif
#endif

#endif /* _ODIN_COMPILATION_H */


/*
 * Compilation params:
 * compiler: GNU Tools ARM Embedded\4.7 2013q1\bin
 * No external include files
 * FPU hard
 * Symbols:
 * 		STM32F407VG
 * 		STM32F4XX
 * 		USE_STDPERIPH_DRIVER
 * 		__ASSEMBLY__
 * 		__FPU_USED
 * 		ARM_MATH_CM4
 * 		HSE_VALUE=16000000
 * 		USE_FULL_ASSERT
 *
 * 	Misc controls:
 * 	-std=c99
 *
 * 	Link:
 * 	Don't use memory layout
 * 	Discard unused sections
 * 	Don't use the standard system startup files
 * 	Use base C library
 *
 *  Provide path to arm-gcc-link.ld in Scatter File
 *
 * 	Libraries:
 * 	g  gnu tools arm embedded\4.7 2013q1\arm-none-eabi\lib\armv7e-m\fpu\
 * 	c  gnu tools arm embedded\4.7 2013q1\arm-none-eabi\lib\armv7e-m\fpu\
 * 	m  gnu tools arm embedded\4.7 2013q1\arm-none-eabi\lib\armv7e-m\fpu\
 * 	arm_cortexm4lf_math  \DSP
 *  -mcpu=cortex-m4; -mfpu=fpv4-sp-d16; -mfloat-abi=hard; -mthumb; -Wall; -g; -nostartfiles; -Map=FreeRTOSOdinFW.map; -O0; --gc-sections; -Ldsp\; -larm_cortexm4lf_math; -Lc:\gnu tools arm embedded\4.7 2013q1\arm-none-eabi\lib\armv7e-m\fpu\; -lm; -Lc:\gnu tools arm embedded\4.7 2013q1\arm-none-eabi\lib\armv7e-m\fpu\; -lc_s; -Lc:\gnu tools arm embedded\4.7 2013q1\arm-none-eabi\lib\armv7e-m\fpu\; -lg_s; -LD:/CooCox/CoIDE/workspace/FreeRTOSOdinFW; -TD:/CooCox/CoIDE/workspace/FreeRTOSOdinFW/arm-gcc-link.ld;
 *
 *
 *	After build:
 *	${project.path}\scripts\map2html.py ${project.bin}\${output.name}.map out.html size open
 */
