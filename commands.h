#ifndef __ODIN_COMMANDS_H
#define __ODIN_COMMANDS_H

/*
 * Command form as should be transmitted:
 * <{one byte, letter as below}:{optional params delimited by ':'}>
 * In fact anything that strtof does not interpret as part of number might be used as delimiter,
 * for example letter (but not all), $, % or others
 * example:
 * <B>
 * <U>
 * <V:s:1>
 * <w:c>
 * <C:l:1:0.1:300>
 * Commands are not checked for validity! Only the basic things are checked to prevent looking outside of a string received.
 * If command is definetely too short, 'Incorrect command' is printed.
 * If command does not exist, 'No such command' is printed.
 * Some commands are checked in their respective handlers (like speed in DRIVE_COMMAND (C) cannot be negative)
 * Some command errors are not handled ex. <C:l:0:0.1> - no length specified, will drive 0.0 metres because strtof will return 0.0f.
 * Some parameters have default value, ex. <C:1:3:0.1:100> will drive 100 metres with pen down (command incorrect because pen-down
 *  	can be only 0 / 1 but here is 3). 1 is not always default. Usually it is 0, like in LOGGING_COMMAND, WIFI_RESET etc.
 */

#define LOW_LEVEL_AUA 			'!'		/* No params | toggles LED 1 */
#define HIGH_LEVEL_AUA 			'?'		/* No params | prints 'Hello!' using OS tasks | ex 'Hello!' */
#define AVAILABLE_MEMORY 		'M'		/* No params | prints 'Available memory: {memory in kB}kB' | ex. 'Available memory: 65045kB' */
#define CPU_RESET				'X'		/* No params | resets CPU */

#define BATTERY_VOLTAGE			'b'		/* No params | prints 'Battery voltage: {voltage in V}V | ex. 'Battery voltage: 7.87V' */
#define PEN_DOWN				'D'		/* No params | moves pen down */
#define PEN_UP					'U'		/* No params | moves pen up */
#define	MOTOR_LEFT_SPEED		'L'		/* One param: speed | Sets motor speed by setting PWM duty cycle | ex: <L:50> sets 50% duty cycle */
#define MOTOR_RIGHT_SPEED		'R'		/* One param: speed | As above */
#define MOTOR_LEFT_ENCODER		'l'		/* No params | prints left encoder tick count in form 'Encoder left: {tick count}' | ex. 'Encoder left: 4574242'*/
#define MOTOR_RIGHT_ENCODER		'r'		/* No params | as above */
#define MOTORS_ENABLE			'E'		/* One param: 1 - enable, 0 - disable | enables or disabled motors by setting appropriate signals in H-bridges and enabling or disabling speed regulator | ex. <E:1> enables motors and regulator */
#define MOTORS_SET_SPEEDS		'S'		/* Two params: left speed & right speed (<S:0.3:-2.6>); speed in rads/sec | regulator kicks in and keeps constant speed based on encoder readings */
#define SPEED_REGULATOR_ENABLE	'G'		/* One param: 1 - enable, 0 - disable | enables or disables motors speed regulator; might be useful when setting speed in raw-PWM-like way */
#define TELEMETRY_PRINT			'T'		/* No params | Prints most up-to-date telemetry data; coordinates in mm, orientation in degrees (-180 - 180) | ex. 'X:-45.24 Y:652.32 O:-42.2' */
#define WIFI_MODE				'w'		/* One param: c - commands / d - data | Sets WiFi mode to command or data | ex. <w:d>*/
#define WIFI_RESET				'x'		/* One param: 1 - reset on / 0 - reset off | Sets WiFi reset signal | ex. <x:1> */
#define LANTERN_ENABLE			'O'		/* One param: 1 - enable / 0 - disable | Enable or disable LED lantern | ex. <O:1> */
#define DELAY_COMMANDS			'W'		/* One param: number of milliseconds to wait before executing next command */
#define CPU_USAGE				'u'		/* No params; Returns CPU usage in percent as float %.1f in form "CPU usage: %.1f%%\n" */

#define LOGGING_COMMAND			'V'		/*
										 * Two params: first - what to log, second - 1/0 wheather to turn logging on or off
										 * Example: <V:t:1>
										 * Types of logging commands are specified in Logging_Type (below migh not be up-to-date)
										 * t - telemetry
										 * s - speed
										 * e - events
										 */

#define IMPORT_TRAJECTORY_POINTS	'P' /*
										 * One param: number of points (<P:#>) After this command reception goes into DMA mode to receive # points
										 * This does not work now
										 */

#define DRIVE_COMMAND			'C'		/*
										 * First param: character representing drive command type
										 * Second param: 1 - use pen / 0 - do not use pen
										 * Third param: speed in m/s (only positive values allowed) | SPEED IN M/S !!!
										 * Fourth param: First option
										 * Fifth param: Optional second option ( not for all commands )
										 * Example:  <C:p:1:0.3:121.3:-13.7> - drive to point (121.3mm, -13.7mm) with pen down at speed 0.3 m/s
										 * Commands types are specified in DriveCommand_Type (below might not be up to date)
										 * p - point
										 * l - line
										 * a - arc
										 * d - degrees turn
										 */

#define MOTORS_CHARACTERISTIC	'h'		/*
										 * 3 params:
										 * First param: Which wheel - l / r; left or right wheel command
										 * Second param: PWM duty cycle in percent; allowed -100.0-100.0; the sign determines the direction
										 * Third param: Number of speed logs; only positive values
										 * Example: <h:l:-34:100>
										 * Side effects: this command turns on logging speed and disables speed regulator.
										 * 	Speed regulator can be enabled with another command.
										 */

#define MOTORS_CHARACTERISTIC2	'H'		/*
										 * 3 params:
										 * First param: PWM duty cycle in percent for left speed; allowed -100.0-100.0; the sign determines the direction
										 * Second param: PWM duty cycle as above for right wheel
										 * Third param: Number of speed logs; only positive values
										 * Example: <H:10.2:-34:100>
										 * Side effects: this command turns on logging speed and disables speed regulator.
										 * 	Speed regulator can be enabled with another command.
										 */

#endif
