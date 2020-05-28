/*
 * system_params.h
 *
 *  Created on: May 26, 2020
 *      Author: Dang Nam
 */

#ifndef INC_SYSTEM_PARAMS_H_
#define INC_SYSTEM_PARAMS_H_

// Motor Direction Positive( Anti-Clockwise)
#define DIR_J0				(0x01)
#define DIR_J1				(0x01)
#define DIR_J2				(0x00)
#define DIR_J3				(0x01)

// Motor Ratio
#define GEAR_J0				(20000.0f)
#define GEAR_J1				(32000.0f)
#define GEAR_J2				(500.0f / 3.0f) // ball screw 3 mm / round.
#define GEAR_J3				(400.0f*16.0f)

// Limit pulse per 10 ms
#define LIM_PULSE_J0		(104u) // Calculate in file Excel
#define LIM_PULSE_J1		(125u)
#define LIM_PULSE_J2		(33u)
#define LIM_PULSE_J3		(67u)

// Module DDA Address
#define ADDRESS_DDA_0 		(0x60000000)
#define ADDRESS_DDA_1 		(0x60000002)
#define ADDRESS_DDA_2 		(0x60000004)
#define ADDRESS_DDA_3 		(0x60000006)

// Limit Switch Address
#define ADDRESS_LIMIT 		(0x60000010)

// Kinematic Parameters
#define		d1				(211.0f)
#define 	a1				(197.0f)
#define		a2				(160.0f)
#define		d4				(77.0f)
#define		a4				(30.0f)

#endif /* INC_SYSTEM_PARAMS_H_ */
