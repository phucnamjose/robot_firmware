/*
 * system_params.h
 *
 *  Created on: May 26, 2020
 *      Author: Dang Nam
 */

#ifndef INC_SYSTEM_PARAMS_H_
#define INC_SYSTEM_PARAMS_H_

/* Limit Joint*/
#define LIM_MIN_J0		(-85.0*PI/180)
#define LIM_MAX_J0		(85*PI/180)
#define LIM_MIN_J1		(-135*PI/180)
#define LIM_MAX_J1		(135*PI/180)
#define LIM_MIN_J2		(0)
#define LIM_MAX_J2		(95.0)
#define LIM_MIN_J3		(-165*PI/360)
#define LIM_MAX_J3		(165*PI/360)

/* Maximum Velocity System*/
#define V_DESIGN_3D		(0.9*LIM_PULSE_J2)/(GEAR_J2*T_SAMPLING)
#define V_DESIGN_ROLL	(0.9*LIM_PULSE_J3*2*PI)/(GEAR_J3*T_SAMPLING)
#define V_DESIGN_J0		(0.9*LIM_PULSE_J0*2*PI)/(GEAR_J0*T_SAMPLING)
#define V_DESIGN_J1		(0.9*LIM_PULSE_J1*2*PI)/(GEAR_J1*T_SAMPLING)
#define V_DESIGN_J2		(0.9*LIM_PULSE_J2)/(GEAR_J2*T_SAMPLING)
#define V_DESIGN_J3		(0.9*LIM_PULSE_J3*2*PI)/(GEAR_J3*T_SAMPLING)

/* Maximum Accelerate System*/
#define A_DESIGN_3D		(V_DESIGN_3D/5)
#define A_DESIGN_ROLL	(V_DESIGN_ROLL/5)
#define A_DESIGN_J0		(V_DESIGN_J0/5)
#define A_DESIGN_J1		(V_DESIGN_J1/5)
#define A_DESIGN_J2		(V_DESIGN_J2/5)
#define A_DESIGN_J3		(V_DESIGN_J3/5)

/* Sampling Period*/
#define T_SAMPLING		(0.01f)


/* Motor Positive( Anti-Clockwise) Direction ---> Driver */
#define DIR_J0				(0x01)
#define DIR_J1				(0x01)
#define DIR_J2				(0x00)
#define DIR_J3				(0x01)

/* Encoder Increase Direction --- CPLD count independently */
#define DIR_ENCODER_0		(0x00)
#define DIR_ENCODER_1		(0x00)
#define DIR_ENCODER_2		(0x01)

/* Motor Ratio */
// baud rate
#define GEAR_J0				(20000.0f) // per round ( 2*PI)
// baud rate
#define GEAR_J1				(16000.0f) // per round ( 2*PI)
// baud rate, ball screw T3
#define GEAR_J2				(500.0f/3.0f) // per milimeter.
// motor, micro step, big-gear, small-gear
#define GEAR_J3				(400.0f*32.0f*60.0f/19.0f) // per round ( 2*PI)


/* Limit pulse per 10 ms (pulse) */
#define LIM_PULSE_J0		(16u) // Calculate in file Excel
#define LIM_PULSE_J1		(40u)
#define LIM_PULSE_J2		(33u)
#define LIM_PULSE_J3		(101u)

/* Module DDA Address (8 bit data) */
#define ADDRESS_DDA_0 			(0x60000000)
#define ADDRESS_DDA_1 			(0x60000002)
#define ADDRESS_DDA_2 			(0x60000004)
#define ADDRESS_DDA_3 			(0x60000006)

/* Limit Switch Address (8 bit data) */
#define ADDRESS_LIMIT 			(0x60000020)

/* Encoder Registers Address (16 bit data) */
// Encoder 32 bit counter (mode x4)
#define ADDRESS_ENC0_LOW		(0x60000040)
#define ADDRESS_ENC0_HIGH		(0x60000042)
#define ADDRESS_ENC1_LOW		(0x60000044)
#define ADDRESS_ENC1_HIGH		(0x60000046)
#define ADDRESS_ENC2_LOW		(0x60000048)
#define ADDRESS_ENC2_HIGH		(0x6000004A)

/* Limit Capture Registers Address (16 bit data) */
#define ADDRESS_CAP0_LOW		(0x60000060)
#define ADDRESS_CAP0_HIGH		(0x60000062)
#define ADDRESS_CAP1_LOW		(0x60000064)
#define ADDRESS_CAP1_HIGH		(0x60000066)
#define ADDRESS_CAP2_LOW		(0x60000068)
#define ADDRESS_CAP2_HIGH		(0x6000006A)

/* Limit Absolute Position (Calibration) (rad) */
#define POSITION_LIM0_NEG		(-1.621454508)
#define POSITION_LIM1_POS		(2.448773299)
#define POSITION_LIM2_NEG		(77.674)
#define POSITION_LIM3_POS		(2.994575935)

/* Kinematic Parameters (mm) */
#define		d1				(211.0f)
#define 	a1				(197.0f)
#define		a2				(160.0f)
#define		d4				(77.674f)
#define		a4				(32.36f)

#endif /* INC_SYSTEM_PARAMS_H_ */
