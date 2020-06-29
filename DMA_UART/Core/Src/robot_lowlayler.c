/*
 * robot_lowlayler.c
 *
 *  Created on: May 17, 2020
 *      Author: Dang Nam
 */

#include "robot_lowlayer.h"
#include "common_def.h"
#include <math.h>
#include "fsmc.h"
#include "gpio.h"
#include "stdlib.h"
#include "system_params.h"


int32_t position_encoder[3];
int32_t position_capture[3];

int32_t pulse_accumulate[4];

int32_t offset_setpoint[3];
int32_t offset_encoder[3];
int32_t offset_stepper;

uint8_t	limit_switch[4];
uint8_t state_scan;
uint8_t scan_flag;

const int8_t	pulse_scan[4] = {4, 5, 8, 30};

void	lowlayer_scanReset(void) {
	scan_flag = 0;
	state_scan = 0;
	HAL_GPIO_WritePin(CAPTURE_ENABLE_GPIO_Port, CAPTURE_ENABLE_Pin, GPIO_PIN_RESET);
}

uint8_t	lowlayer_scanFlow(void) {
	// Scan limit switch from 3 to 0
	int8_t pulse[4] = {0, 0, 0 ,0};
	lowlayer_updateLimit();
	if (state_scan < 4) {
		if (limit_switch[3 - state_scan] == 0) {
			pulse[3 - state_scan] = pulse_scan[3 - state_scan];
			lowlayer_writePulse(-pulse[0], pulse[1], -pulse[2], pulse[3]);
		} else {
			state_scan++;
		}

		return FALSE;
	} else {
		lowlayer_updateCapture();
		HAL_GPIO_WritePin(CAPTURE_ENABLE_GPIO_Port, CAPTURE_ENABLE_Pin, GPIO_PIN_SET);
		lowlayer_writePulse(0, 0, 0, 0);
		scan_flag = 1;

		return TRUE;
	}
}

void	lowlayer_readTruePosition(SCARA_PositionTypeDef *true) {
	lowlayer_updateEncoder();
	true->Theta1 = LIM_MIN_J0 + position_encoder[0]*2.0*PI/4.0/GEAR_J0; // Servo Motor
	true->Theta2 = LIM_MIN_J1 + position_encoder[1]*2.0*PI/4.0/GEAR_J1; // Servo Motor
	true->D3	 = LIM_MIN_J2 + position_encoder[2]*2.0*PI/4.0/GEAR_J2; // Servo Motor
	true->Theta4 = LIM_MIN_J3 + pulse_accumulate[3]*2.0*PI/4.0/GEAR_J3; // Stepper Motor
}

uint8_t	lowlayer_computeAndWritePulse(SCARA_PositionTypeDef current, SCARA_PositionTypeDef next) {
	uint32_t	current_var0, current_var1, current_var2, current_var3;
	uint32_t	next_var0, next_var1, next_var2, next_var3;
	int64_t		delta_var0, delta_var1, delta_var2, delta_var3;
	uint8_t 	result;

	current_var0	= round((current.Theta1 - LIM_MIN_J0) * GEAR_J0/(2*PI));
	current_var1 	= round((current.Theta2 - LIM_MIN_J1) * GEAR_J1/(2*PI));
	current_var2 	= round((current.D3 - LIM_MIN_J2) * GEAR_J2);
	current_var3 	= round((current.Theta4 - LIM_MIN_J3) * GEAR_J3/(2*PI));
	next_var0 		= round((next.Theta1 - LIM_MIN_J0) * GEAR_J0/(2*PI));
	next_var1 		= round((next.Theta2 - LIM_MIN_J1) * GEAR_J1/(2*PI));
	next_var2 		= round((next.D3 - LIM_MIN_J2) * GEAR_J2);
	next_var3 		= round((next.Theta4 - LIM_MIN_J3) * GEAR_J3/(2*PI));

	delta_var0 = next_var0 - current_var0;
	delta_var1 = next_var1 - current_var1;
	delta_var2 = next_var2 - current_var2;
	delta_var3 = next_var3 - current_var3;

	if (abs(delta_var0) > 127
		|| abs(delta_var1) > 127
		|| abs(delta_var2) > 127
		|| abs(delta_var3) > 127) {
		return FALSE;
	} // Can't convert to int8_t , over range

	result = lowlayer_writePulse((int8_t)delta_var0,
								 (int8_t)delta_var1,
								 (int8_t)delta_var2,
								 (int8_t)delta_var3);

	return result;
}


uint8_t	lowlayer_writePulse(int8_t pulse0, int8_t pulse1, int8_t pulse2, int8_t pulse3) {
	uint8_t pulse0_combine, pulse1_combine, pulse2_combine, pulse3_combine;
	uint8_t pulse0_abs, pulse1_abs, pulse2_abs, pulse3_abs;

	// Var 0
	if (pulse0 < 0) {
		pulse0_abs = -pulse0;
		pulse0_combine = -pulse0 | ((!DIR_J0) << 7); // Negative : Clockwise
	} else {
		pulse0_abs = pulse0;
		pulse0_combine = pulse0 | (DIR_J0 << 7); // Positive : Anti-Clockwise
	}
	// Var 1
	if (pulse1 < 0) {
		pulse1_abs = -pulse1;
		pulse1_combine = -pulse1 | ((!DIR_J1) << 7);
	} else {
		pulse1_abs = pulse1;
		pulse1_combine = pulse1 | (DIR_J1 << 7);
	}
	// Var 2
	if (pulse2 < 0) {
		pulse2_abs = -pulse2;
		pulse2_combine = -pulse2 | ((!DIR_J2) << 7);
	} else {
		pulse2_abs = pulse2;
		pulse2_combine = pulse2 | (DIR_J2 << 7);
	}
	// Var 3
	if (pulse3 < 0) {
		pulse3_abs = -pulse3;
		pulse3_combine = -pulse3 | ((!DIR_J3) << 7);
	} else {
		pulse3_abs = pulse3;
		pulse3_combine = pulse3 | (DIR_J3 << 7);
	}

	// Check limit
	if (pulse0_abs > LIM_PULSE_J0
		|| pulse1_abs > LIM_PULSE_J1
		|| pulse2_abs > LIM_PULSE_J2
		|| pulse3_abs > LIM_PULSE_J3) {
		return FALSE;
	}
	//Sleep Step if it does not work
//	if (pulse3_abs == 0) {
//		HAL_GPIO_WritePin(STEP_ENABLE_GPIO_Port, STEP_ENABLE_Pin, GPIO_PIN_SET); // Disable
//	} else {
//		HAL_GPIO_WritePin(STEP_ENABLE_GPIO_Port, STEP_ENABLE_Pin, GPIO_PIN_RESET);
//	}
	HAL_GPIO_WritePin(STEP_ENABLE_GPIO_Port, STEP_ENABLE_Pin, GPIO_PIN_RESET);

	// Write to Module DDA
	FSMC_Write(ADDRESS_DDA_0, (uint32_t)pulse0_combine);
	FSMC_Write(ADDRESS_DDA_1, (uint32_t)pulse1_combine);
	FSMC_Write(ADDRESS_DDA_2, (uint32_t)pulse2_combine);
	FSMC_Write(ADDRESS_DDA_3, (uint32_t)pulse3_combine);

	// Trigger
	uint8_t delay = 100;
	HAL_GPIO_WritePin(PULSE_WRITE_GPIO_Port, PULSE_WRITE_Pin, GPIO_PIN_RESET);
	while (delay--);
	HAL_GPIO_WritePin(PULSE_WRITE_GPIO_Port, PULSE_WRITE_Pin, GPIO_PIN_SET);

	// Accumulate
	pulse_accumulate[0] += pulse0;
	pulse_accumulate[1] += pulse1;
	pulse_accumulate[2] += pulse2;
	pulse_accumulate[3] += pulse3;

	return TRUE;
}

void	lowlayer_resetEncoder(void) {
	// Trigger
	uint8_t delay = 100;
	HAL_GPIO_WritePin(ENCODER_RESET_GPIO_Port, ENCODER_RESET_Pin, GPIO_PIN_SET);
	while (delay--);
	HAL_GPIO_WritePin(ENCODER_RESET_GPIO_Port, ENCODER_RESET_Pin, GPIO_PIN_RESET);
}

uint8_t lowlayer_readLimitSwitch(void) {
	uint16_t read_data = FSMC_Read(ADDRESS_LIMIT);

	return (uint8_t)read_data;
}

int32_t lowlayer_readEncoder(uint8_t encoder_num) {
	int32_t data;
	if (encoder_num <= 2) {
		uint16_t low_word = FSMC_Read(ADDRESS_ENC0_LOW + encoder_num*4);
		uint16_t high_word = FSMC_Read(ADDRESS_ENC0_LOW + encoder_num*4 + 2);
		data = high_word;
		data = (data << 16) | low_word;
	} else {
		data = 0;
	}
	return data;
}

int32_t lowlayer_readCapture(uint8_t capture_num) {
	int32_t data;
	if (capture_num <= 2) {
		uint16_t low_word = FSMC_Read(ADDRESS_CAP0_LOW + capture_num*4);
		uint16_t high_word = FSMC_Read(ADDRESS_CAP0_LOW + capture_num*4 + 2);
		data = high_word;
		data = (data << 16) | low_word;
	} else {
		data = 0;
	}
	return data;
}

void	lowlayer_updateEncoder(void) {
	for (uint8_t i = 0; i < 3; i++) {
		position_encoder[i] = lowlayer_readEncoder(i);
	}
}

void	lowlayer_updateCapture(void) {
	for (uint8_t i = 0; i < 3; i++) {
			position_capture[i] = lowlayer_readCapture(i);
	}
}

void	lowlayer_updateLimit(void) {
	uint8_t limit_data = lowlayer_readLimitSwitch();
	for (uint8_t i = 0; i < 4; i++) {
			limit_switch[i] = (limit_data & (0x01 << i)) ? 1 : 0;
	}
}

void	lowlayer_setOutput(uint8_t value) {
	if (value > 0) {
		HAL_GPIO_WritePin(OUTPUT_2_GPIO_Port, OUTPUT_2_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(OUTPUT_2_GPIO_Port, OUTPUT_2_Pin, GPIO_PIN_RESET);
	}
}

void	lowlayer_CPLD_Init(void) {
	HAL_GPIO_WritePin(STOP_GPIO_Port, STOP_Pin, GPIO_PIN_SET); // STOP low active
}

void	lowlayer_stepMotorInit(void) {
	HAL_GPIO_WritePin(STEP_ENABLE_GPIO_Port, STEP_ENABLE_Pin, GPIO_PIN_SET); // ENABLE low active
	HAL_GPIO_WritePin(STEP_RESET_GPIO_Port, STEP_RESET_Pin, GPIO_PIN_SET); // RESET low active
	HAL_GPIO_WritePin(STEP_SLEEP_GPIO_Port, STEP_SLEEP_Pin, GPIO_PIN_SET); // SLEEP low active
}
