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

uint8_t	lowlayer_computeAndWritePulse(SCARA_PositionTypeDef current, SCARA_PositionTypeDef next) {
	uint16_t	current_var0, current_var1, current_var2, current_var3;
	uint16_t	next_var0, next_var1, next_var2, next_var3;
	int32_t		delta_var0, delta_var1, delta_var2, delta_var3;

	current_var0	= round((current.Theta1 - LIM_MIN_J0) / GEAR_J0);
	current_var1 	= round((current.Theta2 - LIM_MIN_J1) / GEAR_J1);
	current_var2 	= round((current.D3 - LIM_MIN_J2) / GEAR_J2);
	current_var3 	= round((current.Theta4 - LIM_MIN_J3) / GEAR_J3);
	next_var0 		= round((next.Theta1 - LIM_MIN_J0) / GEAR_J0);
	next_var1 		= round((next.Theta2 - LIM_MIN_J1) / GEAR_J1);
	next_var2 		= round((next.D3 - LIM_MIN_J2) / GEAR_J2);
	next_var3 		= round((next.Theta4 - LIM_MIN_J3) / GEAR_J3);

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

	lowlayer_writePulse((int8_t)delta_var0, (int8_t)delta_var1, (int8_t)delta_var2, (int8_t)delta_var3);

	return TRUE;
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
	// Sleep Step if it does not work
	if (pulse3_abs == 0) {
		HAL_GPIO_WritePin(STEP_SLEEP_GPIO_Port, STEP_SLEEP_Pin, GPIO_PIN_RESET); // Sleep low active
	} else {
		HAL_GPIO_WritePin(STEP_SLEEP_GPIO_Port, STEP_SLEEP_Pin, GPIO_PIN_SET);
	}

	// Write to Module DDA
	FSMC_Write(ADDRESS_DDA_0, (uint32_t)pulse0_combine);
	FSMC_Write(ADDRESS_DDA_1, (uint32_t)pulse1_combine);
	FSMC_Write(ADDRESS_DDA_2, (uint32_t)pulse2_combine);
	FSMC_Write(ADDRESS_DDA_3, (uint32_t)pulse3_combine);

	// Trigger
	uint8_t delay = 50;
	HAL_GPIO_WritePin(PULSE_WRITE_GPIO_Port, PULSE_WRITE_Pin, GPIO_PIN_RESET);
	while (delay--);
	HAL_GPIO_WritePin(PULSE_WRITE_GPIO_Port, PULSE_WRITE_Pin, GPIO_PIN_SET);

	return TRUE;
}

uint8_t lowlayer_readLimitSwitch(void) {
	uint16_t read_data = FSMC_Read(ADDRESS_LIMIT);

	return (uint8_t)read_data;
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
	HAL_GPIO_WritePin(STEP_ENABLE_GPIO_Port, STEP_ENABLE_Pin, GPIO_PIN_RESET); // ENABLE low active
	HAL_GPIO_WritePin(STEP_RESET_GPIO_Port, STEP_RESET_Pin, GPIO_PIN_SET); // RESET low active
	HAL_GPIO_WritePin(STEP_SLEEP_GPIO_Port, STEP_SLEEP_Pin, GPIO_PIN_SET); // SLEEP low active
}
