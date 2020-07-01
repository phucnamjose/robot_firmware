/*
 * robot_lowlayer.h
 *
 *  Created on: May 17, 2020
 *      Author: Dang Nam
 */

#ifndef INC_ROBOT_LOWLAYER_H_
#define INC_ROBOT_LOWLAYER_H_

#include "robot_scara.h"


/* Function Prototype*/
void	lowlayer_scanReset(void);
uint8_t	lowlayer_scanFlow(void);
uint8_t	lowlayer_goToSoftLimit(SCARA_PositionTypeDef *setpoint);
void	lowlayer_readTruePosition(SCARA_PositionTypeDef *true);
void	lowlayer_readSetPosition(SCARA_PositionTypeDef *setpoint);
uint8_t	lowlayer_computeAndWritePulse(SCARA_PositionTypeDef current, SCARA_PositionTypeDef next);
uint8_t	lowlayer_writePulse(int8_t pulse0, int8_t pulse1, int8_t pulse2, int8_t pulse3);
void	lowlayer_resetEncoder(void);
uint8_t lowlayer_readLimitSwitch(void);
int32_t lowlayer_readEncoder(uint8_t encoder_num);
int32_t lowlayer_readCapture(uint8_t capture_num);
void	lowlayer_updateEncoder(void);
void	lowlayer_updateCapture(void);
void	lowlayer_updateLimit(void);
void	lowlayer_setOutput(uint8_t value);
void	lowlayer_CPLD_Init(void);
void	lowlayer_stepMotorInit(void);

#endif /* INC_ROBOT_LOWLAYER_H_ */
