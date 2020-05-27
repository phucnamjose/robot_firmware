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
uint8_t	lowlayer_computeAndWritePulse(SCARA_PositionTypeDef current, SCARA_PositionTypeDef next);
uint8_t	lowlayer_writePulse(int8_t pulse0, int8_t pulse1, int8_t pulse2, int8_t pulse3);
uint8_t lowlayer_readLimitSwitch(void);
void	lowlayer_setOutput(uint8_t value);

#endif /* INC_ROBOT_LOWLAYER_H_ */
