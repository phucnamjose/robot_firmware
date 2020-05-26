/*
 * kinematic.h
 *
 *  Created on: Mar 12, 2020
 *      Author: Dang Nam
 */

#ifndef INC_KINEMATIC_H_
#define INC_KINEMATIC_H_

#include "robot_scara.h"

uint8_t		kinematicForward(SCARA_PositionTypeDef *pnt);
uint8_t		kinematicInverse(SCARA_PositionTypeDef *pnt, SCARA_PositionTypeDef current);

#endif /* INC_KINEMATIC_H_ */
