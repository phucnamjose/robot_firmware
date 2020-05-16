/*
 * kinematic.h
 *
 *  Created on: Mar 12, 2020
 *      Author: Dang Nam
 */

#ifndef INC_KINEMATIC_H_
#define INC_KINEMATIC_H_

#include "common_def.h"
#include "robot_scara.h"


#define		d1		(211.0f)
#define 	a1		(197.0f)
#define		a2		(160.0f)
#define		d4		(77.0f)
#define		a4		(30.0f)



uint8_t		kinematicForward(SCARA_PositionTypeDef *pnt);
uint8_t		kinematicInverse(SCARA_PositionTypeDef *pnt, SCARA_PositionTypeDef current);

#endif /* INC_KINEMATIC_H_ */
