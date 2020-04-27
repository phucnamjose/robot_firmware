/*
 * kinematic.c
 *
 *  Created on: Mar 12, 2020
 *      Author: Dang Nam
 */

#include "kinematic.h"
#include "math.h"

uint8_t		kinematicForward(SCARA_PositionTypeDef *pnt) {
	float x, y, z, roll;

	x =   a1*cosf(pnt->Theta1)
		+ a2*cosf(pnt->Theta1 + pnt->Theta2)
		+ a4*cosf(pnt->Theta1 + pnt->Theta2 - pnt->Theta4);
	y =   a1*sinf(pnt->Theta1)
		+ a2*sinf(pnt->Theta1 + pnt->Theta2)
		+ a4*sinf(pnt->Theta1 + pnt->Theta2 - pnt->Theta4);
	z =   d1 - pnt->D3 - d4;
	roll = pnt->Theta1 + pnt->Theta2 - pnt->Theta4;

	pnt->x = x;
	pnt->y = y;
	pnt->z = z;
	pnt->roll = roll;

	return TRUE;
}

uint8_t		kinematicInverse(SCARA_PositionTypeDef *pnt, SCARA_PositionTypeDef current) {
	float theta1, theta2, theta2_positive, theta2_negative, d3, theta4 , pWx, pWy;
	float s1, c1, s2, s2_positive, s2_negative, c2 , temp;

	d3  = d1 - d4 - pnt->z;
	pWx = pnt->x - a4*cosf(pnt->roll);
	pWy = pnt->y - a4*sinf(pnt->roll);
	c2  = (pWx*pWx + pWy*pWy - a1*a1 - a2*a2) / (2*a1*a2);
	temp = 1 - c2*c2;
	if ( temp < 0 ) {
		return FALSE;
	}
	s2_positive  = sqrtf(temp); // Note that there are 2 solution: elbow up & elbow down
	s2_negative	 = -s2_positive;

	theta2_positive = atan2f(s2_positive,c2);
	theta2_negative = atan2f(s2_negative,c2);
	// Choose relevant situation : nearest
	if ( fabsf( theta2_positive - current.Theta2) <= fabsf( theta2_negative - current.Theta2)) {
		s2 		= s2_positive;
		theta2 	= theta2_positive;
	} else {
		s2 		= s2_negative;
		theta2 	= theta2_negative;
	}

	s1 = ((a1 + a2*c2)*pWy - a2*s2*pWx) / (pWx*pWx + pWy*pWy);
	c1 = ((a1 + a2*c2)*pWx + a2*s2*pWy) / (pWx*pWx + pWy*pWy);
	theta1 = atan2f(s1,c1);
 	theta4 = theta1 + theta2 - pnt->roll;

	if ( SCARA_STATUS_OK != scaraCheckWorkSpace4(theta1, theta2, d3, theta4)) {
		return FALSE; // Over workspace !!!
	}
	pnt->Theta1 = theta1;
	pnt->Theta2 = theta2;
	pnt->D3		= d3;
	pnt->Theta4 = theta4;

	return TRUE; // All is well
}
