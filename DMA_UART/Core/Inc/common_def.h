/*
 * common_def.h
 *
 *  Created on: Feb 29, 2020
 *      Author: Dang Nam
 */

#ifndef INC_COMMON_DEF_H_
#define INC_COMMON_DEF_H_

#include "main.h"



#define FALSE	(0)
#define TRUE	(1)

#define PI		(3.14159265359f)

#define NULL	((void *)0)


#define LOG_BUFFER_SIZE		(256)


uint8_t LOG_REPORT(char *message, uint16_t line);
int32_t	float2string( uint8_t *result, float value, uint8_t precision);

#endif /* INC_COMMON_DEF_H_ */
