/*
 * ringbuffer.h
 *
 *  Created on: Feb 28, 2020
 *      Author: Dang Nam
 */

#ifndef INC_RINGBUFFER_H_
#define INC_RINGBUFFER_H_

#include "common_def.h"


#define RINGBUFFER_SIZE		(1024)



typedef struct
{
	uint8_t 	Array[RINGBUFFER_SIZE];
	int32_t 	head;
	int32_t 	tail;
	uint8_t 	isFull_Flag;
	uint8_t 	isEmpty_Flag;
}RINGBUFFER_TypeDef;


/* USER CODE BEGIN Prototypes */
uint8_t		ringBuff_PushChar		(RINGBUFFER_TypeDef *ringbuff, uint8_t data);
uint8_t 	ringBuff_PopChar		(RINGBUFFER_TypeDef *ringbuff, uint8_t *ptr_data);
int32_t		ringBuff_PushArray		(RINGBUFFER_TypeDef *ringbuff, uint8_t *ptr_data, int32_t len);
int32_t		ringBuff_PopArray		(RINGBUFFER_TypeDef *ringbuff, uint8_t *ptr_data, int32_t len);
uint8_t		ringBuff_IsFull			(RINGBUFFER_TypeDef ringbuff);
uint8_t		ringBuff_IsEmpty		(RINGBUFFER_TypeDef ringbuff);
int32_t		ringBuff_DistanceOf		(RINGBUFFER_TypeDef *ringbuff, uint8_t cmp_char);
int32_t		ringBuff_HowManySpace	(RINGBUFFER_TypeDef ringbuff);

#endif /* INC_RINGBUFFER_H_ */
