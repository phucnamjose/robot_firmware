/*
 * ringbuffer.c
 *
 *  Created on: Feb 28, 2020
 *      Author: Dang Nam
 */

#include "ringbuffer.h"

//Array = 0, head = 0, tail = 0, isFull = 0, isEmpty = 1
RINGBUFFER_TypeDef usb_rx_ringbuff 	= {{0}, 0, 0, FALSE, TRUE};
RINGBUFFER_TypeDef cmd_tx_ringbuff	= {{0}, 0, 0, FALSE, TRUE};

RINGBUFFER_TypeDef uart_tx_ringbuff = {{0}, 0, 0, FALSE, TRUE};


/*
 * Function:  ringBuff_PushChar
 * --------------------
 * Write 1 byte from ring buffer.
 *
 *  *ringbuff	: pointer of ring buffer, destination.
 *  data		: data to write
 *
 *  returns:	: TRUE if success
 *  			  FALSE if fail
 */
uint8_t	ringBuff_PushChar(RINGBUFFER_TypeDef *ringbuff, uint8_t data) {
	if (ringbuff->isFull_Flag) {
		return FALSE;
	} else {
			ringbuff->Array[ringbuff->head]	= data;
			ringbuff->head					= (ringbuff->head + 1) % RINGBUFFER_SIZE;
			if (ringbuff->head == ringbuff->tail) {
				ringbuff->isFull_Flag = TRUE;
			}
			ringbuff->isEmpty_Flag = FALSE;
			return TRUE;
	}
}

/*
 * Function:  ringBuff_PopChar
 * --------------------
 * Read 1 byte from ring buffer.
 *
 *  *ringbuff	: pointer of ring buffer
 *  *ptr_data	: pointer of destination
 *
 *  returns:	: TRUE if success
 *  			  FALSE if fail
 */
uint8_t	ringBuff_PopChar(RINGBUFFER_TypeDef *ringbuff, uint8_t *ptr_data) {
	if (ringbuff->isEmpty_Flag) {
		return FALSE;
	} else {
			*ptr_data		= ringbuff->Array[ringbuff->tail];
			ringbuff->tail	= (ringbuff->tail +1) % RINGBUFFER_SIZE;
			if (ringbuff->head == ringbuff->tail) {
				ringbuff->isEmpty_Flag = TRUE;
			}
			ringbuff->isFull_Flag = FALSE;
			return TRUE;
	}
}

/*
 * Function:  ringBuff_PushArray
 * --------------------
 * Write 'len' bytes to ring buffer.
 *
 *  *ringbuff	: pointer of ring buffer, destination.
 *  *ptr_data	: pointer of source
 *  len			: maximum number of bytes that want to copy
 *
 *  returns:	: number of bytes copied
 */
int32_t ringBuff_PushArray(RINGBUFFER_TypeDef *ringbuff, uint8_t *ptr_data, int32_t len) {
	int16_t success_number;

	for (success_number = 0; success_number < len; success_number++) {
		if ( !ringBuff_PushChar(ringbuff, *(ptr_data + success_number))) {
			break;
		}// stop when ring buffer FULL
	}
	return success_number;
}

/*
 * Function:  ringBuff_PopArray
 * --------------------
 * Read 'len' bytes from ring buffer.
 *
 *  *ringbuff	: pointer of ring buffer, source.
 *  *ptr_data	: pointer of destination
 *  len			: maximum number of bytes that want to copy
 *
 *  returns:	: number of bytes copied
 */
int32_t ringBuff_PopArray(RINGBUFFER_TypeDef *ringbuff, uint8_t *ptr_data, int32_t len) {
	int32_t success_number;

	for (success_number = 0; success_number < len; success_number++) {
		if ( !ringBuff_PopChar(ringbuff, (ptr_data + success_number))) {
			break;
		}// stop when ring buffer EMPTY
	}
	return success_number;
}



/*
 * Function:  ringBuff_IsEmpty
 * --------------------
 * Returns full flag of ring buffer
 *
 *  ringbuff	: ring buffer
 */
uint8_t ringBuff_IsFull(RINGBUFFER_TypeDef ringbuff){
	return ringbuff.isFull_Flag;
}

/*
 * Function:  ringBuff_IsEmpty
 * --------------------
 * Returns empty flag of ring buffer
 *
 *  ringbuff	: ring buffer
 */
uint8_t ringBuff_IsEmpty(RINGBUFFER_TypeDef ringbuff){
	return ringbuff.isEmpty_Flag;
}


/*
 * Function:  ringBuff_DistanceOf
 * --------------------
 * Returns the distance from 'tail' to the first occurrence of the character 'cmp_char'
 * in the ring buffer, searching forward from index position 'tail'.
 *
 *  *ringbuff	: pointer of ring buffer
 *  cmp_char	: compare character
 *
 *  returns:	: distance from 'tail' to 'cmp_char'
 *  			  -1 'cmp_char' could not be found.
 */
int32_t	ringBuff_DistanceOf	(RINGBUFFER_TypeDef *ringbuff, uint8_t cmp_char) {
	int32_t index;
	int32_t distance = 0;

	index = ringbuff->tail;
	if (ringbuff->isFull_Flag) {
		for ( int32_t i = 0; i < RINGBUFFER_SIZE; i++, index = (index + 1) % RINGBUFFER_SIZE) {
				if ( cmp_char == ringbuff->Array[index]) {
					return distance;
				}
				distance++;
			}

	} else {
		for ( ; (index != ringbuff->head) || !ringbuff->isEmpty_Flag; index = (index + 1) % RINGBUFFER_SIZE) {
				if ( cmp_char == ringbuff->Array[index]) {
					return distance;
				}
				distance++;
			}
	}
	return -1;
}

/*
 * Function:  ringBuff_HowManySpace
 * --------------------
 * Returns number of space that can write on.
 *
 *  ringbuff	: ring buffer

 */
int32_t	ringBuff_HowManySpace	(RINGBUFFER_TypeDef ringbuff) {
	int32_t  test;
	if (ringbuff.isFull_Flag) {
		return 0;
	} else {
		test = ringbuff.tail - ringbuff.head;
		if ( 0 == test) {
			return RINGBUFFER_SIZE;
		} else if (0 < test) {
			return test;
		} else {
			return (RINGBUFFER_SIZE + test);
		}
	}
}
