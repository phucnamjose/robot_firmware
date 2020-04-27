/*
 * comunicate_payload.c
 *
 *  Created on: Mar 2, 2020
 *      Author: Dang Nam
 */


#include "communicate_payload.h"
#include "malloc.h"
#include "string.h"


/*
 * Function: packPayLoad
 * --------------------
 * transform data buffer to message, add control char.
 *
 *  *input_buff		: pointer of input buffer, source
 *  *output_buff	: pointer of output buffer, destination
 *  in_lenght		: lenght of input_buff
 *
 *  returns:		: lenght of output_buff
 *  				  -1 if error
 */
int32_t	packPayload		(uint8_t *input_buff, uint8_t *output_buff, int32_t in_lenght) {
	if (NULL == input_buff) {
		return -1;
	}

	if (NULL == output_buff) {
		return -1;
	}

	int32_t out_lenght = 0;

	out_lenght = in_lenght + 2;//lenght of start char and end char
//	for (uint16_t i = 0; i < in_lenght; i++) {
//		if ( (START_CHAR == input_buff[i]) ||	(END_CHAR == input_buff[i])	|| (ADD_CHAR == input_buff[i]) ) {
//			out_lenght++;
//		}
//	}// compute output buffer lenght

	uint8_t temp_buff[out_lenght];

	out_lenght = 0;
	temp_buff[out_lenght++] = START_CHAR;
	for (int32_t i = 0; i < in_lenght; i++) {
//		if ( (START_CHAR == input_buff[i]) ||	(END_CHAR == input_buff[i])	|| (ADD_CHAR == input_buff[i]) ) {
//
//			temp_buff[out_lenght++] = ADD_CHAR;
//			temp_buff[out_lenght++] = input_buff[i] ^ XOR_CHAR;
//		} else {
//			temp_buff[out_lenght++] = input_buff[i];
//		}
		temp_buff[out_lenght++] = input_buff[i];
	}
	temp_buff[out_lenght++] = END_CHAR;

	memmove(output_buff, temp_buff, out_lenght);

	return out_lenght;
}


/*
 * Function: unPackPayLoad
 * --------------------
 * Restore data buffer, remove control char.
 *
 *  *input_buff		: pointer of mes buffer
 *  in_lenght		: lenght of input_buff
 *
 *  returns:		: lenght of output_buff
 *  				  -1 if error
 */
int32_t	unPackPayload	(uint8_t *message_buff, int32_t in_lenght) {
	if (in_lenght < MIN_MESSAGE_LENGHT) {
		return -1;
	}// check minimum lenght

	if ( (START_CHAR != message_buff[0]) || (END_CHAR != message_buff[in_lenght - 1]) ) {
		return -1;
	}// check start char and end char

	int32_t out_lenght = in_lenght - 2;

//	for (uint16_t i = 0; i < in_lenght; i++) {
//		if ( (START_CHAR == message_buff[i]) || (END_CHAR == message_buff[i]) || (ADD_CHAR == message_buff[i]) ) {
//			out_lenght--;
//		}
//	}// compute output buffer lenght

	uint8_t temp_buff[out_lenght];

	out_lenght = 0;
	for (int32_t i = 0; i < (in_lenght - 1); i++) {
		if ( (START_CHAR == message_buff[i]) ||	(END_CHAR == message_buff[i]) ) {
			;
//		} else if ( (ADD_CHAR == message_buff[i]) ) {
//			temp_buff[out_lenght++] = message_buff[++i] ^ XOR_CHAR;
		}
		else {
			temp_buff[out_lenght++] = message_buff[i];
		}
	}
	memset((uint8_t*)message_buff, 0, in_lenght);
	memmove(message_buff, temp_buff, out_lenght);

	return out_lenght;
}
