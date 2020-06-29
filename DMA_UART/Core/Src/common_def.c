/*
 * common_def.c
 *
 *  Created on: Mar 3, 2020
 *      Author: Dang Nam
 */

#include "common_def.h"
#include "ringbuffer.h"
#include "stdio.h"
#include "string.h"
#include "dma.h"
#include "usart.h"

extern RINGBUFFER_TypeDef uart_tx_ringbuff;
extern UART_HandleTypeDef huart4;
extern DMA_HandleTypeDef hdma_uart4_tx;
uint8_t log_uart_dma_buff[LOG_BUFFER_SIZE];


/*
 * Function:  LOG_REPORT
 * --------------------
 * Send message to DMA UART.
 *
 *  *message	: pointer of ring buffer, destination.
 *  line		: line code number
 *
 *  returns:	: TRUE if success
 *  			  FALSE if fail
 */
uint8_t LOG_REPORT(char *message, uint16_t line) {
	uint8_t temp_buff[64];
	int32_t len;

	len = snprintf((char*)temp_buff, 63, "LINE:%d, %s \r\n", line, message);
	if (-1 == len) {
		return FALSE;
	}
	ringBuff_PushArray(&uart_tx_ringbuff, temp_buff, len);
	if (HAL_DMA_GetState(&hdma_uart4_tx) == HAL_DMA_STATE_BUSY) {
		return TRUE;
	}// dma busy
	uint16_t size_dma;
	size_dma = ringBuff_PopArray(&uart_tx_ringbuff, log_uart_dma_buff, LOG_BUFFER_SIZE);
	HAL_UART_Transmit_DMA(&huart4, log_uart_dma_buff, size_dma);
	return TRUE;
}

int32_t	double2string( uint8_t *result, double value, uint8_t precision) {
	uint8_t nguyen[4];
	uint8_t le[6];
	int8_t sign;
	double temp1, temp2;
	int32_t index;

	if((precision < 0) || (6 < precision)) {
		precision = 6;
	}

	if(value < 0) {
		sign = -1;
	} else {
		sign = 1;
	}
	value = value*sign;

	if (value > 10000.0f) {
		return -1;
	}

	nguyen[0] 	= (int32_t)value/1000;
	nguyen[1]	= (int32_t)value/100 - nguyen[0]*10;
	nguyen[2]	= (int32_t)value/10 - nguyen[0]*100 - nguyen[1]*10;
	nguyen[3]	= (int32_t)value - nguyen[0]*1000 - nguyen[1]*100 - nguyen[2]*10;

	temp1	= (value - nguyen[0]*1000 - nguyen[1]*100 - nguyen[2]*10 - nguyen[3])*1000;
	le[0] 	= (int32_t)temp1/100;
	le[1] 	= (int32_t)temp1/10 - le[0]*10;
	le[2]	= (int32_t)temp1 - le[0]*100 - le[1]*10;

	temp2	= (temp1 - le[0]*100 - le[1]*10 - le[2])*1000;
	le[3] 	= (int32_t)temp2/100;
	le[4] 	= (int32_t)temp2/10 - le[3]*10;
	le[5]	= (int32_t)temp2 - le[3]*100 - le[4]*10;
	// Rounding
	if ((temp2 - le[3]*1000 - le[4]*100 - le[5]) >= 0.5) {
		le[5]++;
	}

	index = 0;

	if( -1 == sign) {
		*(result + index++) = '-'; // Negative
	}
	// Find first position
	if ( value < 1) {
		*(result + index++) = 0x30;
	} else {
		int8_t i = 0;
		for( ; i < 4; i++) {
			if (nguyen[i] > 0) {
				*(result + index++) = nguyen[i] + 0x30;
				i++;
				break;
			}
		}

		for( ; i < 4; i++) {
		*(result + index++) = nguyen[i] + 0x30;
		}
	}
	*(result + index++) = '.';
	for( int8_t i = 0; i < precision; i++) {
			*(result + index++) = le[i] + 0x30;
	}

	// Chua giai quyet van de lam tron chu so thap phan
	*(result + index++) = 0;
	return index;
}
