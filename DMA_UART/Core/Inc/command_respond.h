/*
 * command_decode.h
 *
 *  Created on: Mar 7, 2020
 *      Author: Dang Nam
 */

#ifndef INC_COMMAND_RESPOND_H_
#define INC_COMMAND_RESPOND_H_

#include  "common_def.h"
#include  "robot_scara.h"


typedef enum
{
	CMD_STOPNOW					= 0x00,
	CMD_SCAN_LIMIT				= 0x01,
	CMD_MOVE_HOME				= 0x02,
	CMD_MOVE_LINE				= 0x03,
	CMD_MOVE_CIRCLE				= 0x04,
	CMD_MOVE_JOINT				= 0x05,
	CMD_ROTATE_SINGLE			= 0x06,
	CMD_OUTPUT					= 0x07,
	CMD_READ_STATUS				= 0x08,
	CMD_READ_POSITION			= 0x09,
	CMD_SETTING					= 0x0A,
	CMD_ERROR					= 0x0B,
	NUM_OF_COMMAND				= 0x0C
}Robot_CommandTypedef;

typedef enum
{
    RPD_IDLE				= 0x00,
    RPD_BUSY				= 0x01,
	RPD_POSITION			= 0x02,
    RPD_START				= 0x03,
    RPD_RUNNING				= 0x04,
	RPD_DONE				= 0x05,
    RPD_STOP				= 0x06,
    RPD_ERROR				= 0x07,
	RPD_OK 					= 0x08,
	RPD_DUTY				= 0x09,
    NUM_OF_RESPOND			= 0x0A
}Robot_RespondTypedef;


/*** Instance Form ***/



/*---Function Prototype---*/
Robot_CommandTypedef 	commandRead		(uint8_t *message,
										int32_t *id_command,
										DUTY_Command_TypeDef *duty_cmd);

Robot_RespondTypedef	commandReply	(Robot_CommandTypedef cmd_type,
										DUTY_Command_TypeDef duty_cmd,
										uint8_t *detail);

int32_t					commandRespond	(Robot_RespondTypedef rpd,
										int32_t id_command,
										char *detail,
										char *respond);


#endif /* INC_COMMAND_RESPOND_H_ */
