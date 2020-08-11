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
	CMD_STOPNOW = 0x00,
	CMD_SCAN_LIMIT,
	CMD_MOVE_HOME,
	CMD_MOVE_LINE,
	CMD_MOVE_CIRCLE,
	CMD_MOVE_JOINT,
	CMD_ROTATE_SINGLE,
	CMD_OUTPUT,
	CMD_READ_STATUS,
	CMD_READ_POSITION,
	CMD_SETTING,
	CMD_METHOD_CHANGE,

    CMD_JOB_NEW,
    CMD_JOB_DELETE,
    CMD_JOB_PUSH_MOVE_LINE,
    CMD_JOB_PUSH_MOVE_JOINT,
    CMD_JOB_PUSH_OUTPUT,
    CMD_JOB_TEST,
    CMD_JOB_RUN,// 7 job

    CMD_KEYBOARD,// 2 key board
	CMD_KEY_SPEED,
	CMD_ERROR,
	NUM_OF_COMMAND
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
