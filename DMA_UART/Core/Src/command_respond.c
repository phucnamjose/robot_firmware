/*
 * command_decode.c
 *
 *  Created on: Mar 7, 2020
 *      Author: Dang Nam
 */

#include "command_respond.h"
#include "robot_scara.h"
#include <stdio.h>
#include <string.h>

extern const char *DETAIL_STATUS[NUM_OF_STATUS];

const char *ROBOTCOMMAND[NUM_OF_COMMAND - 1]  = {"STOP",
												"SCAN",
												"HOME",
												"MOVL",
												"MOVC",
												"MOVJ",
												"ROTA",
												"OUTP",
												"READ",
												"POSI",
												"SETT",
												"METH",// 12 nomal

										        "JNEW",
										        "JDEL",
										        "JPML",
										        "JPMJ",
										        "JPOP",
										        "JTES",
										        "JRUN ", // 7 job

										        "KEYB"// 1 key board
												};

const char *ROBOTRESPOND[NUM_OF_RESPOND]  	  = {"IDLE",
												"BUSY",
												"POSI",
												"STAR",
												"RUNN",
												"DONE",
												"STOP",
												"ERRO",
												"OKAY"};


Robot_CommandTypedef 	commandRead	(uint8_t *message, int32_t *id_command, DUTY_Command_TypeDef *duty_cmd) {
	char command[10];
	char para[70];
	int32_t result;
	memset(para, 0, 70*sizeof(char));
	result = sscanf((char*)message, "%d %s %70c",(int*) id_command, command, para);
	duty_cmd->id_command = *id_command;
	// Stop Now
	if ( 0 == strcmp( command, ROBOTCOMMAND[CMD_STOPNOW])) {
		duty_cmd->robot_mode = SCARA_MODE_STOP;
		duty_cmd->robot_method = SCARA_METHOD_SEMI_AUTO;
		duty_cmd->change_method = FALSE;
		return CMD_STOPNOW;

	// Scan Limit
	} else if  ( 0 == strcmp( command, ROBOTCOMMAND[CMD_SCAN_LIMIT])) {
		duty_cmd->robot_mode = SCARA_MODE_SCAN;
		duty_cmd->robot_method = SCARA_METHOD_SEMI_AUTO;
		duty_cmd->change_method = FALSE;
		return CMD_SCAN_LIMIT;

	// Move Home
	} else if  ( 0 == strcmp( command, ROBOTCOMMAND[CMD_MOVE_HOME])) {
		if (3 == result) {
			result = sscanf( para, "%lf %lf",
							&(duty_cmd->v_factor),
							&(duty_cmd->a_factor));
			if (2 != result) {
				return CMD_ERROR;
			}
		} else {
			return CMD_ERROR;
		}
		duty_cmd->target_point.x = 250;
		duty_cmd->target_point.y = -200;
		duty_cmd->target_point.z = 120;
		duty_cmd->target_point.roll = 0;

		duty_cmd->modeInit_type = DUTY_MODE_INIT_QVA;
		duty_cmd->space_type = DUTY_SPACE_JOINT;
		duty_cmd->joint_type = DUTY_JOINT_4DOF;
		duty_cmd->robot_mode = SCARA_MODE_DUTY;
		duty_cmd->robot_method = SCARA_METHOD_SEMI_AUTO;
		duty_cmd->change_method = FALSE;
		return CMD_MOVE_HOME;

	// Move Line
	} else if  ( 0 == strcmp( command, ROBOTCOMMAND[CMD_MOVE_LINE])) {
		if (3 == result) {
			double temp_fl;
			int8_t mode_init;
			result = sscanf( para, "%lf %lf %lf %lf %lf %d %lf",
							&(duty_cmd->target_point.x),
							&(duty_cmd->target_point.y),
							&(duty_cmd->target_point.z),
							&(duty_cmd->target_point.roll),
							&(duty_cmd->v_factor),
							(int *)&mode_init,
							&temp_fl);

			if (7 != result) {
				return CMD_ERROR;
			}
			duty_cmd->path_type = DUTY_PATH_LINE;
			duty_cmd->space_type = DUTY_SPACE_TASK;

			if ( DUTY_MODE_INIT_QVA == mode_init) {
				duty_cmd->modeInit_type = DUTY_MODE_INIT_QVA;
				duty_cmd->a_factor		= temp_fl;
			} else if ( DUTY_MODE_INIT_QVT == mode_init) {
				duty_cmd->modeInit_type = DUTY_MODE_INIT_QVT;
				duty_cmd->time_total	= temp_fl;
			} else {
				return CMD_ERROR;
			}
		} else {
			return CMD_ERROR;
		}
		duty_cmd->robot_mode = SCARA_MODE_DUTY;
		duty_cmd->robot_method = SCARA_METHOD_SEMI_AUTO;
		duty_cmd->change_method = FALSE;
		return CMD_MOVE_LINE;

	// Move Circle
	} else if  ( 0 == strcmp( command, ROBOTCOMMAND[CMD_MOVE_CIRCLE])) {
		if (3 == result) {
			double temp_fl;
			int8_t mode_init;
			result = sscanf( para, "%lf %lf %lf %lf %lf %lf %lf %d %lf %d %lf",
							&(duty_cmd->target_point.x),
							&(duty_cmd->target_point.y),
							&(duty_cmd->target_point.z),
							&(duty_cmd->target_point.roll),
							&(duty_cmd->sub_point.x),
							&(duty_cmd->sub_point.y),
							&(duty_cmd->sub_point.z),
							(int *)&(duty_cmd->sub_para_int), // dir
							&(duty_cmd->v_factor),
							(int *)&mode_init,
							&temp_fl);

			if (11 != result) {
				return CMD_ERROR;
			}
			duty_cmd->path_type = DUTY_PATH_CIRCLE;
			duty_cmd->space_type = DUTY_SPACE_TASK;

			if ( DUTY_MODE_INIT_QVA == mode_init) {
				duty_cmd->modeInit_type = DUTY_MODE_INIT_QVA;
				duty_cmd->a_factor		= temp_fl;
			} else if ( DUTY_MODE_INIT_QVT == mode_init) {
				duty_cmd->modeInit_type = DUTY_MODE_INIT_QVT;
				duty_cmd->time_total	= temp_fl;
			} else {
				return CMD_ERROR;
			}
		} else {
			return CMD_ERROR;
		}
		duty_cmd->robot_mode = SCARA_MODE_DUTY;
		duty_cmd->robot_method = SCARA_METHOD_SEMI_AUTO;
		duty_cmd->change_method = FALSE;
		return CMD_MOVE_CIRCLE;

	// Move Joint
	} else if  ( 0 == strcmp( command, ROBOTCOMMAND[CMD_MOVE_JOINT])) {
		if (3 == result) {
			double temp_fl;
			int8_t mode_init;
			result = sscanf( para, "%lf %lf %lf %lf %lf %d %lf",
							&(duty_cmd->target_point.x),
							&(duty_cmd->target_point.y),
							&(duty_cmd->target_point.z),
							&(duty_cmd->target_point.roll),
							&(duty_cmd->v_factor),
							(int *)&mode_init,
							&temp_fl);

			if (7 != result) {
				return CMD_ERROR;
			}
			duty_cmd->space_type = DUTY_SPACE_JOINT;
			duty_cmd->joint_type = DUTY_JOINT_4DOF;

			if ( DUTY_MODE_INIT_QVA == mode_init) {
				duty_cmd->modeInit_type = DUTY_MODE_INIT_QVA;
				duty_cmd->a_factor		= temp_fl;
			} else if ( DUTY_MODE_INIT_QVT == mode_init) {
				duty_cmd->modeInit_type = DUTY_MODE_INIT_QVT;
				duty_cmd->time_total	= temp_fl;
			} else {
				return CMD_ERROR;
			}
		} else {
			return CMD_ERROR;
		}
		duty_cmd->robot_mode = SCARA_MODE_DUTY;
		duty_cmd->robot_method = SCARA_METHOD_SEMI_AUTO;
		duty_cmd->change_method = FALSE;

		return CMD_MOVE_JOINT;

	// Rotate Single
	} else if  ( 0 == strcmp( command, ROBOTCOMMAND[CMD_ROTATE_SINGLE])) {
		if (3 == result) {
			double temp_fl;
			int8_t mode_init;
			result = sscanf( para, "%d %lf %lf %d %lf",
							(int *)&(duty_cmd->sub_para_int),
							&(duty_cmd->sub_para_double),
							&(duty_cmd->v_factor),
							(int *)&mode_init,
							&temp_fl);

			if (5 != result) {
				return CMD_ERROR;
			}
			duty_cmd->space_type = DUTY_SPACE_JOINT;
			duty_cmd->joint_type = DUTY_JOINT_SINGLE;

			if ( DUTY_MODE_INIT_QVA == mode_init) {
				duty_cmd->modeInit_type = DUTY_MODE_INIT_QVA;
				duty_cmd->a_factor		= temp_fl;
			} else if ( DUTY_MODE_INIT_QVT == mode_init) {
				duty_cmd->modeInit_type = DUTY_MODE_INIT_QVT;
				duty_cmd->time_total	= temp_fl;
			} else {
				return CMD_ERROR;
			}
		} else {
			return CMD_ERROR;
		}
		duty_cmd->robot_mode = SCARA_MODE_DUTY;
		duty_cmd->robot_method = SCARA_METHOD_SEMI_AUTO;
		duty_cmd->change_method = FALSE;

		return CMD_ROTATE_SINGLE;

	// Set Output
	} else if  ( 0 == strcmp( command, ROBOTCOMMAND[CMD_OUTPUT])) {
		result = sscanf( para, "%d",(int *)&(duty_cmd->sub_para_int));

		if (1 != result) {
			return CMD_ERROR;
		}

		return CMD_OUTPUT;

	// Read Status
	} else if  ( 0 == strcmp( command, ROBOTCOMMAND[CMD_READ_STATUS])) {
		return CMD_READ_STATUS;

	// Read Position
	} else if  ( 0 == strcmp( command, ROBOTCOMMAND[CMD_READ_POSITION])) {
			return CMD_READ_POSITION;

	// Setting
	} else if  ( 0 == strcmp( command, ROBOTCOMMAND[CMD_SETTING])) {
		int8_t mode_trajec;
		int8_t mode_coordinate;
		result = sscanf( para, "%d %d",
						(int *)&mode_coordinate,
						(int *)&mode_trajec);

		if (2 != result) {
			return CMD_ERROR;
		}

		if ( DUTY_COORDINATES_ABS == mode_coordinate) {
			duty_cmd->coordinate_type = DUTY_COORDINATES_ABS;
		} else if ( DUTY_COORDINATES_REL == mode_coordinate) {
			duty_cmd->coordinate_type = DUTY_COORDINATES_REL;
		} else {
			return CMD_ERROR;
		}

		if ( DUTY_TRAJECTORY_LSPB == mode_trajec) {
			duty_cmd->trajec_type = DUTY_TRAJECTORY_LSPB;
		} else if ( DUTY_TRAJECTORY_SCURVE == mode_trajec) {
			duty_cmd->trajec_type = DUTY_TRAJECTORY_SCURVE;
		} else {
			return CMD_ERROR;
		}
		return CMD_SETTING;

	} else if  ( 0 == strcmp( command, ROBOTCOMMAND[CMD_METHOD_CHANGE])) {
		int8_t method;
		result = sscanf( para, "%d",
						(int *)&method);
		if (1 != result) {
			return CMD_ERROR;
		}
		if (SCARA_METHOD_MANUAL == method) {
			duty_cmd->robot_method = SCARA_METHOD_MANUAL;
		} else if (SCARA_METHOD_SEMI_AUTO == method) {
			duty_cmd->robot_method = SCARA_METHOD_SEMI_AUTO;
		} else if (SCARA_METHOD_AUTO == method) {
			duty_cmd->robot_method = SCARA_METHOD_AUTO;
		} else {
			return CMD_ERROR;
		}
		duty_cmd->change_method = TRUE;
		return CMD_METHOD_CHANGE;
	} else if  ( 0 == strcmp( command, ROBOTCOMMAND[CMD_JOB_NEW])) {
		duty_cmd->robot_method = SCARA_METHOD_AUTO;
		duty_cmd->change_method = FALSE;
		return CMD_JOB_NEW;
	} else if  ( 0 == strcmp( command, ROBOTCOMMAND[CMD_JOB_DELETE])) {
		duty_cmd->robot_method = SCARA_METHOD_AUTO;
		duty_cmd->change_method = FALSE;
		return CMD_JOB_DELETE;
	} else if  ( 0 == strcmp( command, ROBOTCOMMAND[CMD_JOB_PUSH_MOVE_LINE])) {
		duty_cmd->robot_method = SCARA_METHOD_AUTO;
		duty_cmd->change_method = FALSE;
		return CMD_JOB_PUSH_MOVE_LINE;
	} else if  ( 0 == strcmp( command, ROBOTCOMMAND[CMD_JOB_PUSH_MOVE_JOINT])) {
		duty_cmd->robot_method = SCARA_METHOD_AUTO;
		duty_cmd->change_method = FALSE;
		return CMD_JOB_PUSH_MOVE_JOINT;
	} else if  ( 0 == strcmp( command, ROBOTCOMMAND[CMD_JOB_PUSH_OUTPUT])) {
		duty_cmd->robot_method = SCARA_METHOD_AUTO;
		duty_cmd->change_method = FALSE;
		return CMD_JOB_PUSH_OUTPUT;
	} else if  ( 0 == strcmp( command, ROBOTCOMMAND[CMD_JOB_TEST])) {
		duty_cmd->robot_method = SCARA_METHOD_AUTO;
		duty_cmd->change_method = FALSE;
		return CMD_JOB_TEST;
	} else if  ( 0 == strcmp( command, ROBOTCOMMAND[CMD_JOB_RUN])) {
		duty_cmd->robot_method = SCARA_METHOD_AUTO;
		duty_cmd->change_method = FALSE;
		return CMD_JOB_RUN;
	} else if  ( 0 == strcmp( command, ROBOTCOMMAND[CMD_KEYBOARD])) {
		int8_t key_num;
		result = sscanf( para, "%d",
						(int *)&key_num);
		if (1 != result) {
			return CMD_ERROR;
		}
		duty_cmd->keyboard = (SCARA_KeyTypeDef)key_num;
		duty_cmd->robot_method = SCARA_METHOD_MANUAL;
		duty_cmd->change_method = FALSE;
		return CMD_KEYBOARD;
	}
	// Error command
	else {
		return CMD_ERROR;
	}
}

Robot_RespondTypedef	commandReply	(Robot_CommandTypedef cmd_type,
										DUTY_Command_TypeDef duty_cmd,
										uint8_t *detail) {
	Robot_RespondTypedef ret;

	switch(cmd_type) {
	case CMD_STOPNOW:
	case CMD_SCAN_LIMIT:
	case CMD_MOVE_HOME:
	case CMD_MOVE_LINE:
	case CMD_MOVE_CIRCLE:
	case CMD_MOVE_JOINT:
	case CMD_ROTATE_SINGLE:
		ret = RPD_DUTY;
		break;
	case CMD_OUTPUT:
		{
			if (1 == duty_cmd.sub_para_int) {
				scaraSetOutput(1);
				strcpy( (char *)detail, "Output ON");
			} else if (0 == duty_cmd.sub_para_int) {
				scaraSetOutput(0);
				strcpy( (char *)detail, "Output OFF");
			} else {
				strcpy( (char *)detail, "Wrong Value");
				return RPD_ERROR;
			}
			ret = RPD_OK;
		}
		break;
	case CMD_READ_STATUS:
		{
			SCARA_ModeTypeDef		current_mode;
			SCARA_DutyStateTypeDef 	current_state;
			current_mode	 = scaraGetMode();
			current_state	 = scaraGetDutyState();
			if ( SCARA_MODE_DUTY == current_mode && SCARA_DUTY_STATE_READY == current_state) {
				ret = RPD_IDLE;
			} else {
				ret = RPD_BUSY;
			}
		}
		break;
	case CMD_READ_POSITION:
		{
			SCARA_PositionTypeDef position;
			scaraGetPosition(&position);
			scaraPosition2String((char *)detail, position);
			ret =  RPD_POSITION;
		}
		break;
	case CMD_SETTING:
		if ( DUTY_COORDINATES_ABS == duty_cmd.coordinate_type) {
			strcpy( (char *)detail, "ABSOLUTE.");
		} else if ( DUTY_COORDINATES_REL == duty_cmd.coordinate_type) {
			strcpy( (char *)detail, "RELATIVE.");
		} else {
			strcat((char *)detail, DETAIL_STATUS[SCARA_STATUS_ERROR_COORDINATE]);
			return RPD_ERROR;
		}

		if ( DUTY_TRAJECTORY_LSPB == duty_cmd.trajec_type) {
			strcat((char *)detail, " LSPB");
		} else if ( DUTY_TRAJECTORY_SCURVE == duty_cmd.trajec_type) {
			strcat((char *)detail, " S-CURVE");
		} else {
			strcat((char *)detail, DETAIL_STATUS[SCARA_STATUS_ERROR_TRAJECTORY]);
			return RPD_ERROR;
		}
		ret = RPD_OK;
		break;
	case CMD_METHOD_CHANGE:
		ret = RPD_DUTY;
		break;
	case CMD_JOB_NEW:
		break;
	case CMD_JOB_DELETE:
		break;
	case CMD_JOB_PUSH_MOVE_LINE:
		break;
	case CMD_JOB_PUSH_MOVE_JOINT:
		break;
	case CMD_JOB_PUSH_OUTPUT:
		break;
	case CMD_JOB_TEST:
		break;
	case CMD_JOB_RUN:
		ret = RPD_DUTY;
		break;
	case CMD_KEYBOARD:
		ret = RPD_DUTY;
		break;
	case CMD_ERROR:
		strcpy( (char *)detail, "Check parameters");
		ret = RPD_ERROR;
		break;
	default:
		strcpy( (char *)detail, "Unknown command");
		ret = RPD_ERROR;
	}
	return ret;
}


int32_t				commandRespond	(Robot_RespondTypedef rpd,
										int32_t id_command,
										char *detail,
										char *respond) {
	int out_lenght;
	switch(rpd) {
	case RPD_IDLE:
	case RPD_BUSY:
		{
			uint8_t		isScanLitmit;
			isScanLitmit = scaraIsScanLimit();
			out_lenght = snprintf( (char *)respond,
									20,
									"%d %s %d",
									(int)id_command,
									ROBOTRESPOND[rpd],
									(int)isScanLitmit);
		}
		break;
	case RPD_POSITION:
	case RPD_START:
	case RPD_RUNNING:
	case RPD_DONE:
	case RPD_STOP:
	case RPD_ERROR:
	case RPD_OK:

		{
			out_lenght = snprintf( (char *)respond,
									145,
									"%d %s %s",
									(int)id_command,
									ROBOTRESPOND[rpd],
									(char *)detail);
		}
		break;
	default:
		out_lenght = 0;
	}

	return (int32_t)out_lenght;
}
