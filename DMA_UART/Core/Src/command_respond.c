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
												"SETT"
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
		return CMD_STOPNOW;

	// Scan Limit
	} else if  ( 0 == strcmp( command, ROBOTCOMMAND[CMD_SCAN_LIMIT])) {
		duty_cmd->robot_mode = SCARA_MODE_SCAN;
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
			duty_cmd->robot_mode = SCARA_MODE_DUTY;

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
			duty_cmd->robot_mode = SCARA_MODE_DUTY;

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
			duty_cmd->robot_mode = SCARA_MODE_DUTY;

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
			duty_cmd->robot_mode = SCARA_MODE_DUTY;

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

	// Error command
	} else {
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
			strcpy( (char *)detail, "Absolute.");
		} else if ( DUTY_COORDINATES_REL == duty_cmd.coordinate_type) {
			strcpy( (char *)detail, "Relative.");
		} else {
			strcat((char *)detail, DETAIL_STATUS[SCARA_STATUS_ERROR_COORDINATE]);
			return RPD_ERROR;
		}

		if ( DUTY_TRAJECTORY_LSPB == duty_cmd.trajec_type) {
			strcat((char *)detail, " LSPB");
		} else if ( DUTY_TRAJECTORY_SCURVE == duty_cmd.trajec_type) {
			strcat((char *)detail, " S-curve");
		} else {
			strcat((char *)detail, DETAIL_STATUS[SCARA_STATUS_ERROR_TRAJECTORY]);
			return RPD_ERROR;
		}
		ret = RPD_OK;
		break;
	case CMD_ERROR:
		strcpy( (char *)detail, "Check parameters");
		ret = RPD_ERROR;
		break;
	default:
		strcpy( (char *)detail, "Check parameters");
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
