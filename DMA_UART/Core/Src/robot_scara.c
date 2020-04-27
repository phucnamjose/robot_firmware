/*
 * robot_scara.c
 *
 *  Created on: Mar 10, 2020
 *      Author: Dang Nam
 */


#include "robot_scara.h"
#include "kinematic.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

SCARA_TypeDef 				mySCARA = { SCARA_MODE_DUTY,
										SCARA_DUTY_STATE_READY,
										TRUE,
										FALSE};
DUTY_TypeDef				myDUTY;

SCARA_PositionTypeDef		positionCurrent;
SCARA_PositionTypeDef		positionNext;

Trajectory_TargetTypeDef	joint_taget[4] = {  TRAJECTORY_J0, TRAJECTORY_J1,
												TRAJECTORY_J2, TRAJECTORY_J3};

const char *DETAIL_STATUS[NUM_OF_STATUS]  = {"Accept Command",
											 "Stupid Code",
											 "Wrong Space Type",
											 "Wrong Task Type",
											 "Wrong Joint Type",
											 "Wrong Trajectory Type",
											 "Wrong Parameters",
											 "Over Workspace",
											 "Wrong Mode Init",
											 "Over Velocity",
											 "Over Accelerate",
											 "Wrong Joint Num",
											 "Wrong Coordinate"
											};



SCARA_StatusTypeDef	scaraInitDuty		(DUTY_Command_TypeDef command) {
	SCARA_StatusTypeDef status, status1, status2;

	/*----------- Space Task ------------*/
	if 			(DUTY_SPACE_TASK == command.space_type) {
		float total_s, angle_s, dir_angle;
		SCARA_PositionTypeDef	target_point;

		if( DUTY_COORDINATES_REL == command.coordinate_type) {
			target_point.x 		= positionCurrent.x + command.target_point.x;
			target_point.y 		= positionCurrent.y + command.target_point.y;
			target_point.z 		= positionCurrent.z + command.target_point.z;
			target_point.roll	= positionCurrent.roll + command.target_point.roll;
		} else if (DUTY_COORDINATES_ABS == command.coordinate_type) {
			target_point.x 		= command.target_point.x;
			target_point.y 		= command.target_point.y;
			target_point.z 		= command.target_point.z;
			target_point.roll	= command.target_point.roll;
		} else {
			return SCARA_STATUS_ERROR_COORDINATE;
		}

		angle_s = target_point.roll - positionCurrent.roll;
		dir_angle = 1;
		if ( angle_s < 0) {
			dir_angle = -1;
		}
		if ( fabsf(angle_s) > PI) {
			dir_angle = -dir_angle;
			angle_s = (2*PI - fabsf(angle_s))*dir_angle;
		}


		myDUTY.space_type = DUTY_SPACE_TASK;// Change type
		myDUTY.task.roll_start = positionCurrent.roll;
		// 1-Path Planning
			// Straight Line
		if ( DUTY_PATH_LINE == command.path_type ) {
			myDUTY.task.path.path_type = DUTY_PATH_LINE;
			status = scaraInitLine(&(myDUTY.task.path.line), positionCurrent, target_point);
			total_s = myDUTY.task.path.line.total_s;
			// Circular
		} else if ( DUTY_PATH_CIRCLE == command.path_type ) {
			myDUTY.task.path.path_type = DUTY_PATH_CIRCLE;
			status = scaraInitCircle(&(myDUTY.task.path.circle),
										positionCurrent,
										target_point,
										command.sub_point,
										command.sub_para_int );
			total_s = myDUTY.task.path.circle.total_s;

		} else {
			return SCARA_STATUS_ERROR_TASK;
		}

		if ( SCARA_STATUS_OK != status) {
			return status;
		}

		// 2-Trajectory Planning
			// LSPB
		if ( DUTY_TRAJECTORY_LSPB == command.trajec_type ) {

			if ( DUTY_MODE_INIT_QVT == command.modeInit_type) {
				myDUTY.task.trajectory_3d.lspb.Tf = command.time_total;
				myDUTY.task.trajectory_3d.trajectory_type = DUTY_TRAJECTORY_LSPB;
				status1 = scaraInitLSPB(&(myDUTY.task.trajectory_3d.lspb), TRAJECTORY_3D,
						 total_s, DUTY_MODE_INIT_QVT);

				myDUTY.task.trajectory_roll.lspb.Tf = command.time_total;
				myDUTY.task.trajectory_roll.trajectory_type = DUTY_TRAJECTORY_LSPB;
				status2 = scaraInitLSPB(&(myDUTY.task.trajectory_roll.lspb), TRAJECTORY_ROLL,
						 angle_s, DUTY_MODE_INIT_QVT);
				myDUTY.time_total = command.time_total;

			} else if (DUTY_MODE_INIT_QVA == command.modeInit_type) {
				myDUTY.task.trajectory_3d.trajectory_type = DUTY_TRAJECTORY_LSPB;
				status1 = scaraInitLSPB(&(myDUTY.task.trajectory_3d.lspb), TRAJECTORY_3D,
						 total_s, DUTY_MODE_INIT_QVA);
				myDUTY.task.trajectory_roll.trajectory_type = DUTY_TRAJECTORY_LSPB;
				status2 = scaraInitLSPB(&(myDUTY.task.trajectory_roll.lspb), TRAJECTORY_ROLL,
						 angle_s, DUTY_MODE_INIT_QVA);
				// Synchronous time end
				if(myDUTY.task.trajectory_3d.lspb.Tf > myDUTY.task.trajectory_roll.lspb.Tf) {
					myDUTY.task.trajectory_roll.lspb.Tf = myDUTY.task.trajectory_3d.lspb.Tf;
					status2 = scaraInitLSPB(&(myDUTY.task.trajectory_roll.lspb), TRAJECTORY_ROLL,
							 angle_s, DUTY_MODE_INIT_QVT);
					myDUTY.time_total = myDUTY.task.trajectory_3d.lspb.Tf;
				} else {
					myDUTY.task.trajectory_3d.lspb.Tf = myDUTY.task.trajectory_roll.lspb.Tf;
					status1 = scaraInitLSPB(&(myDUTY.task.trajectory_3d.lspb), TRAJECTORY_3D,
							 total_s, DUTY_MODE_INIT_QVT);
					myDUTY.time_total = myDUTY.task.trajectory_roll.lspb.Tf;
				}
			} else {
				return SCARA_STATUS_ERROR_MODE_INIT ;
			}
			// SCURVE
		} else if 	( DUTY_TRAJECTORY_SCURVE == command.trajec_type ){

			if ( DUTY_MODE_INIT_QVT == command.modeInit_type) {
				myDUTY.task.trajectory_3d.scurve.Tf = command.time_total;
				myDUTY.task.trajectory_3d.trajectory_type = DUTY_TRAJECTORY_SCURVE;
				status1 = scaraInitScurve(&(myDUTY.task.trajectory_3d.scurve), TRAJECTORY_3D,
						 total_s, command.modeInit_type);

				myDUTY.task.trajectory_roll.scurve.Tf = command.time_total;
				myDUTY.task.trajectory_roll.trajectory_type = DUTY_TRAJECTORY_SCURVE;
				status2 = scaraInitScurve(&(myDUTY.task.trajectory_roll.scurve), TRAJECTORY_ROLL,
						 angle_s, command.modeInit_type);
				myDUTY.time_total = command.time_total;

			} else if (DUTY_MODE_INIT_QVA == command.modeInit_type) {
				myDUTY.task.trajectory_3d.trajectory_type = DUTY_TRAJECTORY_SCURVE;
				status1 = scaraInitScurve(&(myDUTY.task.trajectory_3d.scurve), TRAJECTORY_3D,
						 total_s, DUTY_MODE_INIT_QVA);
				myDUTY.task.trajectory_roll.trajectory_type = DUTY_TRAJECTORY_SCURVE;
				status2 = scaraInitScurve(&(myDUTY.task.trajectory_roll.scurve), TRAJECTORY_ROLL,
						 angle_s, DUTY_MODE_INIT_QVA);
				// synchronous time end
				if(myDUTY.task.trajectory_3d.scurve.Tf > myDUTY.task.trajectory_roll.scurve.Tf) {
					myDUTY.task.trajectory_roll.scurve.Tf = myDUTY.task.trajectory_3d.scurve.Tf;
					status2 = scaraInitScurve(&(myDUTY.task.trajectory_roll.scurve), TRAJECTORY_ROLL,
							 angle_s, DUTY_MODE_INIT_QVT);
					myDUTY.time_total = myDUTY.task.trajectory_3d.scurve.Tf;
				} else {
					myDUTY.task.trajectory_3d.scurve.Tf = myDUTY.task.trajectory_roll.scurve.Tf;
					status1 = scaraInitScurve(&(myDUTY.task.trajectory_3d.scurve), TRAJECTORY_3D,
							 total_s, DUTY_MODE_INIT_QVT);
					myDUTY.time_total = myDUTY.task.trajectory_roll.scurve.Tf;
				}
			} else {
				return SCARA_STATUS_ERROR_MODE_INIT;
			}
		} else {
			return SCARA_STATUS_ERROR_TRAJECTORY;
		}
		// Check Init Status
		if ( SCARA_STATUS_OK != status1) {
			return status1;
		}
		if ( SCARA_STATUS_OK != status2) {
			return status2;
		}

	/*----------- Space Joint ------------*/
	} else if (DUTY_SPACE_JOINT == command.space_type) {

		myDUTY.space_type = DUTY_SPACE_JOINT;// Change type
		myDUTY.joint.theta1_start	 = positionCurrent.Theta1;
		myDUTY.joint.theta2_start	 = positionCurrent.Theta2;
		myDUTY.joint.d3_start		 = positionCurrent.D3;
		myDUTY.joint.theta4_start 	 = positionCurrent.Theta4;
		//-----Joint Single
		if ( DUTY_JOINT_SINGLE == command.joint_type) {
			// Trajectory 1 profile
			float s, abs_position;
			switch(command.sub_para_int) {
			case 0:
				if( DUTY_COORDINATES_REL == command.coordinate_type) {
					abs_position = myDUTY.joint.theta1_start + command.sub_para_float;
					s			 = command.sub_para_float;
				} else if (DUTY_COORDINATES_ABS == command.coordinate_type) {
					abs_position = command.sub_para_float;
					s			 = command.sub_para_float - myDUTY.joint.theta1_start;
				} else {
					return SCARA_STATUS_ERROR_COORDINATE;
				}

				if( SCARA_STATUS_OK != scaraCheckWorkSpace1(TRAJECTORY_J0, abs_position)) {
					return SCARA_STATUS_ERROR_OVER_WORKSPACE;
				}
				break;
			case 1:
				if( DUTY_COORDINATES_REL == command.coordinate_type) {
					abs_position = myDUTY.joint.theta2_start + command.sub_para_float;
					s			 = command.sub_para_float;
				} else if (DUTY_COORDINATES_ABS == command.coordinate_type) {
					abs_position = command.sub_para_float;
					s			 = command.sub_para_float - myDUTY.joint.theta2_start;
				} else {
					return SCARA_STATUS_ERROR_COORDINATE;
				}

				if( SCARA_STATUS_OK != scaraCheckWorkSpace1(TRAJECTORY_J1, abs_position)) {
					return SCARA_STATUS_ERROR_OVER_WORKSPACE;
				}
				break;
			case 2:
				if( DUTY_COORDINATES_REL == command.coordinate_type) {
					abs_position = myDUTY.joint.d3_start + command.sub_para_float;
					s			 = command.sub_para_float;
				} else if (DUTY_COORDINATES_ABS == command.coordinate_type) {
					abs_position = command.sub_para_float;
					s			 = command.sub_para_float - myDUTY.joint.d3_start;
				} else {
					return SCARA_STATUS_ERROR_COORDINATE;
				}

				if( SCARA_STATUS_OK != scaraCheckWorkSpace1(TRAJECTORY_J2, abs_position)) {
					return SCARA_STATUS_ERROR_OVER_WORKSPACE;
				}
				break;
			case 3:
				if( DUTY_COORDINATES_REL == command.coordinate_type) {
					abs_position = myDUTY.joint.theta4_start + command.sub_para_float;
					s			 = command.sub_para_float;
				} else if (DUTY_COORDINATES_ABS == command.coordinate_type) {
					abs_position = command.sub_para_float;
					s			 = command.sub_para_float - myDUTY.joint.theta4_start;
				} else {
					return SCARA_STATUS_ERROR_COORDINATE;
				}

				if( SCARA_STATUS_OK != scaraCheckWorkSpace1(TRAJECTORY_J3, abs_position)) {
					return SCARA_STATUS_ERROR_OVER_WORKSPACE;
				}
				break;
			default:
				return SCARA_STATUS_ERROR_JOINT_NUM;
			}

			// LSPB
			if ( DUTY_TRAJECTORY_LSPB == command.trajec_type) {
				// Mode Init Time
				if ( DUTY_MODE_INIT_QVT == command.modeInit_type) {
					for ( uint8_t i = 0; i < 4; i++) {
						if ( i == command.sub_para_int) {
							myDUTY.joint.trajectory[i].trajectory_type = DUTY_TRAJECTORY_LSPB;
							myDUTY.joint.trajectory[i].lspb.Tf = command.time_total;
							status1 = scaraInitLSPB(&(myDUTY.joint.trajectory[i].lspb), joint_taget[i],
											s, DUTY_MODE_INIT_QVT);
						} else {
							myDUTY.joint.trajectory[i].trajectory_type = DUTY_TRAJECTORY_LSPB;
							myDUTY.joint.trajectory[i].lspb.Tf = command.time_total;
							status2 = scaraInitLSPB(&(myDUTY.joint.trajectory[i].lspb), joint_taget[i],
											0, DUTY_MODE_INIT_QVT);
						}
						myDUTY.time_total = command.time_total;
					}
				// Mode Init Acc
				} else if  ( DUTY_MODE_INIT_QVA == command.modeInit_type) {
					for ( uint8_t i = 0; i < 4; i++) {
						if ( i == command.sub_para_int) {
							myDUTY.joint.trajectory[i].trajectory_type = DUTY_TRAJECTORY_LSPB;
							status1 = scaraInitLSPB(&(myDUTY.joint.trajectory[i].lspb), joint_taget[i],
											s, DUTY_MODE_INIT_QVA);
							myDUTY.time_total = myDUTY.joint.trajectory[i].lspb.Tf;
						} else {
							myDUTY.joint.trajectory[i].trajectory_type = DUTY_TRAJECTORY_LSPB;
							status2 = scaraInitLSPB(&(myDUTY.joint.trajectory[i].lspb), joint_taget[i],
											0, DUTY_MODE_INIT_QVA);
						}
					}
				} else {
					return SCARA_STATUS_ERROR_MODE_INIT;
				}
			// SCURVE
			} else if ( DUTY_TRAJECTORY_SCURVE == command.trajec_type) {
					// Mode Init Time
					if ( DUTY_MODE_INIT_QVT == command.modeInit_type) {
						for ( uint8_t i = 0; i < 4; i++) {
							if ( i == command.sub_para_int) {
								myDUTY.joint.trajectory[i].trajectory_type = DUTY_TRAJECTORY_SCURVE;
								myDUTY.joint.trajectory[i].scurve.Tf = command.time_total;
								status1 = scaraInitScurve(&(myDUTY.joint.trajectory[i].scurve), joint_taget[i],
												s, DUTY_MODE_INIT_QVT);
							} else {
								myDUTY.joint.trajectory[i].trajectory_type = DUTY_TRAJECTORY_SCURVE;
								myDUTY.joint.trajectory[i].scurve.Tf = command.time_total;
								status2 = scaraInitScurve(&(myDUTY.joint.trajectory[i].scurve), joint_taget[i],
												0, DUTY_MODE_INIT_QVT);
							}
							myDUTY.time_total = command.time_total;
						}
					// Mode Init Acc
					} else if  ( DUTY_MODE_INIT_QVA == command.modeInit_type) {
						for ( uint8_t i = 0; i < 4; i++) {
							if ( i == command.sub_para_int) {
								myDUTY.joint.trajectory[i].trajectory_type = DUTY_TRAJECTORY_SCURVE;
								status1 = scaraInitScurve(&(myDUTY.joint.trajectory[i].scurve), joint_taget[i],
												s, DUTY_MODE_INIT_QVA);
								myDUTY.time_total = command.time_total;
							} else {
								myDUTY.joint.trajectory[i].trajectory_type = DUTY_TRAJECTORY_SCURVE;
								status2 = scaraInitScurve(&(myDUTY.joint.trajectory[i].scurve), joint_taget[i],
												0, DUTY_MODE_INIT_QVA);
							}
						}

					} else {
						return SCARA_STATUS_ERROR_MODE_INIT;
					}
			} else {
				return SCARA_STATUS_ERROR_TRAJECTORY;
			}
		// Check Init Status
			if ( SCARA_STATUS_OK != status1) {
				return status1;
			}
			if ( SCARA_STATUS_OK != status2) {
				return status2;
			}

		//----Joint Quatra
		} else if 	( DUTY_JOINT_4DOF == command.joint_type) {
			SCARA_PositionTypeDef	target_point;

			if( DUTY_COORDINATES_REL == command.coordinate_type) {
				target_point.x 		= positionCurrent.x + command.target_point.x;
				target_point.y 		= positionCurrent.y + command.target_point.y;
				target_point.z 		= positionCurrent.z + command.target_point.z;
				target_point.roll	= positionCurrent.roll + command.target_point.roll;
			} else if (DUTY_COORDINATES_ABS == command.coordinate_type) {
				target_point.x 		= command.target_point.x;
				target_point.y 		= command.target_point.y;
				target_point.z 		= command.target_point.z;
				target_point.roll	= command.target_point.roll;
			} else {
				return SCARA_STATUS_ERROR_COORDINATE;
			}

			if( FALSE == kinematicInverse(&target_point, positionCurrent)) {
				return SCARA_STATUS_ERROR_OVER_WORKSPACE;// Exit with error
			}
			// Trajectory 4 profile
			float q[4];
			q[0] = target_point.Theta1 - positionCurrent.Theta1;
			q[1] = target_point.Theta2 - positionCurrent.Theta2;
			q[2] = target_point.D3 	   - positionCurrent.D3;
			q[3] = target_point.Theta4 - positionCurrent.Theta4;



			// LSPB
			if ( DUTY_TRAJECTORY_LSPB == command.trajec_type) {
				// Mode Init Time
				if ( DUTY_MODE_INIT_QVT == command.modeInit_type) {
					for ( uint8_t i = 0; i < 4; i++) {
						myDUTY.joint.trajectory[i].trajectory_type = DUTY_TRAJECTORY_LSPB;
						myDUTY.joint.trajectory[i].lspb.Tf = command.time_total;
						status1 = scaraInitLSPB(&(myDUTY.joint.trajectory[i].lspb), joint_taget[i],
											q[i], DUTY_MODE_INIT_QVT);

					}
					myDUTY.time_total = command.time_total;
				// Mode Init Acc
				} else if  ( DUTY_MODE_INIT_QVA == command.modeInit_type) {
					for ( uint8_t i = 0; i < 4; i++) {
						myDUTY.joint.trajectory[i].trajectory_type = DUTY_TRAJECTORY_LSPB;
						status1 = scaraInitLSPB(&(myDUTY.joint.trajectory[i].lspb), joint_taget[i],
													q[i], DUTY_MODE_INIT_QVA);
						myDUTY.time_total = command.time_total;
					}
					myDUTY.time_total = 0;
					for ( uint8_t i = 0; i < 4; i++) {
						if ( myDUTY.joint.trajectory[i].lspb.Tf > myDUTY.time_total) {
							myDUTY.time_total = myDUTY.joint.trajectory[i].lspb.Tf;
						}
					}
					for ( uint8_t i = 0; i < 4; i++) {
						myDUTY.joint.trajectory[i].trajectory_type = DUTY_TRAJECTORY_LSPB;
						myDUTY.joint.trajectory[i].lspb.Tf = myDUTY.time_total;
						status1 = scaraInitLSPB(&(myDUTY.joint.trajectory[i].lspb), joint_taget[i],
													q[i], DUTY_MODE_INIT_QVT);
					}

				} else {
					return SCARA_STATUS_ERROR_MODE_INIT;
				}
				// SCURVE
			} else if ( DUTY_TRAJECTORY_SCURVE == command.trajec_type) {
				// Mode Init Time
				if ( DUTY_MODE_INIT_QVT == command.modeInit_type) {
					for ( uint8_t i = 0; i < 4; i++) {
						myDUTY.joint.trajectory[i].trajectory_type = DUTY_TRAJECTORY_SCURVE;
						myDUTY.joint.trajectory[i].scurve.Tf = command.time_total;
						status1 = scaraInitScurve(&(myDUTY.joint.trajectory[i].scurve), joint_taget[i],
											q[i], DUTY_MODE_INIT_QVT);

					}
					myDUTY.time_total = command.time_total;
				// Mode Init Acc
				} else if  ( DUTY_MODE_INIT_QVA == command.modeInit_type) {
					for ( uint8_t i = 0; i < 4; i++) {
						myDUTY.joint.trajectory[i].trajectory_type = DUTY_TRAJECTORY_SCURVE;
						status1 = scaraInitScurve(&(myDUTY.joint.trajectory[i].scurve), joint_taget[i],
													q[i], DUTY_MODE_INIT_QVA);
					}
					myDUTY.time_total = 0;
					for ( uint8_t i = 0; i < 4; i++) {
						if ( myDUTY.joint.trajectory[i].lspb.Tf > myDUTY.time_total) {
							myDUTY.time_total = myDUTY.joint.trajectory[i].lspb.Tf;
						}
					}
					for ( uint8_t i = 0; i < 4; i++) {
						myDUTY.joint.trajectory[i].trajectory_type = DUTY_TRAJECTORY_SCURVE;
						myDUTY.joint.trajectory[i].scurve.Tf = myDUTY.time_total;
						status1 = scaraInitScurve(&(myDUTY.joint.trajectory[i].scurve), joint_taget[i],
													q[i], DUTY_MODE_INIT_QVT);
					}

				} else {
					return SCARA_STATUS_ERROR_MODE_INIT;
				}

			} else {
				return SCARA_STATUS_ERROR_TRAJECTORY;
			}

		} else {
			return SCARA_STATUS_ERROR_JOINT;
		}

	} else {
		return SCARA_STATUS_ERROR_SPACE;
	}

	return SCARA_STATUS_OK;
}

SCARA_StatusTypeDef			scaraInitLine		(Path_Line_TypeDef *line,
												SCARA_PositionTypeDef start,
												SCARA_PositionTypeDef end) {
	// Check limit workspace
	if ( FALSE ==  kinematicInverse(&end, start)) {
		return SCARA_STATUS_ERROR_OVER_WORKSPACE;
	}
	// Init line params
	line->x1 = end.x;
	line->y1 = end.y;
	line->z1 = end.z;
	line->x0 = start.x;
	line->y0 = start.y;
	line->z0 = start.z;
	line->denta_x = end.x  -  start.x;
	line->denta_y = end.y  -  start.y;
	line->denta_z = end.z  -  start.z;
	line->total_s = sqrtf((line->denta_x)*(line->denta_x)
			       +(line->denta_y)*(line->denta_y)
				   +(line->denta_z)*(line->denta_z));
	line->x_current = line->x0;
	line->y_current = line->y0;
	line->z_current = line->z0;

	return SCARA_STATUS_OK;
}

SCARA_StatusTypeDef			scaraInitCircle		(Path_Circle_TypeDef *circle,
												SCARA_PositionTypeDef start,
												SCARA_PositionTypeDef end,
												SCARA_PositionTypeDef center,
												int32_t dir){
	// Check limit workspace
	if ( FALSE == kinematicInverse(&end, start) ) {
		return SCARA_STATUS_ERROR_OVER_WORKSPACE;
	}

	if( 1 != dir && -1 != dir) {
		return SCARA_STATUS_ERROR_PARA;// error direction param !!!
	}

	float v_x_start, v_y_start, v_x_stop, v_y_stop;
	float r1, r2, angle_start, angle_stop, delta_angle;
	v_x_start = start.x - center.x;
	v_y_start = start.y - center.y;
	v_x_stop  = end.x  - center.x;
	v_y_stop  = end.y  - center.y;
	r1 = sqrtf(v_x_start*v_x_start + v_y_start*v_y_start);
	r2 = sqrtf(v_x_stop*v_x_stop + v_y_stop*v_y_stop);

	if( 1.0 < fabsf(r1 - r2)) {
		return SCARA_STATUS_ERROR_PARA; // 2 point start & stop are not in a circle together !!
	}

	angle_start = atan2f(v_y_start, v_x_start);
	angle_stop  = atan2f(v_y_stop, v_x_stop);
	delta_angle = angle_stop - angle_start;

	if ( 0 > delta_angle) {
		delta_angle += 2*PI;
	} // atan2 range : -PI --> PI

	if ( 0 > dir) {
		delta_angle = 2*PI - delta_angle;
	}
	// Init circle params
	circle->dir = dir;
	circle->radius = r1;
	circle->angle_start = angle_start;
	circle->angle_stop  = angle_stop;
	circle->total_angle = delta_angle;
	circle->total_s = delta_angle*r1;
	circle->x0 = start.x;
	circle->y0 = start.y;
	circle->x1 = end.x;
	circle->y1 = end.y;
	circle->xi = center.x;
	circle->yi = center.y;
	circle->z0 = start.z;
	circle->z1 = start.z;
	circle->zi = start.z;
	circle->x_current = circle->x0;
	circle->y_current = circle->y0;
	circle->z_current = circle->z0;

	return SCARA_STATUS_OK;
}

SCARA_StatusTypeDef	scaraInitLSPB	(Trajectory_LSPB_TypeDef *lspb,
									Trajectory_TargetTypeDef target,
									float total_s,
									ModeInitTypeDef modeinit) {
	 float v_design, a_design, v_lim, q0, q1, v0, v1, ta, td, tf;
	 uint32_t	no_sample;
	 int8_t	dir;

	 if ( TRAJECTORY_3D == target) {
		 v_design = V_DESIGN_3D;
		 a_design = A_DESIGN_3D;
	 } else if (TRAJECTORY_ROLL == target) {
		 v_design = V_DESIGN_ROLL;
		 a_design = A_DESIGN_ROLL;
	 } else if (TRAJECTORY_J0 == target) {
		 v_design = V_DESIGN_J0;
		 a_design = A_DESIGN_J0;
	 } else if (TRAJECTORY_J1 == target) {
		 v_design = V_DESIGN_J1;
		 a_design = A_DESIGN_J1;
	 } else if (TRAJECTORY_J2 == target) {
		 v_design = V_DESIGN_J2;
		 a_design = A_DESIGN_J2;
	 } else if (TRAJECTORY_J3 == target) {
		 v_design = V_DESIGN_J3;
		 a_design = A_DESIGN_J3;
	 } else {
		 return SCARA_STATUS_ERROR_PARA;
	 }

	 q0 = 0;
	 q1 = total_s;
	 v0 = 0;
	 v1 = 0;

	 if ( q0 <= q1) {
		 dir = 1;
	 } else {
		 dir = -1;
		 q1 = -q1;
	 }

	 if (DUTY_MODE_INIT_QVT == modeinit) {
		 float v_lower, v_upper, tc_upper, a_upper;
		 tf = lspb->Tf;
		 v_lower 	= (q1 - q0) / tf;
		 v_upper 	= 2*(q1 - q0) / tf;

		 if ( v_design < v_lower) {
			 return SCARA_STATUS_ERROR_OVER_VELOC;
		 } else {

			 if ( v_upper <= v_design) {
				 v_design = v_upper;
			 }

			 tc_upper	= tf - (q1 - q0)/v_design;
			 a_upper	= v_design/tc_upper;

			 if ( a_upper > a_design) {
				 return SCARA_STATUS_ERROR_OVER_ACCEL;
			 } else {
				 a_design = a_upper;
			 }
		 }
	 }

	 // Check condition trapezoidal ---> triangle
	 if ( (fabsf(q1 - q0)*a_design) <= (v_design*v_design - (v0*v0 + v1*v1)/2)) {
		 v_lim 	= sqrtf(fabsf(q1 - q0)*a_design + (v0*v0 + v1*v1)/2);
		 ta		= (v_lim - v0)/a_design;
		 td		= (v_lim - v1)/a_design;
		 tf		= ta + td;
	 } else {
		 v_lim	= v_design;
		 ta		= (v_lim - v0)/a_design;
		 td		= (v_lim - v1)/a_design;
		 tf		= fabsf(q0 - q1)/v_lim + v_lim/(2*a_design)*(1 - v0/v_lim)*(1 - v0/v_lim)
					 	+ v_lim/(2*a_design)*(1 - v1/v_lim)*(1 - v1/v_lim);
	 }

	 no_sample = ceilf(tf / T_SAMPLING); // ceiling
	 // Init lspb params
	 lspb->dir= dir;
	 lspb->s0 = q0;
	 lspb->s1 = q1;
	 lspb->Ta = ta;
	 lspb->Td = td;
	 lspb->Tf = tf;
	 lspb->a_design = a_design;
	 lspb->v_design = v_design;
	 lspb->v_lim = v_lim;
	 lspb->v0 = v0;
	 lspb->v1 = v1;
	 lspb->num_of_sampling = no_sample;
	 lspb->total_s = lspb->s1 - lspb->s0;

	 return SCARA_STATUS_OK;
}

SCARA_StatusTypeDef	scaraInitScurve	(Trajectory_Scurve_TypeDef *scurve,
									Trajectory_TargetTypeDef target,
									float total_s,
									ModeInitTypeDef modeinit) {
	 float v_design, a_design, q0, q1, v0, v1, v_lim, j_max, tm, tc, tf, dir;
	 float v_1, s_1, v_2, s_2, v_3, s_3, v_4, s_4;
	 uint32_t	 no_sample;
	 uint8_t	 no_phases;

	 if ( TRAJECTORY_3D == target) {
		 v_design = V_DESIGN_3D;
		 a_design = A_DESIGN_3D;
	 } else if (TRAJECTORY_J0 == target) {
		 v_design = V_DESIGN_J0;
		 a_design = A_DESIGN_J0;
	 } else if (TRAJECTORY_J1 == target) {
		 v_design = V_DESIGN_J1;
		 a_design = A_DESIGN_J1;
	 } else if (TRAJECTORY_J2 == target) {
		 v_design = V_DESIGN_J2;
		 a_design = A_DESIGN_J2;
	 } else if (TRAJECTORY_J3 == target) {
		 v_design = V_DESIGN_J3;
		 a_design = A_DESIGN_J3;
	 } else {
		 return SCARA_STATUS_ERROR_PARA;
	 }

	 v0 = 0;
	 v1 = 0;
	 q0 = 0;
	 q1 = total_s;

	 if ( q1 < q0) {
		 dir = -1;
		 q1 = -q1;

	 } else {
		 dir = 1;
	 }


	 if (DUTY_MODE_INIT_QVT == modeinit) {
		 float t_upper, v_upper, a_upper;

		 tf = scurve->Tf;
		 t_upper = tf / 4;

		 v_upper = (q1 - q0) / t_upper;
		 if ( v_upper > v_design) {
			 return SCARA_STATUS_ERROR_OVER_VELOC;
		 } else {
			 v_design = v_upper;
		 }

		 a_upper = v_design / t_upper;
		 if ( a_upper > a_design) {
			 return SCARA_STATUS_ERROR_OVER_ACCEL;
		 } else {
			 a_design = a_upper;
		 }
	 }

	 no_phases = 5;
	 tm = v_design/a_design;
	 tc = total_s/v_design - 2*tm;
	 // Check condition 4 phase
	 if( 0.0 > tc) {
		 tc = 0;
		 no_phases = 4;
		 tm = sqrtf(total_s/(2*a_design));
	 }
	 if ( 0.001 > tm) {
		 j_max = 0;
	 } else {
		 j_max = a_design/tm;
	 }
	 v_lim = a_design*tm;
	 tf    = 4*tm + tc;
	 no_sample = ceilf(tf / T_SAMPLING);

	 v_1 = 0.5*j_max*tm*tm;
	 s_1 = j_max*tm*tm*tm/6;
	 v_2 = j_max*tm*tm;
	 s_2 = v_2*tm;
	 v_3 = v_2;
	 s_3 = s_2 + v_2*tc;
	 v_4 = v_1;
	 s_4 = s_3 + s_2 - s_1;

	 scurve->Tc = tc;
	 scurve->Tf = tf;
	 scurve->Tm = tm;
	 scurve->a_design = a_design;
	 scurve->v_design = v_design;
	 scurve->v_lim = v_lim;
	 scurve->v0 = v0;
	 scurve->v1 = v1;
	 scurve->j_max = j_max;
	 scurve->s0   = q0;
	 scurve->s1   = q1;
	 scurve->num_of_phase = no_phases;
	 scurve->num_of_sampling = no_sample;
	 scurve->total_s = total_s;
	 scurve->dir = dir;

	 scurve->a_current = 0;
	 scurve->v_current = 0;
	 scurve->s_current = 0;
	 scurve->v_1 = v_1;
	 scurve->v_2 = v_2;
	 scurve->v_3 = v_3;
	 scurve->v_4 = v_4;
	 scurve->s_1 = s_1;
	 scurve->s_2 = s_2;
	 scurve->s_3 = s_3;
	 scurve->s_4 = s_4;

	 return SCARA_STATUS_OK;
}

SCARA_StatusTypeDef	scaraFlowDuty		(float time) {
	SCARA_StatusTypeDef status1, status2, status3, status4;
	SCARA_PositionTypeDef	positionCompute;
	/*---- Task space ----*/
	if ( DUTY_SPACE_TASK == myDUTY.space_type) {
		float s, angle, x, y, z;
		//---Trajectory flowing
			// LSPB
		if( DUTY_TRAJECTORY_LSPB == myDUTY.task.trajectory_3d.trajectory_type) {
			status1 = scaraFlowLSPB(&(myDUTY.task.trajectory_3d.lspb), time);
			status2 = scaraFlowLSPB(&(myDUTY.task.trajectory_roll.lspb), time);
			s = myDUTY.task.trajectory_3d.lspb.s_current;
			angle = myDUTY.task.trajectory_roll.lspb.s_current;
			// SCURVE
		} else if ( DUTY_TRAJECTORY_SCURVE == myDUTY.task.trajectory_3d.trajectory_type) {
			status1 = scaraFLowScurve(&(myDUTY.task.trajectory_3d.scurve), time);
			status2 = scaraFLowScurve(&(myDUTY.task.trajectory_roll.scurve), time);
			s = myDUTY.task.trajectory_3d.scurve.s_current;
			angle = myDUTY.task.trajectory_roll.scurve.s_current;

		} else {
			return SCARA_STATUS_ERROR_TRAJECTORY;
		}

		if ( SCARA_STATUS_OK != status1) {
			return status1;
		}
		if ( SCARA_STATUS_OK != status2) {
			return status2;
		}

		//---Path flowing
			// Straight line
		if( DUTY_PATH_LINE == myDUTY.task.path.path_type) {
			status1 = scaraFlowLine(&(myDUTY.task.path.line), s);
			x = myDUTY.task.path.line.x_current;
			y = myDUTY.task.path.line.y_current;
			z = myDUTY.task.path.line.z_current;
			// Circular
		} else if ( DUTY_PATH_CIRCLE == myDUTY.task.path.path_type) {
			status1 = scaraFlowCircle(&(myDUTY.task.path.circle), s);
			x = myDUTY.task.path.circle.x_current;
			y = myDUTY.task.path.circle.y_current;
			z = myDUTY.task.path.circle.z_current;
		} else {
			return SCARA_STATUS_ERROR_TASK;
		}

		positionCompute.x 		= x;
		positionCompute.y		= y;
		positionCompute.z 		= z;
		positionCompute.roll 	= myDUTY.task.roll_start + angle;

		if ( FALSE == kinematicInverse(&positionCompute, positionCurrent)) {
			return SCARA_STATUS_ERROR_OVER_WORKSPACE;
		} else {
			memcpy(&positionNext, &positionCompute, sizeof(SCARA_PositionTypeDef));
		}

	/*---- Joint space -----*/
	} else if (DUTY_SPACE_JOINT == myDUTY.space_type) {
		float s0, s1, s2, s3;
		int8_t dir0, dir1, dir2, dir3;
		// Trajectory flowing
			// LSPB
		if( DUTY_TRAJECTORY_LSPB == myDUTY.joint.trajectory[0].trajectory_type) {
			status1 = scaraFlowLSPB(&(myDUTY.joint.trajectory[0].lspb), time);
			status2 = scaraFlowLSPB(&(myDUTY.joint.trajectory[1].lspb), time);
			status3 = scaraFlowLSPB(&(myDUTY.joint.trajectory[2].lspb), time);
			status4 = scaraFlowLSPB(&(myDUTY.joint.trajectory[3].lspb), time);
			dir0 = myDUTY.joint.trajectory[0].lspb.dir;
			dir1 = myDUTY.joint.trajectory[1].lspb.dir;
			dir2 = myDUTY.joint.trajectory[2].lspb.dir;
			dir3 = myDUTY.joint.trajectory[3].lspb.dir;
			s0 = myDUTY.joint.trajectory[0].lspb.s_current;
			s1 = myDUTY.joint.trajectory[1].lspb.s_current;
			s2 = myDUTY.joint.trajectory[2].lspb.s_current;
			s3 = myDUTY.joint.trajectory[3].lspb.s_current;

			// SCURVE
		} else if ( DUTY_TRAJECTORY_SCURVE == myDUTY.joint.trajectory[0].trajectory_type) {
			status1 = scaraFLowScurve(&(myDUTY.joint.trajectory[0].scurve), time);
			status2 = scaraFLowScurve(&(myDUTY.joint.trajectory[1].scurve), time);
			status3 = scaraFLowScurve(&(myDUTY.joint.trajectory[2].scurve), time);
			status4 = scaraFLowScurve(&(myDUTY.joint.trajectory[3].scurve), time);
			dir0 = myDUTY.joint.trajectory[0].scurve.dir;
			dir1 = myDUTY.joint.trajectory[1].scurve.dir;
			dir2 = myDUTY.joint.trajectory[2].scurve.dir;
			dir3 = myDUTY.joint.trajectory[3].scurve.dir;
			s0 = myDUTY.joint.trajectory[0].scurve.s_current;
			s1 = myDUTY.joint.trajectory[1].scurve.s_current;
			s2 = myDUTY.joint.trajectory[2].scurve.s_current;
			s3 = myDUTY.joint.trajectory[3].scurve.s_current;
		} else {
			return SCARA_STATUS_ERROR_TRAJECTORY;
		}
		// Check init status
		if ( SCARA_STATUS_OK != status1) {
			return status1;
		}
		if ( SCARA_STATUS_OK != status2) {
			return status2;
		}
		if ( SCARA_STATUS_OK != status3) {
			return status3;
		}
		if ( SCARA_STATUS_OK != status4) {
			return status4;
		}

		positionCompute.Theta1 	= myDUTY.joint.theta1_start + s0*dir0;
		positionCompute.Theta2 	= myDUTY.joint.theta2_start + s1*dir1;
		positionCompute.D3 		= myDUTY.joint.d3_start 	+ s2*dir2;
		positionCompute.Theta4 	= myDUTY.joint.theta4_start + s3*dir3;
		// Check workspace
		if( SCARA_STATUS_OK != scaraCheckWorkSpace4(positionCompute.Theta1,
							 	 	 	  positionCompute.Theta2,
										  positionCompute.D3,
										  positionCompute.Theta4)) {
			return SCARA_STATUS_ERROR_OVER_WORKSPACE;
		} else {
			memcpy(&positionNext, &positionCompute, sizeof(SCARA_PositionTypeDef));
		}
		kinematicForward(&positionNext);

	} else {
		return SCARA_STATUS_ERROR_SPACE;
	}

	return SCARA_STATUS_OK;
}

SCARA_StatusTypeDef	scaraFlowLine		(Path_Line_TypeDef *line, float s) {
	line->x_current	 = line->x0 + line->denta_x*s/line->total_s;
	line->y_current	 = line->y0 + line->denta_y*s/line->total_s;
	line->z_current	 = line->z0 + line->denta_z*s/line->total_s;

	return SCARA_STATUS_OK;
}

SCARA_StatusTypeDef	scaraFlowCircle		(Path_Circle_TypeDef *circle, float s) {
	float angle;
	angle = s/(2*PI*circle->radius);
	circle->x_current = circle->xi + circle->radius*cosf(circle->angle_start + circle->dir*angle);
	circle->y_current = circle->yi + circle->radius*sinf(circle->angle_start + circle->dir*angle);
	circle->z_current = circle->zi; // XY plane

	return SCARA_STATUS_OK;
}

SCARA_StatusTypeDef	scaraFlowLSPB		(Trajectory_LSPB_TypeDef *lspb, float time) {
	float tf, td, ta;

	tf = lspb->Tf;
	td = lspb->Td;
	ta = lspb->Ta;

	// Accelerate
	if ( 0.0f <= time && time <= ta) {
		lspb->a_current		=	lspb->a_design;
		lspb->v_current		=	lspb->v0 + lspb->a_design*time;
		lspb->s_current		=	lspb->s0 + lspb->v0*time + 0.5*lspb->a_design*time*time;
	// Constant velocity
	} else if (ta <= time && time <= (tf - td)) {
		lspb->a_current		=	0;
		lspb->v_current		=	lspb->v_lim;
		lspb->s_current		=	lspb->s0 + lspb->v0*ta/2 + lspb->v_lim*(time - ta/2);
	// Decelerate
	} else if ((tf - td) <= time && time <= tf) {
		lspb->a_current		=	-lspb->a_design;
		lspb->v_current		=	lspb->v1 + lspb->a_design*(tf - time);
		lspb->s_current		=	lspb->s1 - lspb->v1*(tf - time)
								- (lspb->v_lim - lspb->v1)*(tf - time)*(tf -time)/(2*td);
	} else {
		lspb->a_current 	=	0;
		lspb->v_current		=	0;
		lspb->s_current		=	lspb->total_s;
	}

	return SCARA_STATUS_OK;
}

SCARA_StatusTypeDef	scaraFLowScurve		(Trajectory_Scurve_TypeDef *scurve, float time) {
	float j_max, tm, tc, t;

	j_max = scurve->j_max;
	tm	  = scurve->Tm;
	tc	  = scurve->Tc;

	 // SCURVE 4 PHASE: tc = 0;
	 if ( 4 == scurve->num_of_phase) {
		 // Phase 1
		 if ( (0.0f <= time) && ( time <= tm)) {
			 t = time;
			 scurve->a_current 	= 	j_max*t;
			 scurve->v_current	= 	0.5*j_max*t*t;
			 scurve->s_current 	= 	j_max*t*t*t/6;
		 // Phase 2
		 } else if ( (tm <= time) && ( time <= (2*tm) ) ) {
			 t = time - tm;
			 scurve->a_current 	= 	j_max*tm - j_max*t;
			 scurve->v_current 	= 	scurve->v_1 + j_max*tm*t - 0.5*j_max*t*t;
			 scurve->s_current 	=	scurve->s_1 + scurve->v_1*t + 0.5*j_max*tm*t*t - j_max*t*t*t/6;
		 // Phase 3: does not exist
		 // Phase 4
		 } else if ( ((2*tm) <= time) && ( time <= (3*tm)) ) {
			 t = time - 2*tm;
			 scurve->a_current 	=	-j_max*t;
			 scurve->v_current	=	scurve->v_2 - 0.5*j_max*t*t;
			 scurve->s_current	=	scurve->s_2 + scurve->v_2*t - j_max*t*t*t/6;
		 // Phase 4
		 } else if ( ((3*tm) <= time) && ( time <= (4*tm)) ) {
			 t = time - 3*tm;
			 scurve->a_current	=	-j_max*tm + j_max*t;
			 scurve->v_current	=	scurve->v_4 - j_max*tm*t + 0.5*j_max*t*t;
			 scurve->s_current	=	scurve->s_4 + scurve->v_4*t
									- 0.5*j_max*tm*t*t
									+ j_max*t*t*t/6;
		 } else {
			 scurve->a_current = 0;
			 scurve->v_current = 0;
			 scurve->s_current = scurve->total_s;
		 }

	 // SCURVE 5 PHASE: tc > 0
	 } else if ( 5 == scurve->num_of_phase) {
		 // Phase 1
		 if ( (0 <= time) && ( time <= tm)) {
			 t = time;
			 scurve->a_current 	= 	j_max*t;
			 scurve->v_current	= 	0.5*j_max*t*t;
			 scurve->s_current 	= 	j_max*t*t*t/6;
		 // Phase 2
		 } else if ( (tm <= time) && ( time <= (2*tm) ) ) {
			 t = time - tm;
			 scurve->a_current 	= 	j_max*tm - j_max*t;
			 scurve->v_current 	= 	scurve->v_1 + j_max*tm*t - 0.5*j_max*t*t;
			 scurve->s_current 	=	scurve->s_1 + scurve->v_1*t + 0.5*j_max*tm*t*t - j_max*t*t*t/6;
		 // Phase 3
		 } else if ( ((2*tm) <= time) && ( time <= (2*tm + tc)) ) {
			 t = time - 2*tm;
			 scurve->a_current 	=	0;
			 scurve->v_current	=	scurve->v_2;
			 scurve->s_current	=	scurve->s_2 + scurve->v_2*t;
		 // Phase 4
		 } else if ( ((2*tm + tc) <= time) && ( time <= (3*tm + tc)) ) {
			 t = time - (2*tm + tc);
			 scurve->a_current 	=	-j_max*(t - (2*tm + tc));
			 scurve->v_current	=	scurve->v_3 - 0.5*j_max*t*t;
			 scurve->s_current	=	scurve->s_3 + scurve->v_2*t - j_max*t*t*t/6;
		 // Phase 5
		 } else if ( ((3*tm + tc) <= time) && ( time <= (4*tm + tc)) ) {
			 t = time - (3*tm + tc);
			 scurve->a_current	=	-j_max*tm + j_max*t;
			 scurve->v_current	=	scurve->v_4 - j_max*tm*t + 0.5*j_max*t*t;
			 scurve->s_current	=	scurve->s_4 + scurve->v_4*t - 0.5*j_max*tm*t + j_max*t*t*t/6;
		 } else {
			 scurve->a_current = 0;
			 scurve->v_current = 0;
			 scurve->s_current = scurve->total_s;
		 }

	 } else {
		 return SCARA_STATUS_ERROR;
	 }

	 return SCARA_STATUS_OK;
}

SCARA_StatusTypeDef	scaraCheckWorkSpace4 (float theta1, float theta2, float d3, float theta4) {
	// check theta 1
	if ( theta1 < LIM_MIN_J0 || theta1 > LIM_MAX_J0) {
		return SCARA_STATUS_ERROR_OVER_WORKSPACE;
	}
	// check theta 2
	if ( theta2 < LIM_MIN_J1 || theta2 > LIM_MAX_J1) {
		return SCARA_STATUS_ERROR_OVER_WORKSPACE;
	}
	// check d 3
	if ( d3 < LIM_MIN_J2 || d3 > LIM_MAX_J2) {
		return SCARA_STATUS_ERROR_OVER_WORKSPACE;
	}
	// check theta 4
	if ( theta4 < LIM_MIN_J3 || theta4 > LIM_MAX_J3) {
		return SCARA_STATUS_ERROR_OVER_WORKSPACE;
	}
	return SCARA_STATUS_OK;
}

SCARA_StatusTypeDef			scaraCheckWorkSpace1 (Trajectory_TargetTypeDef target, float value) {
	if ( TRAJECTORY_J0 == target) {
		if ( (LIM_MIN_J0 <= value) && ( value <= LIM_MAX_J0)) {
			return SCARA_STATUS_OK;
		} else {
			return SCARA_STATUS_ERROR_OVER_WORKSPACE;
		}
	} else if (TRAJECTORY_J1 == target) {
		if ( (LIM_MIN_J1 <= value) && ( value <= LIM_MAX_J1)) {
			return SCARA_STATUS_OK;
		} else {
			return SCARA_STATUS_ERROR_OVER_WORKSPACE;
		}
	} else if (TRAJECTORY_J2 == target) {
		if ( (LIM_MIN_J2 <= value) && ( value <= LIM_MAX_J2)) {
			return SCARA_STATUS_OK;
		} else {
			return SCARA_STATUS_ERROR_OVER_WORKSPACE;
		}
	} else if (TRAJECTORY_J3 == target) {
		if ( (LIM_MIN_J3 <= value) && ( value <= LIM_MAX_J3)) {
			return SCARA_STATUS_OK;
		} else {
			return SCARA_STATUS_ERROR_OVER_WORKSPACE;
		}
	} else {
		return SCARA_STATUS_ERROR;
	}
}

void				scaraSetScanFlag(void) {
	mySCARA.isScanLitmit = TRUE;
}


void				scaraSetOutput		(int8_t level) {
	mySCARA.outputSet	 = level;
}


void				scaraSetDutyState(SCARA_DutyStateTypeDef state) {
	mySCARA.duty_State = state;
}

void				scaraGetPosition	(SCARA_PositionTypeDef *pos) {
	memcpy(pos, &positionCurrent, sizeof(SCARA_PositionTypeDef));
}

void				scaraSetMode(SCARA_ModeTypeDef mode) {
	mySCARA.mode = mode;
}

SCARA_ModeTypeDef	scaraGetMode(void) {
	return mySCARA.mode;
}

SCARA_DutyStateTypeDef	scaraGetDutyState(void) {
	return mySCARA.duty_State;
}

uint8_t					scaraIsScanLimit(void) {
	return mySCARA.isScanLitmit;
}

uint8_t					scaraIsFinish		(float run_time) {
	if(myDUTY.time_total < (run_time + 0.01)) {
		return TRUE;
	} else {
		return FALSE;
	}
}

int32_t					scaraPosition2String(char * result, SCARA_PositionTypeDef position) {
	uint8_t theta1[12];
	uint8_t theta2[12];
	uint8_t d3[12];
	uint8_t theta4[12];
	uint8_t x[12];
	uint8_t y[12];
	uint8_t z[12];
	int32_t lenght;

	float2string(theta1, position.Theta1, 6);
	float2string(theta2, position.Theta2, 6);
	float2string(d3, position.D3, 6);
	float2string(theta4, position.Theta4, 6);
	float2string(x, position.x, 6);
	float2string(y, position.y, 6);
	float2string(z, position.z, 6);

	lenght = snprintf( (char *)result, 85,
						"%s %s %s %s %s %s %s",
						theta1,
						theta2,
						d3,
						theta4,
						x,
						y,
						z);
	return lenght;
}