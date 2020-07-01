/*
 * robot_scara.h
 *
 *  Created on: Mar 10, 2020
 *      Author: Dang Nam
 */

#ifndef INC_ROBOT_SCARA_H_
#define INC_ROBOT_SCARA_H_

#include "common_def.h"
#include "system_params.h"


/* FOR ROBOT */
typedef enum
{
	  SCARA_STATUS_OK					= 0x00U,
	  SCARA_STATUS_ERROR				= 0x01U,
	  SCARA_STATUS_ERROR_SPACE			= 0x02U,
	  SCARA_STATUS_ERROR_TASK			= 0x03U,
	  SCARA_STATUS_ERROR_JOINT			= 0x04U,
	  SCARA_STATUS_ERROR_TRAJECTORY		= 0x05U,
	  SCARA_STATUS_ERROR_PARA			= 0x06U,
	  SCARA_STATUS_ERROR_OVER_WORKSPACE = 0x07U,
	  SCARA_STATUS_ERROR_MODE_INIT 		= 0x08U,
	  SCARA_STATUS_ERROR_OVER_VELOC		= 0x09U,
	  SCARA_STATUS_ERROR_OVER_ACCEL		= 0x0AU,
	  SCARA_STATUS_ERROR_JOINT_NUM		= 0x0BU,
	  SCARA_STATUS_ERROR_COORDINATE		= 0x0CU,
	  NUM_OF_STATUS						= 0x0DU
}SCARA_StatusTypeDef;

typedef enum
{
	  SCARA_MODE_STOP					= 0x00U,  /*!< SCARA scan limit switch to determine absolute position*/
	  SCARA_MODE_SCAN					= 0x01U,  /*!< SCARA work  */
	  SCARA_MODE_DUTY					= 0x02U   /*!< SCARA stop immediate  */
}SCARA_ModeTypeDef;

typedef enum
{
	  SCARA_DUTY_STATE_READY			= 0x00U,  /*!< There is not duty yet */
	  SCARA_DUTY_STATE_INIT				= 0x01U,  /*!< Init duty   */
	  SCARA_DUTY_STATE_FLOW				= 0x02U,  /*!< FLow in duty */
	  SCARA_DUTY_STATE_FINISH			= 0x03U  /*!< Fishish duty */
}SCARA_DutyStateTypeDef;


typedef enum
{
	  SCARA_SCAN_STATE_INIT				= 0x00U,
	  SCARA_SCAN_STATE_HARD				= 0x01U,
	  SCARA_SCAN_STATE_SOFT				= 0x02U,
	  SCARA_SCAN_STATE_FINISH			= 0x03U
}SCARA_ScanStateTypeDef;

/* FOR DUTY */
typedef enum
{
	  DUTY_COORDINATES_ABS				= 0x00U,  /*!< Absolute position */
	  DUTY_COORDINATES_REL				= 0x01U  /*!< Relative position*/
}CoordinatesTypeDef;

typedef enum
{
	  DUTY_MODE_INIT_QVA				= 0x00U,  /*!< Consume A max, determine T max */
	  DUTY_MODE_INIT_QVT				= 0x01U  /*!< Consume T max, determine A max   */
}ModeInitTypeDef;

typedef enum
{
	  DUTY_SPACE_TASK					= 0x00U,  /*!< Task space */
	  DUTY_SPACE_JOINT					= 0x01U  /*!< Joint space  */
}SpaceTypeDef;

typedef enum
{
	  DUTY_PATH_LINE					= 0x00U,  /*!< Path planning straight line in task space */
	  DUTY_PATH_CIRCLE					= 0x01U  /*!< Path planning circle in task space */
}PathTypeDef;

typedef enum
{
	  DUTY_JOINT_SINGLE					= 0x00U,  /*!< Robot rotate a single joint */
	  DUTY_JOINT_4DOF					= 0x01U  /*!< Robot  */
}JointTypeDef;

typedef enum
{
	  DUTY_TRAJECTORY_LSPB				= 0x00U,  /*!< Trajectory planning LSBP */
	  DUTY_TRAJECTORY_SCURVE			= 0x01U  /*!< Trajectory planning S-curve */
}TrajectoryTypeDef;

typedef enum
{
	  TRAJECTORY_J0				= 0x00U,  /*!< Trajectory planning for rad */
	  TRAJECTORY_J1				= 0x01U,  /*!< Trajectory planning for rad */
	  TRAJECTORY_J2				= 0x02U,  /*!< Trajectory planning for rad */
	  TRAJECTORY_J3				= 0x03U,  /*!< Trajectory planning for rad */
	  TRAJECTORY_3D				= 0x04U,  /*!< Trajectory planning for mm  */
	  TRAJECTORY_ROLL			= 0x05U,  /*!< Trajectory planning for mm  */
}Trajectory_TargetTypeDef;


/*** Instance Form ***/
typedef struct
{
	double						q;
	double						v;
	double						a;
	double						t;
	double						total_time;
	double 						x;
	double 						y;
	double 						z;
	double						roll;
	double 						Theta1;//Rotate
	double 						Theta2;//Rotate
	double 						D3;//Transpose
	double 						Theta4;//Rotate
}SCARA_PositionTypeDef;

typedef struct
{
	int8_t			 dir;
	double			 s0;
	double 			 s1;
	double			 v0;
	double			 v1;
	double 			 v_design;
	double 			 a_design;
	double			 v_lim;
	double			 Ta;
	double			 Td;
	double			 Tf;
	uint32_t		 num_of_sampling;
	double			 total_s;
	double			 a_current;
	double			 v_current;
	double			 s_current;

}Trajectory_LSPB_TypeDef;


typedef struct
{
	int8_t			 dir;
	double			 s0;
	double 			 s1;
	double			 v0;
	double			 v1;
	double 			 v_design;
	double 			 a_design;
	double			 v_lim;
	uint8_t			 num_of_phase;
	double			 j_max;
	double			 Tm;
	double			 Tc;
	double			 Tf;
	uint32_t		 num_of_sampling;
	double			 total_s;
	double			 v_1;
	double			 s_1;
	double			 v_2;
	double			 s_2;
	double			 v_3;
	double			 s_3;
	double			 v_4;
	double			 s_4;
	double			 a_current;
	double			 v_current;
	double			 s_current;
}Trajectory_Scurve_TypeDef;


typedef struct
{
	double			 x0;
	double			 x1;
	double			 y0;
	double			 y1;
	double			 z0;
	double			 z1;
	double			 denta_x;
	double			 denta_y;
	double			 denta_z;
	double			 total_s;
	double			 delta_s;
	double			 x_current;
	double			 y_current;
	double			 z_current;
}Path_Line_TypeDef;


typedef struct
{
	int8_t			 dir;
	double			 radius;
	double			 angle_start;
	double			 angle_stop;
	double			 x0;
	double			 x1;
	double			 y0;
	double			 y1;
	double			 xi;
	double			 yi;
	double			 z0;
	double			 z1;
	double			 zi;
	double			 total_s;
	double			 total_angle;
	double			 x_current;
	double			 y_current;
	double			 z_current;
}Path_Circle_TypeDef;


typedef struct
{
	PathTypeDef 			path_type;
	Path_Line_TypeDef 		line;
	Path_Circle_TypeDef 	circle;
}Path_TypeDef;


typedef struct
{
	TrajectoryTypeDef 					trajectory_type;
	Trajectory_LSPB_TypeDef 			lspb;
	Trajectory_Scurve_TypeDef			scurve;
}Trajectory_TypeDef;


typedef struct
{
	Path_TypeDef	 		path;
	Trajectory_TypeDef 		trajectory_3d;
	Trajectory_TypeDef 		trajectory_roll;
	double					roll_start;
}DUTY_Task_TypeDef;


typedef struct
{
	Trajectory_TypeDef		trajectory[4];
	double					theta1_start;
	double					theta2_start;
	double					d3_start;
	double					theta4_start;
}DUTY_Joint_TypeDef;


typedef struct
{
	SpaceTypeDef		space_type;
	DUTY_Task_TypeDef	task;
	DUTY_Joint_TypeDef	joint;
	double				time_total;
}DUTY_TypeDef;


typedef struct
{
	SCARA_ModeTypeDef			robot_mode;
	int32_t 					id_command;
	CoordinatesTypeDef			coordinate_type;
	SpaceTypeDef				space_type;
	PathTypeDef 				path_type;
	JointTypeDef				joint_type;
	TrajectoryTypeDef			trajec_type;
	ModeInitTypeDef				modeInit_type;
	int32_t						sub_para_int;
	double						sub_para_double;
	double						time_total;
	SCARA_PositionTypeDef		target_point;
	SCARA_PositionTypeDef		sub_point;
	double						v_factor;
	double						a_factor;
}DUTY_Command_TypeDef;


typedef struct
{
	SCARA_ModeTypeDef 			mode;
	SCARA_DutyStateTypeDef 		duty_State;
	uint8_t						isScanLitmit;
	uint8_t						outputSet;
}SCARA_TypeDef;


/* Function prototype */
void						scaraStartup		(void);
SCARA_StatusTypeDef			scaraInitDuty		(DUTY_Command_TypeDef command);

SCARA_StatusTypeDef			scaraInitLine		(Path_Line_TypeDef *line,
												SCARA_PositionTypeDef start,
												SCARA_PositionTypeDef end);

SCARA_StatusTypeDef			scaraInitCircle		(Path_Circle_TypeDef *circle,
												SCARA_PositionTypeDef start,
												SCARA_PositionTypeDef end,
												SCARA_PositionTypeDef center,
												int32_t dir);

SCARA_StatusTypeDef			scaraInitLSPB		(Trajectory_LSPB_TypeDef *lspb,
												Trajectory_TargetTypeDef target,
												double total_s,
												ModeInitTypeDef modeinits,
												double v_factor,
												double a_factor);

SCARA_StatusTypeDef			scaraInitScurve		(Trajectory_Scurve_TypeDef *scurve,
												Trajectory_TargetTypeDef target,
												double total_s,
												ModeInitTypeDef modeinit,
												double v_factor,
												double a_factor);

SCARA_StatusTypeDef			scaraFlowDuty		(double time,
												SCARA_PositionTypeDef *pos_Next ,
												SCARA_PositionTypeDef pos_Current);

SCARA_StatusTypeDef			scaraFlowLine		(Path_Line_TypeDef *line, double s);
SCARA_StatusTypeDef			scaraFlowCircle		(Path_Circle_TypeDef *circle, double s);
SCARA_StatusTypeDef			scaraFlowLSPB		(Trajectory_LSPB_TypeDef *lspb, double time);
SCARA_StatusTypeDef			scaraFLowScurve		(Trajectory_Scurve_TypeDef *scurve, double time);


SCARA_StatusTypeDef			scaraCheckWorkSpace4(double theta1, double theta2, double d3, double theta4);
SCARA_StatusTypeDef			scaraCheckWorkSpace1(Trajectory_TargetTypeDef target, double value);
SCARA_StatusTypeDef			scaraTestDuty(void);

void						scaraSetScanFlag	(void);
void						scaraSetOutput		(uint8_t level);
void						scaraSetDutyState	(SCARA_DutyStateTypeDef state);
void						scaraSetMode		(SCARA_ModeTypeDef mode);

void						scaraGetPosition	(SCARA_PositionTypeDef *pos);
SCARA_ModeTypeDef			scaraGetMode		(void);
SCARA_DutyStateTypeDef		scaraGetDutyState	(void);
uint8_t						scaraIsScanLimit	(void);
uint8_t						scaraIsFinish		(double run_time);
int32_t						scaraPosition2String(char *result, SCARA_PositionTypeDef position);

#endif /* INC_ROBOT_SCARA_H_ */
