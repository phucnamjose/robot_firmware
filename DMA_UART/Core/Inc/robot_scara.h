/*
 * robot_scara.h
 *
 *  Created on: Mar 10, 2020
 *      Author: Dang Nam
 */

#ifndef INC_ROBOT_SCARA_H_
#define INC_ROBOT_SCARA_H_

#include "common_def.h"

/* Limit Joint*/
#define LIM_MIN_J0		(-PI/2)
#define LIM_MAX_J0		(PI/2)
#define LIM_MIN_J1		(-3*PI/4)
#define LIM_MAX_J1		(3*PI/4)
#define LIM_MIN_J2		(0)
#define LIM_MAX_J2		(100)
#define LIM_MIN_J3		(-PI)
#define LIM_MAX_J3		(PI)

/* Maximum Velocity System*/
#define V_DESIGN_3D		(20.0f)
#define V_DESIGN_ROLL	(PI/10)
#define V_DESIGN_J0		(PI/10)
#define V_DESIGN_J1		(PI/10)
#define V_DESIGN_J2		(20.0f)
#define V_DESIGN_J3		(PI/10)

/* Maximum Accelerate System*/
#define A_DESIGN_3D		(5.0f)
#define A_DESIGN_ROLL	(PI/100)
#define A_DESIGN_J0		(PI/100)
#define A_DESIGN_J1		(PI/100)
#define A_DESIGN_J2		(5.0f)
#define A_DESIGN_J3		(PI/100)

/* Sampling Period*/
#define T_SAMPLING		(0.01f)


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
	float						q;
	float						v;
	float						a;
	float						t;
	float 						x;
	float 						y;
	float 						z;
	float						roll;
	float 						Theta1;//Rotate
	float 						Theta2;//Rotate
	float 						D3;//Transpose
	float 						Theta4;//Rotate
}SCARA_PositionTypeDef;

typedef struct
{
	int8_t			 dir;
	float			 s0;
	float 			 s1;
	float			 v0;
	float			 v1;
	float 			 v_design;
	float 			 a_design;
	float			 v_lim;
	float			 Ta;
	float			 Td;
	float			 Tf;
	uint32_t		 num_of_sampling;
	float			 total_s;
	float			 a_current;
	float			 v_current;
	float			 s_current;

}Trajectory_LSPB_TypeDef;


typedef struct
{
	int8_t			 dir;
	float			 s0;
	float 			 s1;
	float			 v0;
	float			 v1;
	float 			 v_design;
	float 			 a_design;
	float			 v_lim;
	uint8_t			 num_of_phase;
	float			 j_max;
	float			 Tm;
	float			 Tc;
	float			 Tf;
	uint32_t		 num_of_sampling;
	float			 total_s;
	float			 v_1;
	float			 s_1;
	float			 v_2;
	float			 s_2;
	float			 v_3;
	float			 s_3;
	float			 v_4;
	float			 s_4;
	float			 a_current;
	float			 v_current;
	float			 s_current;
}Trajectory_Scurve_TypeDef;


typedef struct
{
	float			 x0;
	float			 x1;
	float			 y0;
	float			 y1;
	float			 z0;
	float			 z1;
	float			 denta_x;
	float			 denta_y;
	float			 denta_z;
	float			 total_s;
	float			 delta_s;
	float			 x_current;
	float			 y_current;
	float			 z_current;
}Path_Line_TypeDef;


typedef struct
{
	int8_t			 dir;
	float			 radius;
	float			 angle_start;
	float			 angle_stop;
	float			 x0;
	float			 x1;
	float			 y0;
	float			 y1;
	float			 xi;
	float			 yi;
	float			 z0;
	float			 z1;
	float			 zi;
	float			 total_s;
	float			 total_angle;
	float			 x_current;
	float			 y_current;
	float			 z_current;
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
	float					roll_start;
}DUTY_Task_TypeDef;


typedef struct
{
	Trajectory_TypeDef		trajectory[4];
	float					theta1_start;
	float					theta2_start;
	float					d3_start;
	float					theta4_start;
}DUTY_Joint_TypeDef;


typedef struct
{
	SpaceTypeDef		space_type;
	DUTY_Task_TypeDef	task;
	DUTY_Joint_TypeDef	joint;
	float				time_total;
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
	float						sub_para_float;
	float						time_total;
	SCARA_PositionTypeDef		target_point;
	SCARA_PositionTypeDef		sub_point;
	float						v_factor;
	float						a_factor;
}DUTY_Command_TypeDef;


typedef struct
{
	SCARA_ModeTypeDef 			mode;
	SCARA_DutyStateTypeDef 		duty_State;
	uint8_t						isScanLitmit;
	uint8_t						outputSet;
}SCARA_TypeDef;


/* Function prototype */
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
												float total_s,
												ModeInitTypeDef modeinits);

SCARA_StatusTypeDef			scaraInitScurve		(Trajectory_Scurve_TypeDef *scurve,
												Trajectory_TargetTypeDef target,
												float total_s,
												ModeInitTypeDef modeinit);

SCARA_StatusTypeDef			scaraFlowDuty		(float time);
SCARA_StatusTypeDef			scaraFlowLine		(Path_Line_TypeDef *line, float s);
SCARA_StatusTypeDef			scaraFlowCircle		(Path_Circle_TypeDef *circle, float s);
SCARA_StatusTypeDef			scaraFlowLSPB		(Trajectory_LSPB_TypeDef *lspb, float time);
SCARA_StatusTypeDef			scaraFLowScurve		(Trajectory_Scurve_TypeDef *scurve, float time);


SCARA_StatusTypeDef			scaraCheckWorkSpace4(float theta1, float theta2, float d3, float theta4);
SCARA_StatusTypeDef			scaraCheckWorkSpace1(Trajectory_TargetTypeDef target, float value);

void						scaraSetScanFlag	(void);
void						scaraSetOutput		(int8_t level);
void						scaraSetDutyState	(SCARA_DutyStateTypeDef state);
void						scaraSetMode		(SCARA_ModeTypeDef mode);

void						scaraGetPosition	(SCARA_PositionTypeDef *pos);
SCARA_ModeTypeDef			scaraGetMode		(void);
SCARA_DutyStateTypeDef		scaraGetDutyState	(void);
uint8_t						scaraIsScanLimit	(void);
uint8_t						scaraIsFinish		(float run_time);
int32_t						scaraPosition2String(char * result, SCARA_PositionTypeDef position);

#endif /* INC_ROBOT_SCARA_H_ */
