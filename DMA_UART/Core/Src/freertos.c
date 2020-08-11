/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "tim.h"
#include <stdio.h>
#include <string.h>
#include <malloc.h>
#include "dma.h"
#include "usart.h"
#include "common_def.h"
#include "ringbuffer.h"
#include "robot_scara.h"
#include "robot_lowlayer.h"
#include "command_respond.h"
#include "kinematic.h"
#include "communicate_payload.h"
#include "usbd_cdc_if.h"
#include "gpio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

extern RINGBUFFER_TypeDef 			usb_rx_ringbuff;
extern RINGBUFFER_TypeDef 			cmd_tx_ringbuff;
extern const char 					*DETAIL_STATUS[NUM_OF_STATUS];
extern SCARA_PositionTypeDef		positionPrevios;
extern SCARA_PositionTypeDef		positionCurrent;
extern SCARA_PositionTypeDef		positionNext;
extern SCARA_PositionTypeDef		positionTrue;

extern TIM_HandleTypeDef htim7;
osMailQId commandMailHandle;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId USB_RX_Check_Handle;
osMutexId usbTxMutexHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void Start_USB_RX_Task(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* Create the mutex(es) */
  /* definition and creation of usbTxMutex */
  osMutexDef(usbTxMutex);
  usbTxMutexHandle = osMutexCreate(osMutex(usbTxMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	  /* Create the queue(s) */
	  /* definition and creation of commandMail */
	  osMailQDef(commandMail, 1, DUTY_Command_TypeDef);
	  commandMailHandle = osMailCreate(osMailQ(commandMail), NULL);

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityHigh, 0, 2048);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of USB_RX_Check_ */
  osThreadDef(USB_RX_Check_, Start_USB_RX_Task, osPriorityNormal, 0, 1024);
  USB_RX_Check_Handle = osThreadCreate(osThread(USB_RX_Check_), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  HAL_GPIO_WritePin(USB_SIGN_GPIO_Port, USB_SIGN_Pin, GPIO_PIN_SET); // Pull-up Resistor

  osEvent 				ret_mail;
  DUTY_Command_TypeDef 	duty_cmd;
  DUTY_Command_TypeDef 	*dataMail;
  uint8_t 				isNewDuty = FALSE;


  // Report buffer;
  uint8_t				respond[40];
  int32_t				respond_lenght;
  uint8_t				position[135];
  uint8_t				infor[145];
  int32_t				infor_lenght;
  uint8_t				task_usb[150];
  int32_t				task_usb_lenght;

  uint8_t				respond_packed[50];
  int32_t				respond_packed_lenght;
  uint8_t				infor_packed[150];
  int32_t				infor_packed_lenght;
  uint8_t				usb_buff[350];
  int32_t				usb_lenght;
  // Debug variables
  //uint32_t total_pulse = 0;

  // Robot variables
  SCARA_MethodTypeDef		current_method;
  SCARA_ModeTypeDef			current_mode;
  SCARA_DutyStateTypeDef 	current_duty_state;
  SCARA_ScanStateTypeDef	current_scan_state;
  SCARA_KeyStateTypeDef		current_key_state;
  SCARA_KeyTypeDef			current_key;
  int32_t					current_key_speed;
  double						run_time;

  LOG_REPORT("free_rtos.c: PROGRAM START...", __LINE__);

  // Init value
  current_method = scaraGetMethod();
  current_mode	 = scaraGetMode();
  current_duty_state	 = scaraGetDutyState();

  // Start up robot
  scaraStartup();
  osDelay(10);

  positionNext.Theta1 = -PI/3;
  positionNext.Theta2 = PI/3;
  positionNext.D3 = 10;
  positionNext.Theta4 = 0;
  positionNext.t = 0;
  kinematicForward(&positionNext);
  /* Infinite loop */
//Start Timer 7
	  HAL_TIM_Base_Start_IT(&htim7);
  for(;;)
  {
	  /*---------Wait for Timer Trigger-----------*/
	  osSignalWait(0x01, osWaitForever); // Very Important
	  /* 1--- Reset Value ---*/
	  respond_lenght		= 0;
	  respond_packed_lenght = 0;
	  infor_lenght			= 0;
	  infor_packed_lenght	= 0;
	  task_usb_lenght		= 0;
	  usb_lenght			= 0;
	  // Update new position
	  memcpy(&positionPrevios, &positionCurrent, sizeof(SCARA_PositionTypeDef));
	  memcpy(&positionCurrent, &positionNext, sizeof(SCARA_PositionTypeDef));
#ifndef SIMULATION
	  if(scaraIsScanLimit()) {
		  lowlayer_readTruePosition(&positionTrue);
		  kinematicForward(&positionTrue);
		  positionTrue.t = positionCurrent.t;
		  positionTrue.total_time = positionCurrent.total_time;
		  positionTrue.q = positionCurrent.q;
	  }
#endif
	  /* 2--- Check New Duty Phase ---*/
	  // Check mail
	  ret_mail = osMailGet(commandMailHandle, 0);
	  if ( ret_mail.status == osEventMail) {
		   dataMail = ret_mail.value.p;
		   memcpy( &duty_cmd, dataMail, sizeof(DUTY_Command_TypeDef));
		   isNewDuty = TRUE;
		   osMailFree(commandMailHandle, dataMail);/* free memory allocated for mail */
	  }
	  if(isNewDuty) {
		  memset(respond, 0, 40);
		  // Check change method
		  if (duty_cmd.change_method == TRUE) {
			  if (SCARA_METHOD_MANUAL == duty_cmd.robot_method) {
				  // Need add check condition idle in each method
				  current_method = SCARA_METHOD_MANUAL;
				  respond_lenght = commandRespond(RPD_OK,
												duty_cmd.id_command,
												"Changed MANUAL Method",
												(char *)respond);
			  } else if (SCARA_METHOD_SEMI_AUTO == duty_cmd.robot_method) {
				  current_method = SCARA_METHOD_SEMI_AUTO;
				  respond_lenght = commandRespond(RPD_OK,
												duty_cmd.id_command,
												"Changed SEMI AUTO Method",
												(char *)respond);
			  } else if (SCARA_METHOD_AUTO == duty_cmd.robot_method) {
				  current_method = SCARA_METHOD_AUTO;
				  respond_lenght = commandRespond(RPD_OK,
												duty_cmd.id_command,
												"Changed AUTO Method",
												(char *)respond);
			  }
		  } else {
			  	  // Check current method & cmd method
			  	  if (current_method == duty_cmd.robot_method) {
				  	  switch( duty_cmd.robot_method) {
				  	  case SCARA_METHOD_MANUAL:
				  	  {
				  		  if (current_key_state == SCARA_KEY_STATE_READY) {
				  			  current_key = duty_cmd.keyboard;
				  			  current_key_state = SCARA_KEY_STATE_INIT;// Init new path
				  			  current_key_speed = duty_cmd.key_speed;
				  		  } else if (current_key == duty_cmd.keyboard
				  				  	  && current_key_state == SCARA_KEY_STATE_FLOW
									  && current_key_speed == duty_cmd.key_speed) {
				  			  current_key_state = SCARA_KEY_STATE_INIT;// Continue old path
				  		  }
				  	  }
				  	  break;
				  	  case SCARA_METHOD_SEMI_AUTO:
				  	  {
						  switch( duty_cmd.robot_mode) {
						  case SCARA_MODE_STOP:
							  {
								  current_mode	 = SCARA_MODE_STOP;
								  respond_lenght = commandRespond(RPD_OK,
																  duty_cmd.id_command,
																  "Stop Now",
																  (char *)respond);
								  LOG_REPORT("ROBOT STOP !!!", __LINE__);
							  }
							  break;

						  case SCARA_MODE_SCAN:
							  {
								  if (SCARA_MODE_DUTY == current_mode
									  && SCARA_DUTY_STATE_READY == current_duty_state) {
									  current_mode = SCARA_MODE_SCAN;
									  current_scan_state = SCARA_SCAN_STATE_INIT;
									  respond_lenght = commandRespond(RPD_OK,
																	  duty_cmd.id_command,
																	  "Start Scan",
																	  (char *)respond);
								  } else {
									  respond_lenght = commandRespond(RPD_ERROR,
																	  duty_cmd.id_command,
																	  "Busy",
																	  (char *)respond);
									  LOG_REPORT("SCAN FAIL: BUSY", __LINE__);
								  }
							  }
							  break;

						  case SCARA_MODE_DUTY:
							  {
								  if (SCARA_MODE_DUTY == current_mode && SCARA_DUTY_STATE_READY == current_duty_state) {
									  if (scaraIsScanLimit()) {
										  current_mode	 = SCARA_MODE_DUTY;
										  current_duty_state	 = SCARA_DUTY_STATE_INIT;
									  } else {
										  respond_lenght = commandRespond(RPD_ERROR,
																		  duty_cmd.id_command,
																		  "Has Not Scan Yet.",
																		  (char *)respond);
										  LOG_REPORT("MOVE FAIL:NOT SCAN", __LINE__);
									  }
								  } else {
									  respond_lenght	= commandRespond(RPD_ERROR,
																		  duty_cmd.id_command,
																		  "Busy.",
																		  (char *)respond);
									  LOG_REPORT("MOVE FAIL:BUSY", __LINE__);
								  }
							  }
							  break;
						  default:
							  {
								  LOG_REPORT("CMD Error Mode !!!", __LINE__);
							  }
						  }
					  }
					  break;
				  	  case SCARA_METHOD_AUTO:
				  	  {

				  	  }
				  	  break;
				  	  default:
				  	  {
				  		  LOG_REPORT("CMD Error Method !!!", __LINE__);
				  	  }
				  	  }
			  	  } else {
					  respond_lenght = commandRespond(RPD_ERROR,
													duty_cmd.id_command,
													"METHOD isn't correct",
													(char *)respond);
			  	  }

			  }
		  isNewDuty = FALSE;
	  }

	  /* 3--- Execute Phase ---*/
	  switch(current_method) {
	  case SCARA_METHOD_MANUAL:
	  {
		  switch( current_key_state) {
		  case SCARA_KEY_STATE_READY:
			  /* Wait for application keyboard , do nothing*/
		  break;
		  case SCARA_KEY_STATE_INIT:
		  {
			  if (scaraKeyInit(current_key, current_key_speed, &run_time) == SCARA_STATUS_OK) {
				  current_key_state = SCARA_KEY_STATE_FLOW;
#ifdef SIMULATION
				  scaraPosition2String((char *)position, positionCurrent);
#else
				  scaraPosition2String((char *)position, positionTrue);
#endif
				  infor_lenght 		= commandRespond(RPD_START,
													  0,
													  (char *)position,
													  (char *)infor);
			  } else {
				  current_key_state = SCARA_KEY_STATE_READY;
			  }
		  }
		  break;
		  case SCARA_KEY_STATE_FLOW:
		  {
			  SCARA_StatusTypeDef status;
			  // Increase run time
			  run_time += T_SAMPLING;
			  // Check Time Out
			  if (scaraIsFinish(run_time)) {
				  current_key_state = SCARA_KEY_STATE_FINISH;// Key Done
			  } else {
				  status = scaraKeyFlow(run_time, &positionNext, positionCurrent);
				  if ( SCARA_STATUS_OK == status) {
					  lowlayer_computeAndWritePulse(positionCurrent, positionNext);
					  // Running Inform
#ifdef SIMULATION
					  scaraPosition2String((char *)position, positionCurrent);
#else
					  scaraPosition2String((char *)position, positionTrue);
#endif
					  infor_lenght = commandRespond(RPD_RUNNING,
													0,
													(char *)position,
													(char *)infor);
				  } else {
					  current_key_state = SCARA_KEY_STATE_FINISH;
					  // Critical
					  // If a error appear while Flowing, This is very important
					  infor_lenght = commandRespond(RPD_STOP,
													0,
													(char *)DETAIL_STATUS[status],
													(char *)infor);
					  LOG_REPORT("STOP KEY", __LINE__);
				  }
			  }
		  }
		  break;
		  case SCARA_KEY_STATE_FINISH:
		  {
			  current_key_state = SCARA_KEY_STATE_READY;
			  positionNext.t = 0;
			  positionNext.total_time = 0;
			  positionNext.q = 0;
		  }
		  break;
		  }
	  }
	  break;

	  case SCARA_METHOD_SEMI_AUTO:
	  {
		  switch( current_mode) {
		  case SCARA_MODE_STOP:
			  {
				  current_mode 	= SCARA_MODE_DUTY;
				  current_duty_state = SCARA_DUTY_STATE_READY;
			  }
			  break;

		  case SCARA_MODE_SCAN:
			  {
				  switch (current_scan_state) {
				  case SCARA_SCAN_STATE_INIT:
					  {
						  lowlayer_scanReset();
						  current_scan_state = SCARA_SCAN_STATE_HARD;
					  }
					  break;
				  case SCARA_SCAN_STATE_HARD:
					  {
						  if(lowlayer_scanFlow()) {
							  current_scan_state = SCARA_SCAN_STATE_SOFT;
						  }
					  }
					  break;
				  case SCARA_SCAN_STATE_SOFT:
					  {
						  if(lowlayer_goToSoftLimit(&positionNext)) {
							  current_scan_state = SCARA_SCAN_STATE_FINISH;
							}
					  }
					  break;
				  case SCARA_SCAN_STATE_FINISH:
					  {
						  lowlayer_readSetPosition(&positionNext);
						  current_mode 	= SCARA_MODE_DUTY;
						  current_duty_state = SCARA_DUTY_STATE_READY;
						  kinematicForward(&positionNext);
						  scaraSetScanFlag();
						  //Done Inform
						  scaraPosition2String((char *)position, positionNext);
						  infor_lenght 		= commandRespond(RPD_DONE,
															 0,
															(char *)position,
															(char *)infor);
					  }
					  break;
				  default:
					  {
						  LOG_REPORT("ERROR STATE !!!", __LINE__);
						  while(1);
					  }
				  }

			  }
			  break;

		  case SCARA_MODE_DUTY:
			  {
				  switch (current_duty_state) {
				  case SCARA_DUTY_STATE_READY:
					  {
						  // Do nothing();
						  __NOP();
					  }
				  break;

				  case SCARA_DUTY_STATE_INIT:
					  {
						  SCARA_StatusTypeDef status1, status2;
						  status1 = scaraInitDuty(duty_cmd);
						  if ( SCARA_STATUS_OK == status1) {
							  status2 = scaraTestDuty();
							  if (SCARA_STATUS_OK == status2) {
							  current_duty_state		= SCARA_DUTY_STATE_FLOW;
							  run_time			= 0;
							  // Respond
							  respond_lenght 	= commandRespond(RPD_OK,
																  duty_cmd.id_command,
																  (char *)DETAIL_STATUS[status1],
																  (char *)respond);
#ifdef SIMULATION
							  scaraPosition2String((char *)position, positionCurrent);
#else
							  scaraPosition2String((char *)position, positionTrue);
#endif
							  // Start Inform
							  infor_lenght 		= commandRespond(RPD_START,
																  0,
																  (char *)position,
																  (char *)infor);
							  } else {
								  current_duty_state 	= SCARA_DUTY_STATE_READY;
								  respond_lenght	= commandRespond(RPD_ERROR,
																	  duty_cmd.id_command,
																	  (char *)DETAIL_STATUS[status2],
																	  (char *)respond);
								  LOG_REPORT("TEST FAIL", __LINE__);
							  }
						  } else {
							  current_duty_state 	= SCARA_DUTY_STATE_READY;
							  respond_lenght	= commandRespond(RPD_ERROR,
																  duty_cmd.id_command,
																  (char *)DETAIL_STATUS[status1],
																  (char *)respond);
							  LOG_REPORT("INIT FAIL", __LINE__);
						  }
					  }
				  break;

				  case SCARA_DUTY_STATE_FLOW:
					  {
						  SCARA_StatusTypeDef status;
						  // Increase run time
						  run_time += T_SAMPLING;
						  // Check Time Out
						  if (scaraIsFinish(run_time)) {
							  current_duty_state = SCARA_DUTY_STATE_FINISH;// Work Done
						  } else {
							  status = scaraFlowDuty(run_time , &positionNext, positionCurrent);
							  if ( SCARA_STATUS_OK == status) {
								  lowlayer_computeAndWritePulse(positionCurrent, positionNext);
								  // Running Inform
#ifdef SIMULATION
								  scaraPosition2String((char *)position, positionCurrent);
#else
								  scaraPosition2String((char *)position, positionTrue);
#endif
								  infor_lenght = commandRespond(RPD_RUNNING,
																0,
																(char *)position,
																(char *)infor);
							  } else {
								  current_duty_state = SCARA_DUTY_STATE_FINISH;
								  // Critical
								  // If a error appear while Flowing, This is very important
								  infor_lenght = commandRespond(RPD_STOP,
																0,
																(char *)DETAIL_STATUS[status],
																(char *)infor);
								  LOG_REPORT("STOP DUTY", __LINE__);
							  }
						  }
					  }
				  break;

				  case SCARA_DUTY_STATE_FINISH:
					  {
						  current_duty_state = SCARA_DUTY_STATE_READY;
						  positionNext.t = 0;
						  positionNext.total_time = 0;
						  positionNext.q = 0;
						  // Done Inform
#ifdef SIMULATION
						  scaraPosition2String((char *)position, positionCurrent);
#else
						  scaraPosition2String((char *)position, positionTrue);
#endif
						  infor_lenght 		= commandRespond(RPD_DONE,
															 0,
															 (char *)position,
															 (char *)infor);
					  }
				  break;

				  default:
					  {
						  LOG_REPORT("ERROR STATE !!!", __LINE__);
						  while(1);
					  }
				  }

			  }
			  break;

		  default:
			  {
				  LOG_REPORT("ERROR MODE !!!", __LINE__);
				  while(1);
			  }
		  }
	  }
	  break;

	  case SCARA_METHOD_AUTO:
	  {

	  }

	  break;
	  default:
	  {

	  }
	  }

	  /* 4--- Send to PC Phase ---*/
	  // Check buffer from USB task
	  osMutexWait(usbTxMutexHandle, osWaitForever);
	  task_usb_lenght = ringBuff_PopArray(&cmd_tx_ringbuff, task_usb, RINGBUFFER_SIZE);
	  osMutexRelease(usbTxMutexHandle);
	  // Intergrate to 1 buffer
	  if (respond_lenght > 0) {
		  respond_packed_lenght = packPayload(respond, respond_packed, respond_lenght);
		  memcpy(usb_buff, respond_packed, respond_packed_lenght);
	  }
	  if (task_usb_lenght > 0) {
		  memcpy(usb_buff + respond_packed_lenght, task_usb, task_usb_lenght);
	  }
	  if (infor_lenght > 0) {
		  infor_packed_lenght 	= packPayload(infor, infor_packed, infor_lenght);
		  memcpy(usb_buff + respond_packed_lenght + task_usb_lenght, infor_packed, infor_packed_lenght);
	  }
	  usb_lenght = respond_packed_lenght + task_usb_lenght + infor_packed_lenght;
	  // Send through USB
	  if (usb_lenght > 0) {
		  CDC_Transmit_FS(usb_buff, (uint16_t)usb_lenght);
	  }

	  /* 5--- Update ---*/
	  scaraSetMethod(current_method);
	  scaraSetMode(current_mode);
	  scaraSetDutyState(current_duty_state);

    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_Start_USB_RX_Task */
/**
* @brief Function implementing the USB_RX_Check_Ta thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_USB_RX_Task */
void Start_USB_RX_Task(void const * argument)
{
  /* USER CODE BEGIN Start_USB_RX_Task */
	int32_t distance;
	int32_t id_command;
	DUTY_Command_TypeDef 	duty_cmd;
	Robot_CommandTypedef 	cmd_type;
	Robot_RespondTypedef 	rpd_type;
	uint8_t			 	detail[135];
	uint8_t				respond[145];
	uint8_t				message[150];
	int32_t				respond_lenght;
	int32_t				message_lenght;

	// Default value
	duty_cmd.key_speed = 1;

  /* Infinite loop */
  for(;;)
  {
	  for(;;) {
		  distance = ringBuff_DistanceOf(&usb_rx_ringbuff, END_CHAR);
		  if ( -1 != distance ) {
			  uint8_t temp[distance+1];
			  int32_t ret;
			  ringBuff_PopArray(&usb_rx_ringbuff, temp, distance + 1);
			  ret = unPackPayload(temp, distance + 1);
			  if( -1 == ret) {
				  LOG_REPORT("UNPACK FAIL", __LINE__);
			  } else {
				  LOG_REPORT((char*) temp, __LINE__);
				  cmd_type = commandRead(temp, &id_command, &duty_cmd);
				  memset(detail, 0, sizeof(detail));
				  rpd_type = commandReply(cmd_type, duty_cmd, detail);

				  if ( RPD_DUTY == rpd_type) {
					  DUTY_Command_TypeDef *dataMail;
					  dataMail = NULL;
					  // Wait allocate
					  while (dataMail == NULL) {
						  dataMail = osMailAlloc(commandMailHandle, osWaitForever);
					  }
					  memcpy( dataMail, &duty_cmd, sizeof(DUTY_Command_TypeDef));
					  osStatus result;
					  result = osMailPut(commandMailHandle, dataMail);
					  if (osOK == result) {
						  //LOG_REPORT("DUTY SEND", __LINE__);
					  }

				  } else {
					  memset(respond, 0, sizeof(respond));
					  memset(message, 0, sizeof(message));
					  respond_lenght	= commandRespond(rpd_type, id_command,
							  	  	  	  (char *)detail,
										  (char *)respond);
					  message_lenght	= packPayload(respond, message, respond_lenght);
					  // Mutex
					  osMutexWait(usbTxMutexHandle, osWaitForever);
					  ringBuff_PushArray(&cmd_tx_ringbuff, message, message_lenght);
					  osMutexRelease(usbTxMutexHandle);
				  }
			  }
		  }
	  }
    osDelay(1);// Never go to this place
  }
  /* USER CODE END Start_USB_RX_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
