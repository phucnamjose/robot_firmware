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
#include <stdio.h>
#include <string.h>
#include <malloc.h>
#include "dma.h"
#include "usart.h"
#include "common_def.h"
#include "ringbuffer.h"
#include "robot_scara.h"
#include "command_respond.h"
#include "kinematic.h"
#include "communicate_payload.h"
#include "usbd_cdc_if.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SIMULATION
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

extern RINGBUFFER_TypeDef 			usb_rx_ringbuff;
extern RINGBUFFER_TypeDef 			cmd_tx_ringbuff;
extern const char 					*DETAIL_STATUS[NUM_OF_STATUS];
extern SCARA_PositionTypeDef		positionCurrent;
extern SCARA_PositionTypeDef		positionNext;

osMailQId commandMailHandle;
osMailQId taskMailHandle;
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
  osEvent 				ret_mail;
  DUTY_Command_TypeDef 	duty_cmd;
  DUTY_Command_TypeDef 	*dataMail;
  uint8_t 				isNewDuty = FALSE;
  int32_t 				no_stop;
  int32_t 				no_scan;
  int32_t 				no_duty;
  int32_t 				no_duty_success;
  int32_t 				no_duty_fail;

  // Report buffer;
  uint8_t				respond[40];
  int32_t				respond_lenght;
  uint8_t				position[85];
  uint8_t				infor[90];
  int32_t				infor_lenght;
  uint8_t				task_usb[100];
  int32_t				task_usb_lenght;

  uint8_t				respond_packed[50];
  int32_t				respond_packed_lenght;
  uint8_t				infor_packed[100];
  int32_t				infor_packed_lenght;
  uint8_t				usb_buff[256];
  int32_t				usb_lenght;

  // Robot variable
  SCARA_ModeTypeDef			current_mode;
  SCARA_DutyStateTypeDef 	current_state;
  float						run_time;

  LOG_REPORT("free_rtos.c: PROGRAM START...", __LINE__);

  // Init value
  no_stop = 0;
  no_scan = 0;
  no_duty = 0;
  no_duty_success	= 0;
  no_duty_fail		= 0;
  current_mode	 = scaraGetMode();
  current_state	 = scaraGetDutyState();

#ifdef SIMULATION
	  positionNext.Theta1 = -PI/3;
	  positionNext.Theta2 = PI/3;
	  positionNext.D3 = 10;
	  positionNext.Theta4 = 0;

	  kinematicForward(&positionNext);
#endif
  /* Infinite loop */

  for(;;)
  {
	  /* 1--- Reset Value ---*/
	  respond_lenght		= 0;
	  respond_packed_lenght = 0;
	  infor_lenght			= 0;
	  infor_packed_lenght	= 0;
	  task_usb_lenght		= 0;
	  usb_lenght			= 0;
	  // Update new position
#ifdef SIMULATION
	  memcpy(&positionCurrent, &positionNext, sizeof(SCARA_PositionTypeDef));
#endif

	  /* 2--- Check New Duty Phase ---*/
	  // Check mail
	  ret_mail = osMailGet(commandMailHandle, 0);
	  if ( ret_mail.status == osEventMail) {
		   dataMail = ret_mail.value.p;
		   memcpy( &duty_cmd, dataMail, sizeof(DUTY_Command_TypeDef));
		   isNewDuty = TRUE;
		   osMailFree(commandMailHandle, dataMail);/* free memory allocated for mail */
		   LOG_REPORT("Receive mail", __LINE__);
	  }
	  if(isNewDuty) {
		  memset(respond, 0, 40);
		  switch( duty_cmd.robot_mode) {
		  case SCARA_MODE_STOP:
			  {
				  no_stop++;
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
				  if (SCARA_MODE_DUTY == current_mode && SCARA_DUTY_STATE_READY == current_state) {
					  no_scan++;
					  current_mode = SCARA_MODE_SCAN;
					  respond_lenght = commandRespond(RPD_OK,
							  	  	  	  	  	  	  duty_cmd.id_command,
													  "Start Scan",
													  (char *)respond);
					  LOG_REPORT("SCAN", __LINE__);
				  } else {
					  respond_lenght = commandRespond(RPD_ERROR,
							  	  	  	  	  	  	  duty_cmd.id_command,
													  "Busy",
													  (char *)respond);
					  LOG_REPORT("SCAN FAIL:NOT SCAN", __LINE__);
				  }
			  }
			  break;
		  case SCARA_MODE_DUTY:
			  {
				  no_duty++;
				  if (SCARA_MODE_DUTY == current_mode && SCARA_DUTY_STATE_READY == current_state) {
					  if (scaraIsScanLimit()) {
						  current_mode	 = SCARA_MODE_DUTY;
						  current_state	 = SCARA_DUTY_STATE_INIT;
					  } else {
						  no_duty_fail++;
						  respond_lenght = commandRespond(RPD_ERROR,
						  							  	  duty_cmd.id_command,
														  "Has Not Scan Yet.",
														  (char *)respond);
						  LOG_REPORT("MOVE FAIL:NOT SCAN", __LINE__);
					  }
				  } else {
					  no_duty_fail++;
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
		  isNewDuty = FALSE;
	  }

	/* 3--- Execute Phase ---*/
	  switch( current_mode) {
	  case SCARA_MODE_STOP:
		  {
			  current_mode 	= SCARA_MODE_DUTY;
			  current_state = SCARA_DUTY_STATE_READY;
		  }
		  break;
	  case SCARA_MODE_SCAN:
		  {
			  current_mode 	= SCARA_MODE_DUTY;
			  current_state = SCARA_DUTY_STATE_READY;
		  }
		  break;
	  case SCARA_MODE_DUTY:
		  {
			  switch (current_state){
			  case SCARA_DUTY_STATE_READY:
				  {
					  // Do nothing();
					  __NOP();
				  }
			  break;
			  case SCARA_DUTY_STATE_INIT:
				  {
					  SCARA_StatusTypeDef status;
					  status = scaraInitDuty(duty_cmd);
					  if ( SCARA_STATUS_OK == status) {
						  no_duty_success++;
						  current_state		= SCARA_DUTY_STATE_FLOW;
						  run_time			= 0;
						  // Respond
						  respond_lenght 	= commandRespond(RPD_OK,
								  	  	  	  	  	  	  	  duty_cmd.id_command,
															  (char *)DETAIL_STATUS[status],
															  (char *)respond);
						  scaraPosition2String((char *)position, positionCurrent);
						  // Start Inform
						  infor_lenght 		= commandRespond(RPD_START,
		  	  	  	  	  	  	  	  	  	  	  	  	  	  0,
															  (char *)position,
															  (char *)infor);
						  LOG_REPORT("INIT SUCCESS", __LINE__);
					  } else {
						  no_duty_fail++;
						  current_state 	= SCARA_DUTY_STATE_READY;
						  respond_lenght	= commandRespond(RPD_ERROR,
								  	  	  	  	  	  	  	  duty_cmd.id_command,
															  (char *)DETAIL_STATUS[status],
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
						  current_state = SCARA_DUTY_STATE_FINISH;// Work Done
					  } else {
						  status = scaraFlowDuty(run_time);
						  if ( SCARA_STATUS_OK == status) {
							  // lowLevelExcute();
							  // Running Inform
							  scaraPosition2String((char *)position, positionCurrent);
							  infor_lenght = commandRespond(RPD_RUNNING,
									  	  	  	  	  	  	0,
															(char *)position,
															(char *)infor);
						  } else {
							  current_state = SCARA_DUTY_STATE_FINISH;
							  // Critical
							  // If appear a error while Flow, This is very important
							  infor_lenght = commandRespond(RPD_STOP,
															0,
															(char *)DETAIL_STATUS[status],
															(char *)infor);
							  LOG_REPORT("STOP", __LINE__);
						  }
					  }
				  }
			  break;
			  case SCARA_DUTY_STATE_FINISH:
				  {
					  current_state = SCARA_DUTY_STATE_READY;
					  // Done Inform
					  scaraPosition2String((char *)position, positionCurrent);
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
	  scaraSetMode(current_mode);
	  scaraSetDutyState(current_state);

    osDelay(10);
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
	int32_t no_duty;
	int32_t no_other;
	DUTY_Command_TypeDef 	duty_cmd;
	Robot_CommandTypedef 	cmd_type;
	Robot_RespondTypedef 	rpd_type;
	uint8_t			 	detail[85];
	uint8_t				respond[100];
	uint8_t				message[110];
	int32_t				respond_lenght;
	int32_t				message_lenght;

	//uint8_t test_command1[60] = "(1 ROTA 3 0.785398 0.3 0 0.3)";
	//uint8_t test_command2[60] = "(456 OUTP 1)";

	//ringBuff_PushArray(&usb_rx_ringbuff, test_command1, strlen((char *)test_command1));
	//ringBuff_PushArray(&usb_rx_ringbuff, test_command2, strlen((char *)test_command2));
	no_duty	 = 0;
	no_other = 0;


  /* Infinite loop */
  for(;;)
  {
	  for(;;) {
		  distance = ringBuff_DistanceOf(&usb_rx_ringbuff, END_CHAR);
		  if ( -1 != distance ) {
			  LOG_REPORT("NEW PACKET", __LINE__);
			  uint8_t temp[distance+1];
			  int32_t ret;
			  ringBuff_PopArray(&usb_rx_ringbuff, temp, distance + 1);
			  ret = unPackPayload(temp, distance + 1);
			  if( -1 == ret) {
				  LOG_REPORT("UNPACK FAIL", __LINE__);
			  } else {
				  LOG_REPORT("UNPACK SUCCESS", __LINE__);
				  LOG_REPORT((char*) temp, __LINE__);
				  cmd_type = commandRead(temp, &id_command, &duty_cmd);
				  memset(detail, 0, sizeof(detail));
				  rpd_type = commandReply(cmd_type, duty_cmd, detail);

				  if ( RPD_DUTY == rpd_type) {
					  no_duty++;
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
						  LOG_REPORT("DUTY SEND", __LINE__);
					  }

				  } else {
					  no_other++;
					  memset(respond, 0, sizeof(respond));
					  memset(message, 0, sizeof(message));
					  respond_lenght	= commandRespond(rpd_type, id_command,
							  	  	  	  (char *)detail,
										  (char *)respond);
					  message_lenght	= packPayload(respond, message, respond_lenght);
					  // Mutex
					  osMutexWait(usbTxMutexHandle, osWaitForever);
					  ringBuff_PushArray(&cmd_tx_ringbuff, message, message_lenght);
					  LOG_REPORT("ADD RINGBUFF", __LINE__);
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
