/**
  ******************************************************************************
  * File Name          : FSMC.h
  * Description        : This file provides code for the configuration
  *                      of the FSMC peripheral.
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FSMC_H
#define __FSMC_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern NOR_HandleTypeDef hnor1;

/* USER CODE BEGIN Private defines */
void FSMC_Write(uint32_t ui_address, uint16_t ui_data);
uint16_t FSMC_Read(uint32_t ui_address);
/* USER CODE END Private defines */

void MX_FSMC_Init(void);
void HAL_NOR_MspInit(NOR_HandleTypeDef* hnor);
void HAL_NOR_MspDeInit(NOR_HandleTypeDef* hnor);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__FSMC_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
