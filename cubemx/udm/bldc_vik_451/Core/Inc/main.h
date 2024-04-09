/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define pairs 7   //number of pole pairs
#define power 900
#define clockwise 1
#define counterclockwise -1
#define trigger_position_for_clockwise 0
#define trigger_position_for_counterclockwise 125
#define rate_for_changing_direction 10
#define set_PMSM_PWM_min 30
#define set_PMSM_PWM_max 900
#define set_PMSM_PWM_middle 50
#define set_timing_max 40
#define set_timing_min 0
#define HALL_UP 1
#define HALL_DOWN 0
#define set_timing_k 0.0016
#define set_timing_b 0.3852
#define set_PMSM_PWM_k 0.0494
#define tim6_counter_period_high 3000
#define set_PMSM_PWM_b_max 200
#define set_PMSM_PWM_b_regular 56
#define speed_to_mean_rate 10.28*1000000
#define calculation_speed_correct_k 5.7
#define calculation_speed_correct_b 88

// Sin table
#define PMSM_SINTABLESIZE	192
#define PMSM_SINTABLE_max_index	191
#define max_sin_value 255
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
