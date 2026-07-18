/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

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
#define M0_LED_36V_Pin GPIO_PIN_3
#define M0_LED_36V_GPIO_Port GPIOE
#define M0_LED_RUN_Pin GPIO_PIN_4
#define M0_LED_RUN_GPIO_Port GPIOE
#define M0_LED_ERR_Pin GPIO_PIN_5
#define M0_LED_ERR_GPIO_Port GPIOE
#define CORE_LED_SYS_Pin GPIO_PIN_3
#define CORE_LED_SYS_GPIO_Port GPIOC
#define CORE_LED_CAN_Pin GPIO_PIN_2
#define CORE_LED_CAN_GPIO_Port GPIOF
#define M1_LED_36V_Pin GPIO_PIN_0
#define M1_LED_36V_GPIO_Port GPIOA
#define M1_LED_RUN_Pin GPIO_PIN_1
#define M1_LED_RUN_GPIO_Port GPIOA
#define M0_IU_A13_Pin GPIO_PIN_2
#define M0_IU_A13_GPIO_Port GPIOA
#define M0_IW_A14_Pin GPIO_PIN_3
#define M0_IW_A14_GPIO_Port GPIOA
#define M1_LED_ERR_Pin GPIO_PIN_6
#define M1_LED_ERR_GPIO_Port GPIOA
#define M0_IV_A24_Pin GPIO_PIN_7
#define M0_IV_A24_GPIO_Port GPIOA
#define M1_IU_A31_Pin GPIO_PIN_1
#define M1_IU_A31_GPIO_Port GPIOB
#define M1_IW_A34_Pin GPIO_PIN_7
#define M1_IW_A34_GPIO_Port GPIOE
#define M1_WL_T11N_Pin GPIO_PIN_8
#define M1_WL_T11N_GPIO_Port GPIOE
#define M1_WH_T11_Pin GPIO_PIN_9
#define M1_WH_T11_GPIO_Port GPIOE
#define M1_VL_T12N_Pin GPIO_PIN_10
#define M1_VL_T12N_GPIO_Port GPIOE
#define M1_VH_T12_Pin GPIO_PIN_11
#define M1_VH_T12_GPIO_Port GPIOE
#define M1_UL_T13N_Pin GPIO_PIN_12
#define M1_UL_T13N_GPIO_Port GPIOE
#define M1_UH_T13_Pin GPIO_PIN_13
#define M1_UH_T13_GPIO_Port GPIOE
#define M1_IV_A41_Pin GPIO_PIN_14
#define M1_IV_A41_GPIO_Port GPIOE
#define M1_BRK_T1B_Pin GPIO_PIN_10
#define M1_BRK_T1B_GPIO_Port GPIOB
#define JOY_BTN_Pin GPIO_PIN_15
#define JOY_BTN_GPIO_Port GPIOB
#define M1_36V_A512_Pin GPIO_PIN_8
#define M1_36V_A512_GPIO_Port GPIOD
#define M1_NTC_A513_Pin GPIO_PIN_9
#define M1_NTC_A513_GPIO_Port GPIOD
#define JOY_X_A57_Pin GPIO_PIN_10
#define JOY_X_A57_GPIO_Port GPIOD
#define JOY_Y_A58_Pin GPIO_PIN_11
#define JOY_Y_A58_GPIO_Port GPIOD
#define M1_HW_T41_Pin GPIO_PIN_12
#define M1_HW_T41_GPIO_Port GPIOD
#define M1_HV_T42_Pin GPIO_PIN_13
#define M1_HV_T42_GPIO_Port GPIOD
#define M1_HU_T43_Pin GPIO_PIN_14
#define M1_HU_T43_GPIO_Port GPIOD
#define M0_HW_T31_Pin GPIO_PIN_6
#define M0_HW_T31_GPIO_Port GPIOC
#define M0_HV_T32_Pin GPIO_PIN_7
#define M0_HV_T32_GPIO_Port GPIOC
#define M0_HU_T33_Pin GPIO_PIN_8
#define M0_HU_T33_GPIO_Port GPIOC
#define MODE_CAN_EN_Pin GPIO_PIN_9
#define MODE_CAN_EN_GPIO_Port GPIOC
#define M0_NTC_A51_Pin GPIO_PIN_8
#define M0_NTC_A51_GPIO_Port GPIOA
#define M0_36V_A52_Pin GPIO_PIN_9
#define M0_36V_A52_GPIO_Port GPIOA
#define M0_WH_T81_Pin GPIO_PIN_15
#define M0_WH_T81_GPIO_Port GPIOA
#define M0_WL_T81N_Pin GPIO_PIN_10
#define M0_WL_T81N_GPIO_Port GPIOC
#define M0_VL_T84N_Pin GPIO_PIN_0
#define M0_VL_T84N_GPIO_Port GPIOD
#define M0_VH_T84_Pin GPIO_PIN_1
#define M0_VH_T84_GPIO_Port GPIOD
#define M0_BRK_T8B_Pin GPIO_PIN_2
#define M0_BRK_T8B_GPIO_Port GPIOD
#define M0_UL_T83N_Pin GPIO_PIN_5
#define M0_UL_T83N_GPIO_Port GPIOB
#define M0_UH_T83_Pin GPIO_PIN_9
#define M0_UH_T83_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
