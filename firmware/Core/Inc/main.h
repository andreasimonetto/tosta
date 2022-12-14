/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
#define ENABLE_I2C 0
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
#define nSSEL_Pin GPIO_PIN_9
#define nSSEL_GPIO_Port GPIOB
#define EN1A_Pin GPIO_PIN_0
#define EN1A_GPIO_Port GPIOA
#define EN1A_EXTI_IRQn EXTI0_1_IRQn
#define LCD_CLK_Pin GPIO_PIN_1
#define LCD_CLK_GPIO_Port GPIOA
#define LCD_Din_Pin GPIO_PIN_2
#define LCD_Din_GPIO_Port GPIOA
#define LCD_nRST_Pin GPIO_PIN_3
#define LCD_nRST_GPIO_Port GPIOA
#define LCD_nCE_Pin GPIO_PIN_4
#define LCD_nCE_GPIO_Port GPIOA
#define EN1SW_Pin GPIO_PIN_5
#define EN1SW_GPIO_Port GPIOA
#define EN1SW_EXTI_IRQn EXTI4_15_IRQn
#define LCD_DC_Pin GPIO_PIN_6
#define LCD_DC_GPIO_Port GPIOA
#define EN1B_Pin GPIO_PIN_7
#define EN1B_GPIO_Port GPIOA
#define EN1B_EXTI_IRQn EXTI4_15_IRQn
#define SPKR_Pin GPIO_PIN_0
#define SPKR_GPIO_Port GPIOB
#define JOY_BT_Pin GPIO_PIN_1
#define JOY_BT_GPIO_Port GPIOB
#define JOY_BT_EXTI_IRQn EXTI0_1_IRQn
#define JOY_RX_Pin GPIO_PIN_2
#define JOY_RX_GPIO_Port GPIOB
#define JOY_RX_EXTI_IRQn EXTI2_3_IRQn
#define JOY_UP_Pin GPIO_PIN_8
#define JOY_UP_GPIO_Port GPIOA
#define JOY_UP_EXTI_IRQn EXTI4_15_IRQn
#define SCL_Pin GPIO_PIN_9
#define SCL_GPIO_Port GPIOA
#define JOY_LX_Pin GPIO_PIN_6
#define JOY_LX_GPIO_Port GPIOC
#define JOY_LX_EXTI_IRQn EXTI4_15_IRQn
#define SDA_Pin GPIO_PIN_10
#define SDA_GPIO_Port GPIOA
#define JOY_DN_Pin GPIO_PIN_11
#define JOY_DN_GPIO_Port GPIOA
#define JOY_DN_EXTI_IRQn EXTI4_15_IRQn
#define SW4_Pin GPIO_PIN_12
#define SW4_GPIO_Port GPIOA
#define SW4_EXTI_IRQn EXTI4_15_IRQn
#define SW3_Pin GPIO_PIN_15
#define SW3_GPIO_Port GPIOA
#define SW3_EXTI_IRQn EXTI4_15_IRQn
#define SW2_Pin GPIO_PIN_3
#define SW2_GPIO_Port GPIOB
#define SW2_EXTI_IRQn EXTI2_3_IRQn
#define SW1_Pin GPIO_PIN_4
#define SW1_GPIO_Port GPIOB
#define SW1_EXTI_IRQn EXTI4_15_IRQn
#define LCD_BL_Pin GPIO_PIN_5
#define LCD_BL_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
