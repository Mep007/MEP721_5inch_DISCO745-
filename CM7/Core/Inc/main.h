/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <string.h>
#include <stdint.h>

#include "pt-1.4/pt.h"

// for LVGL port to STM32H745i-DISCO
#include "stm32h745i_discovery.h"
#include "stm32h745i_discovery_sdram.h"
//#include "stm32h745i_discovery_lcd.h"
#include "stm32h745i_discovery_ts.h"
#include "LVGL_port_screen.h"
#include "LVGL_port_touch.h"
#include "lvgl.h"
#include "lv_conf.h"
#include "lvgl/demos/lv_demos.h"


/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern LTDC_HandleTypeDef  hltdc;
extern DMA2D_HandleTypeDef hdma2d;
extern TIM_HandleTypeDef   htim8;        // MEP - LCD BL PWM control
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
typedef struct
{
  struct pt     pt;
  bool      	enable;

  u_int16_t     target;      // for PWM 0-100
  u_int16_t     target_old;  // save old value
  bool          dir;         // directing for brightness change (+/-)

} tBLtype;

typedef struct
{
  tBLtype   BL;

} tMyLCD;

extern tMyLCD myLCD;

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
#define VSYNC_FREQ_Pin GPIO_PIN_6
#define VSYNC_FREQ_GPIO_Port GPIOB
#define RENDER_TIME_Pin GPIO_PIN_7
#define RENDER_TIME_GPIO_Port GPIOB
#define BUT_USR_Pin GPIO_PIN_13
#define BUT_USR_GPIO_Port GPIOC
#define BUT_USR_EXTI_IRQn EXTI15_10_IRQn
#define FRAME_RATE_Pin GPIO_PIN_3
#define FRAME_RATE_GPIO_Port GPIOG
#define INT_TOUCH_Pin GPIO_PIN_2
#define INT_TOUCH_GPIO_Port GPIOG
#define INT_TOUCH_EXTI_IRQn EXTI2_IRQn
#define LCD_BL_PWM_Pin GPIO_PIN_0
#define LCD_BL_PWM_GPIO_Port GPIOK
#define MCU_ACTIVE_Pin GPIO_PIN_6
#define MCU_ACTIVE_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
