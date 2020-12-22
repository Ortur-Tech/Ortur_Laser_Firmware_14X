/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  * Copyright (c) 2018-2019 Thomas Truong
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
#include "stm32f1xx_hal.h"
#include "stm32f1xx_ll_tim.h"
#include "stm32f1xx_ll_usart.h"
#include "stm32f1xx_ll_rcc.h"
#include "stm32f1xx_ll_bus.h"
#include "stm32f1xx_ll_system.h"
#include "stm32f1xx_ll_exti.h"
#include "stm32f1xx_ll_cortex.h"
#include "stm32f1xx_ll_utils.h"
#include "stm32f1xx_ll_pwr.h"
#include "stm32f1xx_ll_dma.h"
#include "stm32f1xx.h"
#include "stm32f1xx_ll_gpio.h"
#include "stm32utilities.h"

void Error_Handler(void);

/* USER CODE BEGIN EFP */

#define COOL_FLOOD_GPIO_Port 		GPIOA
#define COOL_FLOOD_Pin 				GPIO_PIN_5
#define COOL_MIST_GPIO_Port 		GPIOA
#define COOL_MIST_Pin 				GPIO_PIN_5
#define PROBE_GPIO_Port 			GPIOA
#define PROBE_Pin 					GPIO_PIN_5
#define PROBE_MASK					GPIO_PIN_5

#define SPIN_DIR_GPIO_Port 			GPIOA
#define SPIN_DIR_Pin 				GPIO_PIN_5

#define STEP_ENABLE_GPIO_Port 		GPIOB
#define STEP_ENABLE_Pin 			GPIO_PIN_7

#define CON_CYCLE_START_GPIO_Port 		GPIOA
#define CON_CYCLE_START_Pin 			GPIO_PIN_5

#define CON_SAFETY_DOOR_GPIO_Port 		GPIOA
#define CON_SAFETY_DOOR_Pin 			GPIO_PIN_5

#define CON_RESET_GPIO_Port 			GPIOA
#define CON_RESET_Pin 					GPIO_PIN_5

#define CON_FEED_HOLD_GPIO_Port 		GPIOA
#define CON_FEED_HOLD_Pin 				GPIO_PIN_5
#define AUX_1_Pin 						GPIO_PIN_5
#define AUX_1_GPIO_Port 				GPIOA
#define AUX_2_Pin 						GPIO_PIN_5
#define AUX_2_GPIO_Port 				GPIOA
#define AUX_3_Pin 						GPIO_PIN_5
#define AUX_3_GPIO_Port 				GPIOA
#define AUX_4_Pin 						GPIO_PIN_5
#define AUX_4_GPIO_Port 				GPIOA
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY_Pin 						GPIO_PIN_13
#define KEY_GPIO_Port 					GPIOC
#define LIM_Y_Pin 						GPIO_PIN_0
#define LIM_Y_GPIO_Port 				GPIOA
#define LIM_Y_EXTI_IRQn 				EXTI0_IRQn
#define LIM_X_Pin 						GPIO_PIN_1
#define LIM_X_GPIO_Port 				GPIOA
#define LIM_X_EXTI_IRQn 				EXTI1_IRQn
#define LIM_Z_Pin 						GPIO_PIN_2
#define LIM_Z_GPIO_Port 				GPIOA
#define AMIN_Pin 						GPIO_PIN_4
#define AMIN_GPIO_Port 					GPIOA
#define BEEP_F_Pin 						GPIO_PIN_7
#define BEEP_F_GPIO_Port 				GPIOA
#define SPIN_EN_Pin 					GPIO_PIN_12
#define SPIN_EN_GPIO_Port 				GPIOB
#define EN_Z_Pin 						GPIO_PIN_13
#define EN_Z_GPIO_Port 					GPIOB
#define DIR_Z_Pin 						GPIO_PIN_14
#define DIR_Z_GPIO_Port 				GPIOB
#define STEP_Z_Pin 						GPIO_PIN_15
#define STEP_Z_GPIO_Port 				GPIOB
#define PWM_SPIN_Pin 					GPIO_PIN_8
#define PWM_SPIN_GPIO_Port 				GPIOA
#define STATUS_LED_Pin 					GPIO_PIN_15
#define STATUS_LED_GPIO_Port 			GPIOA
#define POWER_LED_Pin 					GPIO_PIN_3
#define POWER_LED_GPIO_Port 			GPIOB
#define EN_Y_Pin 						GPIO_PIN_4
#define EN_Y_GPIO_Port 					GPIOB
#define STEP_X_Pin 						GPIO_PIN_5
#define STEP_X_GPIO_Port 				GPIOB
#define STEP_Y_Pin 						GPIO_PIN_6
#define STEP_Y_GPIO_Port 				GPIOB
#define EN_X_Pin 						GPIO_PIN_7
#define EN_X_GPIO_Port 					GPIOB
#define DIR_X_Pin 						GPIO_PIN_8
#define DIR_X_GPIO_Port 				GPIOB
#define DIR_Y_Pin 						GPIO_PIN_9
#define DIR_Y_GPIO_Port 				GPIOB
/* USER CODE BEGIN Private defines */
#include "stm32_pin_out.h"
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

