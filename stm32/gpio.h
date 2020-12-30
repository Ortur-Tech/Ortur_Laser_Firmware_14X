/**
  ******************************************************************************
  * File Name          : gpio.h
  * Description        : This file contains all the functions prototypes for
  *                      the gpio
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
#ifndef __gpio_H
#define __gpio_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */
//void PowerLed_Blink(void);
//void PowerLed_On(void);
//void PowerLed_Off(void);
//void StatusLed_On(void);
//void StatusLed_Off(void);
//void StatusLed_Blink(void);


/**
 * @brief PowerLed_Blink
 */
static inline void PowerLed_Blink(void)
{
	LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_15);
}
/**
 * @brief PowerLed_On
 */
static inline void PowerLed_On(void)
{
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_15);
}
/**
 * @brief PowerLed_Off
 */
static inline void PowerLed_Off(void)
{
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_15);
}

/**
 * @brief StatusLed_On
 */
static inline void StatusLed_On(void)
{
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_3);
}
/**
 * @brief StatusLed_Off
 */
static inline void StatusLed_Off(void)
{
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_3);
}
/**
 * @brief StatusLed_Blink
 */
static inline void StatusLed_Blink(void)
{
	LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_3);
}

void Leds_Power(uint8_t onoff);
uint8_t Key_Scan(void);
void PowerOpen_Loop();
void PowerClose_Check();
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ pinoutConfig_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
