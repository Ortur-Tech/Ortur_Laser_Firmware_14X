/**
  ******************************************************************************
  * File Name          : gpio.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
/* USER CODE BEGIN 0 */
#define KEY_ON 0
#define KEY_OFF 1

#define POWEROFF_COUNT 200 //*10ms
#define POWERON_COUNT 50 //*10ms

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SPIN_EN_Pin|EN_Z_Pin|EN_Y_Pin|EN_X_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIR_Z_Pin|STEP_Z_Pin|POWER_LED_Pin|STEP_X_Pin
                          |STEP_Y_Pin|DIR_X_Pin|DIR_Y_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin PAPin */
  GPIO_InitStruct.Pin = LIM_Y_Pin|LIM_X_Pin|LIM_Z_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = AMIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(AMIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin PBPin PBPin
                           PBPin PBPin PBPin PBPin
                           PBPin PBPin */
  GPIO_InitStruct.Pin = SPIN_EN_Pin|EN_Z_Pin|DIR_Z_Pin|STEP_Z_Pin
                          |POWER_LED_Pin|EN_Y_Pin|STEP_Y_Pin|EN_X_Pin
                          |DIR_X_Pin|DIR_Y_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = STATUS_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STATUS_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = STEP_X_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(STEP_X_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

void PowerLed_Blink(void)
{
	HAL_GPIO_TogglePin(POWER_LED_GPIO_Port, POWER_LED_Pin);
}
void PowerLed_On(void)
{
	HAL_GPIO_WritePin(POWER_LED_GPIO_Port, POWER_LED_Pin, GPIO_PIN_SET);
}
void PowerLed_Off(void)
{
	HAL_GPIO_WritePin(POWER_LED_GPIO_Port, POWER_LED_Pin, GPIO_PIN_RESET);
}


void StatusLed_On(void)
{
	HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_SET);
}
void StatusLed_Off(void)
{
	HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);
}
void StatusLed_Blink(void)
{
	HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
}
uint8_t Key_Scan(void)
{
	if(0==HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
	{
		delay_ms(20);
		if(0==HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin))
		{
			return 1;
		}
	}
	return 0;
}

uint32_t key_count = 0;
uint8_t poweron_justnow = 0;

void PowerOpen_Loop()
{
	//默认关机
	key_count = 0;
	//初始化按�?
	//Key_GPIO_Config();

	//debugStr("wait for power on.\n");
	while(1)
	{
		if(Key_Scan() == KEY_ON)
		{
			++key_count;
			if(!(key_count%10))
				PowerLed_Blink();
		}
		else
		{
			if(key_count > 0)
				--key_count;
			PowerLed_Off();
		}

		//按键超过 200 ms 就开�?
		//执行归位操作
		if(key_count > POWERON_COUNT)
		{
			//打开led
			PowerLed_On();
			poweron_justnow = POWERON_COUNT;
			debugStr("power on.\n");
			break;
		}
	}
	key_count = 0;
}

void PowerClose_Check()
{
	//长按关机
	if(Key_Scan() == KEY_ON)
	{
		if(poweron_justnow == 0)
		{
			++key_count;
			if(!(key_count%10))
			{
				debugStr("key pressed.\n");
				PowerLed_Blink();
			}
		}
	}
	else
	{
		if(key_count > 0)
		{
			--key_count;
		}
		if(poweron_justnow > 0)
		{
			--poweron_justnow;
		}

		//电源按钮未按下时,指示固件状�??
		//警告状�??,1秒闪烁一�?
		if(key_count == 0)
		{
			if(sys.state == STATE_ALARM)
			{
				PowerLed_Blink_Limit(1000);
			}
			else
			{
				PowerLed_Blink_Limit(0);
				PowerLed_On();
			}
		}
		else
		{
			PowerLed_Blink_Limit(0);
			PowerLed_On();
		}
	}

	//按键超过 2000 ms 就开�?
	//关机操作
	if(key_count > POWEROFF_COUNT)
	{

		Leds_Power(0); //给信号灯供电

		//关机动作
		PowerLed_Off();
		StatusLed_Off();

		debugStr("power off.\n");

		// Immediately disables steppers
		st_go_idle();

		// Stop and start spindle routines. Called by all spindle routines and stepper ISR.
		spindle_stop();

		// Immediately disables coolant pins.
		coolant_stop();

		//等待用户释放按钮
		debugStr("wait for key release.\n");
		key_count = 0;
		while(1)
		{
			IWDG_Feed();

			if(Key_Scan() == KEY_OFF)
			{
				delay_ms(10);
				++key_count;
			}
			else
			{
				if(key_count > 0)
					--key_count;
			}

			//按键超过 1000 ms 就开�?
			//执行归位操作
			if(key_count > POWERON_COUNT)
			{
				//关闭led
				PowerLed_Off();
				debugStr("key release.\n");
				delay_ms(200);

				__disable_fault_irq();
				NVIC_SystemReset();
				break;
			}
		}
	}
}

/* USER CODE END 2 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
