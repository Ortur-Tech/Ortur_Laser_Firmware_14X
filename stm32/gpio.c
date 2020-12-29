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
#include "iwdg.h"
#include "usb_device.h"

#define KEY_ON 0
#define KEY_OFF 1

#define POWEROFF_COUNT 200 //*10ms
#define POWERON_COUNT 50 //*10ms

/**
 * @brief 初始化GPIO
 */
void MX_GPIO_Init(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	/* GPIO Ports Clock Enable */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOD);

	/*Configure GPIO pin : PtPin */
	LL_GPIO_SetPinPull(KEY_GPIO_Port, LL_GPIO_PIN_13, LL_GPIO_PULL_UP);
	LL_GPIO_SetPinMode(KEY_GPIO_Port, LL_GPIO_PIN_13, LL_GPIO_MODE_INPUT);

	/*Configure GPIO pins : PAPin PAPin PAPin */
	LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_0, LL_GPIO_PULL_UP);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_1, LL_GPIO_PULL_UP);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_2, LL_GPIO_PULL_UP);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_INPUT);
	LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_3, LL_GPIO_PULL_UP);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_3, LL_GPIO_MODE_INPUT);

	/*Configure GPIO pin : PtPin */
	LL_GPIO_SetPinPull(AMIN_GPIO_Port, LL_GPIO_PIN_4, LL_GPIO_PULL_UP);
	LL_GPIO_SetPinMode(AMIN_GPIO_Port, LL_GPIO_PIN_4, LL_GPIO_MODE_INPUT);

	/*Configure GPIO pins : PBPin PBPin PBPin PBPin
						   PBPin PBPin PBPin PBPin
						   PBPin PBPin */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_12|LL_GPIO_PIN_13|LL_GPIO_PIN_14|LL_GPIO_PIN_15
		  |LL_GPIO_PIN_3|LL_GPIO_PIN_4|LL_GPIO_PIN_6|LL_GPIO_PIN_7
		  |LL_GPIO_PIN_8|LL_GPIO_PIN_9;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : PtPin */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_15|LL_GPIO_PIN_12;
	LL_GPIO_Init(STATUS_LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : PtPin */
	GPIO_InitStruct.Pin = LL_GPIO_PIN_5;
	LL_GPIO_Init(STEP_X_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin Output Level */
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_12|LL_GPIO_PIN_13|LL_GPIO_PIN_4|LL_GPIO_PIN_7);

	/*Configure GPIO pin Output Level */
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_14|LL_GPIO_PIN_15|LL_GPIO_PIN_3|LL_GPIO_PIN_5
						  |LL_GPIO_PIN_6|LL_GPIO_PIN_8|LL_GPIO_PIN_9);

	/*Configure GPIO pin Output Level */
	LL_GPIO_ResetOutputPin(STATUS_LED_GPIO_Port, LL_GPIO_PIN_15);

}

/**
 * @brief 电源LED指示控制
 * @param onoff 1:开 0:关
 * 红灯熄灭：关机状态。 红灯闪烁：机器报警。红灯常亮：开机状态。
 * 蓝灯熄灭：USB未插。 蓝灯闪烁：USB有数据传输。蓝灯常亮：USB线插入状态。
 */
void Leds_Power(uint8_t onoff)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOC);
    LL_GPIO_SetPinPull(GPIOC, LL_GPIO_PIN_13, LL_GPIO_PULL_UP);
    LL_GPIO_SetPinMode(GPIOC, LL_GPIO_PIN_13, LL_GPIO_MODE_INPUT);
#if !defined(ORTUR_CNC_MODE)
	if(onoff)
	{
		//NOTE:新主板需要给指示灯供电
		LL_GPIO_SetOutputPin(GPIOA, GPIO_PIN_2);

		//将IO设置为输入模式
		LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_0, LL_GPIO_PULL_UP);
	    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT);
	    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_1, LL_GPIO_PULL_UP);
	    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_1, LL_GPIO_MODE_INPUT);
	    LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_2, LL_GPIO_PULL_UP);
	    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_2, LL_GPIO_MODE_INPUT);

		//启用限位开关
		limits_init();

		LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4); //A
	}
	else
	{


		//禁用限位开关
		limits_disable();

		//将IO设置为输出模式
		GPIO_InitStruct.Pin = LL_GPIO_PIN_0|LL_GPIO_PIN_1|LL_GPIO_PIN_2|GPIO_PIN_4;
	    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
	    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
	    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
	    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
	    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);


		//关闭限位开关指示灯
	    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_0); //Y
	    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1); //X
	    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_2); //Z
	    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_4); //A
	}

#endif
}

/**
 * @brief PowerLed_Blink
 */
void PowerLed_Blink(void)
{
	LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_15);
}
/**
 * @brief PowerLed_On
 */
void PowerLed_On(void)
{
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_15);
}
/**
 * @brief PowerLed_Off
 */
void PowerLed_Off(void)
{
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_15);
}

/**
 * @brief StatusLed_On
 */
void StatusLed_On(void)
{
	LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_3);
}
/**
 * @brief StatusLed_Off
 */
void StatusLed_Off(void)
{
	LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_3);
}
/**
 * @brief StatusLed_Blink
 */
void StatusLed_Blink(void)
{
	LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_3);
}
/**
 * @brief Key_Scan
 * @return 1:没有按键按下 0：有按键按下
 */
uint8_t Key_Scan(void)
{
	if(0==LL_GPIO_IsInputPinSet(KEY_GPIO_Port, LL_GPIO_PIN_13))
	{
		delay_ms(20);
		if(0==LL_GPIO_IsInputPinSet(KEY_GPIO_Port, LL_GPIO_PIN_13))
		{
			return 0;
		}
	}
	return 1;
}

uint32_t key_count = 0;
uint8_t poweron_justnow = 0;
/**
 * @brief 开机等待循环
 */
void PowerOpen_Loop()
{
	//默认关机
	key_count = 0;

	while(1)
	{
		if(Key_Scan() == KEY_ON)
		{
			++key_count;
			if(!(key_count%5))
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
			break;
		}
	}
	key_count = 0;
}
/**
 * @brief 电源led翻转时间
 * @param ms单位毫秒
 */
void PowerLed_Blink_Limit(uint32_t ms)
{
	static int32_t last_blink_time = 0;
	if(last_blink_time + ms <= HAL_GetTick())
	{
		PowerLed_Blink();
		last_blink_time = HAL_GetTick();
	}
}
/**
 * @brief 关机按键检测
 */
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
				PowerLed_On();
			}
		}
		else
		{
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
		// Immediately disables steppers
		st_go_idle();

		// Stop and start spindle routines. Called by all spindle routines and stepper ISR.
		spindle_stop();

		// Immediately disables coolant pins.
		coolant_stop();

		//等待用户释放按钮
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
				delay_ms(200);
				__disable_fault_irq();
				NVIC_SystemReset();
				break;
			}
		}
	}
}
