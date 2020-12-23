/**
  ******************************************************************************
  * File Name          : IWDG.c
  * Description        : This file provides code for the configuration
  *                      of the IWDG instances.
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
#include "iwdg.h"

/* USER CODE BEGIN 0 */
#include "system.h"
#include "usb_device.h"
/* USER CODE END 0 */

IWDG_HandleTypeDef hiwdg;

/**
 * @brief IWDG init function 1s最长喂狗时间
 */
void IWDG_Init(void)
{

	IWDG->KR=0X5555;//使能对IWDG->PR和IWDG->RLR的写
  	IWDG->PR=6;  //设置分频系数
  	IWDG->RLR=625;  //从加载寄存器 IWDG->RLR
	IWDG->KR=0XAAAA;//reload
  	IWDG->KR=0XCCCC;//使能看门狗

  	__HAL_DBGMCU_FREEZE_IWDG();

}

/* USER CODE BEGIN 1 */
#ifdef ORTUR_LASER_MODE

int32_t last_sys_position[N_AXIS]; 		//!< 最后一次检测位置
uint64_t last_check_timestamp = 0; 		// 最后一次检测时间
uint32_t max_exposure_time = 60;		// 最长曝光时间 100s
uint32_t min_exposure_time = 10;		//最短曝光时间
uint32_t max_weak_time = 100;			//最长弱光时间

uint32_t curr_laser_power;//当前激光功率

uint32_t allow_laser_time;//允许激光静态开启时间

uint32_t max_laser_power = SPINDLE_PWM_MAX_VALUE; ///< 激光最大功率
uint32_t weak_laser_power = 20;//弱光功率
uint32_t off_laser_power = SPINDLE_PWM_OFF_VALUE;//认为激光关闭的功率
#endif

/**
 * @brief IWDG_Feed  喂狗同时检测激光是否在固定位置一直开着
 */
void IWDG_Feed(void)
{
#ifdef ORTUR_LASER_MODE
    //检查xyz位置是否变化
	if(sys_position[0] != last_sys_position[0] ||
	   sys_position[1] != last_sys_position[1])
	{
		last_sys_position[0] = sys_position[0];
		last_sys_position[1] = sys_position[1];
		last_check_timestamp = HAL_GetTick()/1000;
	}

	//注意,激光已经开启
	if(isLaserOpen())
	{
		//激光功率过大且长时间未移动
		curr_laser_power = getLaserPower();
		if(curr_laser_power > off_laser_power )
		{
#ifdef USEUSB
			//usb线未连接
			if(!isUsbPlugIn())
			{
				return;
			}
#endif
			if(curr_laser_power > max_weak_time )
			{
				allow_laser_time = min_exposure_time + (max_exposure_time - min_exposure_time) *
						           (max_laser_power - getLaserPower()) /(max_laser_power);
			}
			else
			{
				allow_laser_time = max_weak_time;
			}

			if((HAL_GetTick()/1000) - last_check_timestamp >= allow_laser_time)
			{
				printStringAll("Laser exposure timeout!\r\n");
				sys.state = STATE_ALARM;
				sys.abort = 1;
			}
		}
	}
	else
	{
		last_check_timestamp = HAL_GetTick()/1000;
	}
#endif
#ifndef DEBUG
	IWDG->KR=0XAAAA;//reload
#endif
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
