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

/* USER CODE END 0 */

IWDG_HandleTypeDef hiwdg;

/* IWDG init function */
void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 256;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }

}

/* USER CODE BEGIN 1 */
void Feed_IWDG(void)
{
#ifdef ORTUR_LASER_MODE
	//�?查xyz位置是否变化
	if(sys_position[0] != last_sys_position[0] ||
	   sys_position[1] != last_sys_position[1])
	{
		last_sys_position[0] = sys_position[0];
		last_sys_position[1] = sys_position[1];
		last_check_timestamp = sys_seconds();
	}

	//注意,�?光已经开�?
	if(isLaserOpen())
	{
		//�?光功率过大且长时间未移动
		curr_laser_power = getLaserPower();
		if(curr_laser_power > off_laser_power )
		{
#ifdef USEUSB
			//usb线未连接
			if(!isUSBConnect())
			{
				return;
			}
#endif
			if(curr_laser_power > max_weak_time )
			{
				allow_laser_time = min_exposure_time + (float)(max_exposure_time - min_exposure_time) *
						           (float)(max_laser_power - getLaserPower()) / (float)(max_laser_power);
			}
			else
			{
				allow_laser_time = max_weak_time;
			}

			if(sys_seconds() - last_check_timestamp >= allow_laser_time)
			{
				printString("Exceeding the maximum exposure time of the laser!\r\n");
				printString("In weak laser mode, the maximum time allowed is 100 second.\r\n");
				printString("None weak laser mode , the maximum time allowed is in the range from 10 to 60 second , according to the power .\r\n");
				printString("Reset Grbl.\r\n");

				sys.state = STATE_ALARM;
				sys.abort = 1;
			}
		}
	}
	else
	{
		last_check_timestamp = sys_seconds();
	}
#endif
	HAL_IWDG_Refresh(&hiwdg);
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
