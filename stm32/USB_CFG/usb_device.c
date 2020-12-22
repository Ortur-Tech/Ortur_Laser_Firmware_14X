/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usb_device.c
  * @version        : v2.0_Cube
  * @brief          : This file implements the USB Device
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

USBD_HandleTypeDef hUsbDeviceFS;
/**
 * @brief Reset_Usb 强制复位USB
 */
void Reset_Usb()
{
	HAL_Delay(250);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);
	HAL_Delay(250);
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);

}
static uint8_t usbCdcConnectFlag=0;
/**
 * @brief isUsbCDCConnected
 * @return 1：已连接 0：未连接
 */
uint8_t isUsbCDCConnected(void)
{
	if((usbCdcConnectFlag==1)&&(isUSBConnect()==1))
	{
		return usbCdcConnectFlag;
	}

	return 0;

}
/**
 * @brief setUsbCDCConnected
 * @param status
 */
void setUsbCDCConnected(uint8_t status)
{
	usbCdcConnectFlag=status;
}
/* USER CODE END 1 */
/**
 * @brief isUSBConnect
 * @return 1：已连接 0：未连接
 */
uint8_t isUSBConnect(void)
{
	if(hUsbDeviceFS.dev_state== USBD_STATE_CONFIGURED)
	{
		return 1;
	}
	return 0;
}
/**
  * @brief Init USB device Library, add supported class and start the library
  * @retval None
  */
void MX_USB_DEVICE_Init(void)
{
  /* USER CODE BEGIN USB_DEVICE_Init_PreTreatment */

  /* USER CODE END USB_DEVICE_Init_PreTreatment */

  /* Init Device Library, add supported class and start the library. */
  if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK)
  {
    Error_Handler();
  }
  if (USBD_Start(&hUsbDeviceFS) != USBD_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN USB_DEVICE_Init_PostTreatment */

  /* USER CODE END USB_DEVICE_Init_PostTreatment */
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
