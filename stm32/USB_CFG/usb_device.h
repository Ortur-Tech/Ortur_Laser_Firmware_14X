/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usb_device.h
  * @version        : v2.0_Cube
  * @brief          : Header for usb_device.c file.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_DEVICE__H__
#define __USB_DEVICE__H__

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "usbd_def.h"

extern uint8_t usbPlugIn;
extern uint8_t usbCdcConnectFlag;


/** USB Device initialization function. */
void MX_USB_DEVICE_Init(void);
void UsbCDCDisconnectStopEngrave(void);
/*
 * -- Insert functions declaration here --
 */
/* USER CODE BEGIN FD */
/**
 * @brief 强制复位USB
 */
static inline void Reset_Usb()
{
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET);
	HAL_Delay(250);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET);
	HAL_Delay(250);
}

/**
 * @brief isUsbCDCConnected
 * @return 1：已连接 0：未连接
 */
uint8_t isUsbCDCConnected(void);

/**
 * @brief setUsbCDCConnected
 * @param status
 */
inline void setUsbCDCConnected(uint8_t status)
{
	usbCdcConnectFlag = status;
}

inline uint8_t isUsbPlugIn(void)
{
	return usbPlugIn;
}

// Is usb cable plug in
void setUsbPlugIn(uint8_t value);

/* USER CODE END FD */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __USB_DEVICE__H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
