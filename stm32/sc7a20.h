
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SC7A20_H
#define __SC7A20_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "iic.h"
#include "main.h"

#define SC7A20_ADDR	  0x30 //0x30
#define SC7A20_DEVICE 0X11
#define OTHER_DEVICE  0X03

 void Gsensor_Init(void);//!< 传感器初始化
 void Sc7a20_Init(void);
 char Read_One_Byte(char device_addr, char reg_addr);//!<从传感器初读一个字节
 void Write_One_Byte_iicaddr(char iicchar, char addr, char thedata);//!<写一个字节到传感器
 uint8_t Get_GsensorType(void);
 void Sc7a20_Get_Acceleration(short *gx, short *gy, short *gz);
#ifdef __cplusplus
}
#endif
#endif /*__SC7A20_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
