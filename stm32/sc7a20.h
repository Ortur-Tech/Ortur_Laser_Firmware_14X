
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SC7A20_H
#define __SC7A20_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "iic.h"
#include "main.h"

#define SC7A20_ADDR		0x30 //0x30

 typedef struct {
 	uint16_t xValue;
 	uint16_t yValue;
 	uint16_t zValue;
 }SC7A20_TYPE;

 extern SC7A20_TYPE scInfo;
 void Sc7a20_Init(void);
 uint8_t Read_Sc7a20_data(void);

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
