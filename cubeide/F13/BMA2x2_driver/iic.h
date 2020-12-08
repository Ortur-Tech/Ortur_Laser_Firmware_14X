/*
 * iic.h
 *
 *  Created on: 2019年11月11日
 *      Author: MSI-PC
 */

#ifndef BMA2X2_DRIVER_IIC_H_
#define BMA2X2_DRIVER_IIC_H_

#include "board.h"

//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(unsigned char txd);			//IIC发送一个字节
unsigned char IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
unsigned char IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号


#endif /* BMA2X2_DRIVER_IIC_H_ */
