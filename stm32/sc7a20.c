/*
 * sc7a20.c
 *
 *  Created on: 2020年11月3日
 *      Author: c
 */


#include "sc7a20.h"
#include "main.h"
#include "board.h"

void Write_One_Byte_iicaddr(char iicchar, char addr, char thedata)
{
	uint8_t acktemp = 1;
	/*write a byte to mem*/
	IIC_Start();
	IIC_Send_Byte(iicchar);
	acktemp = IIC_Wait_Ack();
	IIC_Send_Byte(addr);/*address*/
	acktemp = IIC_Wait_Ack();
	IIC_Send_Byte(thedata);/*thedata*/
	acktemp = IIC_Wait_Ack();
	IIC_Stop();
}


char Read_One_Byte(char device_addr, char reg_addr)
{
	uint8_t acktemp = 1;
	char mydata;
	 /*read a byte from mem*/
	IIC_Start();
	IIC_Send_Byte(device_addr);
	acktemp = IIC_Wait_Ack();
	IIC_Send_Byte(reg_addr);/*address*/
	acktemp = IIC_Wait_Ack();
	IIC_Start();
	IIC_Send_Byte(device_addr + 0x01);
	acktemp = IIC_Wait_Ack();

	mydata  = IIC_Read_Byte(0);

	IIC_Stop();
	return mydata;

}

void Sc7a20_Init(void)
{

	//IIC_Init();
	//soft_delay_ms(20);
	Write_One_Byte_iicaddr(SC7A20_ADDR, 0x20, 0x47);
	Write_One_Byte_iicaddr(SC7A20_ADDR, 0x23, 0x88);

}


uint8_t Get_GsensorType(void)
{
	if(SC7A20_DEVICE== Read_One_Byte(SC7A20_ADDR, 0xf))
	{
		return SC7A20_DEVICE;
	}
	else
	{
		return OTHER_DEVICE;
	}
}


//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
void Sc7a20_Get_Acceleration(short *gx, short *gy, short *gz)
{
	uint16_t x_l8,x_h8,y_l8,y_h8,z_l8,z_h8;

	x_l8 = Read_One_Byte(SC7A20_ADDR, 0x28);
	x_h8 = Read_One_Byte(SC7A20_ADDR, 0x29);
	y_l8 = Read_One_Byte(SC7A20_ADDR, 0x2A);
	y_h8 = Read_One_Byte(SC7A20_ADDR, 0x2B);
	z_l8 = Read_One_Byte(SC7A20_ADDR, 0x2C);
	z_h8 = Read_One_Byte(SC7A20_ADDR, 0x2D);

    *gx=(short)((x_h8<<8)|x_l8);
    *gy=(short)((y_h8<<8)|y_l8);
    *gz=(short)((z_h8<<8)|z_l8);

    *gx=(*gx)>>6;
    *gy=(*gy)>>6;
    *gz=(*gz)>>6;

	//mprintf(LOG_INFO,"xValue:%d. yValue:%d. zValue:%d.\r\n",*gx,*gy,*gz);
	return ;
}







