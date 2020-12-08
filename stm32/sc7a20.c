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

	IIC_Init();
	soft_delay_ms(20);
	Write_One_Byte_iicaddr(SC7A20_ADDR, 0x20, 0x47);
	Write_One_Byte_iicaddr(SC7A20_ADDR, 0x23, 0x88);

	soft_delay_ms(10);
}

#define SC7A20_DEVICE 0X11
#define OTHER_DEVICE 0X03

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

SC7A20_TYPE scInfo;

uint8_t Read_Sc7a20_data(void)
{
	char temp;
	char x_l8,x_h8,y_l8,y_h8,z_l8,z_h8;
	/*可以用来兼容BM250*/
	if(Get_GsensorType()==SC7A20_DEVICE)
	{
		temp = Read_One_Byte(SC7A20_ADDR, 0x27);
		temp = temp & 0x0f;
		if (0x0f == temp)
		{
			x_l8 = Read_One_Byte(SC7A20_ADDR, 0x28);
			x_h8 = Read_One_Byte(SC7A20_ADDR, 0x29);
			y_l8 = Read_One_Byte(SC7A20_ADDR, 0x2A);
			y_h8 = Read_One_Byte(SC7A20_ADDR, 0x2B);
			z_l8 = Read_One_Byte(SC7A20_ADDR, 0x2C);
			z_h8 = Read_One_Byte(SC7A20_ADDR, 0x2D);

			scInfo.xValue=(x_h8<<8)+x_l8;
			scInfo.yValue=(y_h8<<8)+y_l8;
			scInfo.zValue=(z_h8<<8)+z_l8;

			mprintf(LOG_INFO,"xValue:%d. yValue:%d. zValue:%d.\r\n",scInfo.xValue,scInfo.yValue,scInfo.zValue);
			return 0;
		}
	}
	else
	{
		mprintf(LOG_INFO,"NOT SC7A20 DEVICE.\r\n");
		return 1;
	}

}






