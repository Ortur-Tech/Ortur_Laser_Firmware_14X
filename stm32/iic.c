/*
 * iic.c
 *
 *  Created on: 2019年11月11日
 *      Author: MSI-PC
 */

#include "iic.h"
#include "main.h"
#include "board.h"

#define BITBAND(addr,bitnum)   ((addr&0xF0000000)+0x2000000+((addr&0xFFFFF)<<5)+(bitnum<<2))
#define MEM_ADDR(addr)          *((volatile unsigned long*)(addr)) //强制转换为指针
#define BIT_ADDR(addr,bitnum)   MEM_ADDR(BITBAND(addr,bitnum)) //再来一个宏重命名一下

#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C 宏定义地址，输出寄存器。
#define GPIOC_ODR_Addr    (GPIOC_BASE+12) //0x4001100C 宏定义地址，输出寄存器。
#define GPIOD_ODR_Addr    (GPIOD_BASE+12) //0x4001140C
#define GPIOE_ODR_Addr    (GPIOE_BASE+12) //0x4001180C
#define GPIOF_ODR_Addr    (GPIOF_BASE+12) //0x40011A0C,
#define GPIOG_ODR_Addr    (GPIOG_BASE+12) //0x40011E0C宏定义地址，输出寄存器。


#define GPIOA_IDR_Addr    (GPIOA_BASE+8) //0x40010808 GPIO->IDR是32位的输入数据寄存器
#define GPIOB_IDR_Addr    (GPIOB_BASE+8) //0x40010C08 宏定义地址，输入寄存器。
#define GPIOC_IDR_Addr    (GPIOC_BASE+8) //0x40011008
#define GPIOD_IDR_Addr    (GPIOD_BASE+8) //0x40011408
#define GPIOE_IDR_Addr    (GPIOE_BASE+8) //0x40011808
#define GPIOF_IDR_Addr    (GPIOF_BASE+8) //0x40011A08
#define GPIOG_IDR_Addr    (GPIOG_BASE+8) //0x40011E08 宏定义地址，输入寄存器。

//IO口操作,只对单一的IO口!
//确保n的值小于16!
#define PAout(n)   BIT_ADDR(GPIOA_ODR_Addr,n)//输出 典型的位操作定义
#define PAin(n)    BIT_ADDR(GPIOA_IDR_Addr,n)//输入


#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)//输出
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)//输入


#define PCout(n)   BIT_ADDR(GPIOC_ODR_Addr,n)//输出
#define PCin(n)    BIT_ADDR(GPIOC_IDR_Addr,n)//输入


#define PDout(n)   BIT_ADDR(GPIOD_ODR_Addr,n)//输出
#define PDin(n)    BIT_ADDR(GPIOD_IDR_Addr,n)//输入


#define PEout(n)   BIT_ADDR(GPIOE_ODR_Addr,n)//输出
#define PEin(n)    BIT_ADDR(GPIOE_IDR_Addr,n)//输入


#define PFout(n)   BIT_ADDR(GPIOF_ODR_Addr,n)//输出
#define PFin(n)    BIT_ADDR(GPIOF_IDR_Addr,n)//输入


#define PGout(n)   BIT_ADDR(GPIOG_ODR_Addr,n)//输出
#define PGin(n)    BIT_ADDR(GPIOG_IDR_Addr,n)//输入


//IO方向设置
#define SDA_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=(uint32_t)(8<<12);}
#define SDA_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=(uint32_t)(3<<12);}
//IO操作函数
#define IIC_SCL    PBout(10) //IIC_SCL
#define IIC_SDA    PBout(11) //IIC_SDA
#define READ_SDA   PBin(11)  //输入SDA

#define TRUE 1
#define FALSE 0

//#define soft_delay_us(...)
/**
 * @brief 初始化IIC
 */
void IIC_Init(void)
{
	LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOB);
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_GPIOA);
	/**I2C2 GPIO Configuration
	PB10     ------> I2C2_SCL
	PB11     ------> I2C2_SDA
	*/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_10|LL_GPIO_PIN_11;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    /*在这里初始化USB DP 以降低代码量*/
    GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	IIC_SCL=1;
	IIC_SDA=1;

}

/**
 * @brief 产生IIC起始信号
 */
void IIC_Start(void)
{
	SDA_OUT();  //sda线输出
	IIC_SDA=1;
	IIC_SCL=1;
	soft_delay_us(4);
 	IIC_SDA=0;  //START:when CLK is high,DATA change form high to low
	soft_delay_us(4);
	IIC_SCL=0;  //钳住I2C总线，准备发送或接收数据
	soft_delay_us(1);
}

/**
 * @brief IIC_Stop产生IIC停止信号
 */
void IIC_Stop(void)
{
	SDA_OUT();  //sda线输出
	IIC_SCL=0;
	IIC_SDA=0;  //STOP:when CLK is high DATA change form low to high
 	soft_delay_us(4);
	IIC_SCL=1;
	soft_delay_us(4);
	IIC_SDA=1;  //发送I2C总线结束信号
	soft_delay_us(4);
}

/**
 * @brief IIC_Wait_Ack 发送数据后，等待应答信号到来
 * @return 1，接收应答失败，IIC直接退出
 *         0，接收应答成功，什么都不做
 */
uint8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	SDA_IN();      //SDA设置为输入
	IIC_SDA=1;soft_delay_us(1);
	IIC_SCL=1;soft_delay_us(1);
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;  //时钟输出0
	soft_delay_us(1);
	return 0;
}

/**
 * @brief IIC_Ack产生ACK应答
 */
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	soft_delay_us(2);
	IIC_SCL=1;
	soft_delay_us(2);
	IIC_SCL=0;
	soft_delay_us(1);
}

/**
 * @brief IIC_NAck 不产生ACK应答
 */
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	soft_delay_us(2);
	IIC_SCL=1;
	soft_delay_us(2);
	IIC_SCL=0;
	soft_delay_us(1);
}

/**
 * @brief IIC_Send_ByteIIC发送一个字节
 * @param txd 1，有应答
 *            0，无应答
 */
void IIC_Send_Byte(uint8_t txd)
{
    uint8_t t;
	SDA_OUT();
	IIC_SCL=0; //拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {
    	soft_delay_us(2);
        //IIC_SDA=(txd&0x80)>>7;
		if((txd&0x80)>>7)
			IIC_SDA=1;
		else
			IIC_SDA=0;
		txd<<=1;
		soft_delay_us(2);	//对TEA5767这三个延时都是必须的
		IIC_SCL=1;
		soft_delay_us(2);
		IIC_SCL=0;

    }
    SDA_IN();
}

/**
 * @brief IIC_Read_Byte
 * @param ack 读1个字节，ack=1时，发送ACK，ack=0，发送nACK
 * @return 返回读到的数据
 */
uint8_t IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	IIC_SDA=1;
	SDA_IN();	//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0;
        soft_delay_us(2);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;
		soft_delay_us(1);
    }
    if (!ack)
        IIC_NAck();	//发送nACK
    else
        IIC_Ack();	//发送ACK
    return receive;
}
