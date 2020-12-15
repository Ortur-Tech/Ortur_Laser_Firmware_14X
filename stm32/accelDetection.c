#include "grbl.h"
#include "iic.h"
#include "sc7a20.h"
//#include "timer.h"

#ifndef DEFAULT_ACCEL_SENSITIVITY
	#define DEFAULT_ACCEL_SENSITIVITY 350
#endif
uint8_t BMA250_ReadByte(uint8_t I2C_addr, uint8_t Reg_addr);
extern int32_t bma2x2_data_readout(int16_t * accel_x_int16_t,int16_t * accel_y_int16_t,int16_t * accel_z_int16_t);
#define _ABS(x) (x<0?-x:x)

int16_t accel_x,accel_y, accel_z;
float accel_x_g,accel_y_g, accel_z_g;
int16_t accel_x_old,accel_y_old,accel_z_old;
int16_t accel_x_diff,accel_y_diff,accel_z_diff,accel_diff;
//uint16_t max_mutation_value = 300;
uint8_t accel_reverse = 0; //是否有加速度反向
uint8_t accel_mutation = 0; //是否有加速度突变
uint8_t shake_detected = 0; //是否检测到震动
uint32_t accel_check_interval_ms = 100; //250毫秒检测一次
uint64_t last_accel_check_ms = 0;
uint32_t detection_count = 0; //开始检测

uint8_t BMA250_WriteByte(uint8_t I2C_addr,uint8_t Reg_addr,uint8_t data);
uint8_t Check_BMA250_ID(void);
void BMA250_Get_Acceleration(short *gx, short *gy, short *gz);

#define BMA250_Addr          (0x18<<1)    //SD0 Low  0x18;    SD0 High 0x19;

#define BMP_CHIPID           0x00
#define BMP_ACC_X_LSB        0x02
#define BMP_ACC_X_MSB        0x03
#define BMP_ACC_Y_LSB        0x04
#define BMP_ACC_Y_MSB        0x05
#define BMP_ACC_Z_LSB        0x06
#define BMP_ACC_Z_MSB        0x07

#define BMP_GRANGE           0x0F	   // g Range
#define BMP_BWD              0x10	   // Bandwidth
#define BMP_PM               0x11	   // Power modes
#define BMP_SCR              0x13	   // Special Control Register
#define BMP_RESET            0x14	   // Soft reset register writing 0xB6 causes reset
#define BMP_ISR1             0x16	   // Interrupt settings register 1
#define BMP_ISR2             0x17	   // Interrupt settings register 2
#define BMP_IMR1             0x19	   // Interrupt mapping register 1
#define BMP_IMR2             0x1A	   // Interrupt mapping register 2
#define BMP_IMR3             0x1B	   // Interrupt mapping register 3

#define BMP_AS_RANGE 2
#define BMP_AS_BANDWIDTH 8
#define BMP_AS_SLEEPPHASE 2

uint8_t GsensorDeviceType=0;


void BMA250_Init(void)
{
    uint8_t bGRange = 0;                                  // g Range;
    uint8_t bBwd = 0;                                     // Bandwidth
    uint8_t bSleep = 0;                                   // Sleep phase

// Configure sensor and start to sample data
#if (BMP_AS_RANGE == 2)
	bGRange = 0x03;
#elif (BMP_AS_RANGE == 4)
	bGRange = 0x05;
#elif (BMP_AS_RANGE == 8)
	bGRange = 0x08;
#elif (BMP_AS_RANGE == 16)
	bGRange = 0x0C;
#else
//	#error "Measurement range not supported"
#endif

#if (BMP_AS_BANDWIDTH == 8)
	bBwd = 0x08;
#elif (BMP_AS_BANDWIDTH == 16)
	bBwd = 0x09;
#elif (BMP_AS_BANDWIDTH == 31)
	bBwd = 0x0A;
#elif (BMP_AS_BANDWIDTH == 63)
	bBwd = 0x0B;
#elif (BMP_AS_BANDWIDTH == 125)
	bBwd = 0x0C;
#elif (BMP_AS_BANDWIDTH == 250)
	bBwd = 0x0D;
#elif (BMP_AS_BANDWIDTH == 500)
	bBwd = 0x0E;
#elif (BMP_AS_BANDWIDTH == 1000)
	bBwd = 0x0F;
#else
//	#error "Sample rate not supported"
#endif

#if (BMP_AS_SLEEPPHASE == 1)
	bSleep = 0x4C;
#elif (BMP_AS_SLEEPPHASE == 2)
	bSleep = 0x4E;
#elif (BMP_AS_SLEEPPHASE == 4)
	bSleep = 0x50;
#elif (BMP_AS_SLEEPPHASE == 6)
	bSleep = 0x52;
#elif (BMP_AS_SLEEPPHASE == 10)
	bSleep = 0x54;
#elif (BMP_AS_SLEEPPHASE == 25)
	bSleep = 0x56;
#elif (BMP_AS_SLEEPPHASE == 50)
	bSleep = 0x58;
#else
//	#error "Sleep phase duration not supported"
#endif


	// write sensor configuration
	BMA250_WriteByte(BMA250_Addr, BMP_GRANGE, bGRange);  // Set measurement range
	BMA250_WriteByte(BMA250_Addr, BMP_BWD, bBwd);        // Set filter bandwidth
	BMA250_WriteByte(BMA250_Addr, BMP_PM, bSleep);       // Set filter bandwidth


#ifndef BMP_AS_FILTERING
	BMA250_WriteByte(BMA250_Addr, BMP_SCR, 0x80);        // acquire unfiltered acceleration data
#endif

//	while(1)
//	{
////		BMA250_WriteByte(BMA250_Addr, BMP_BWD, 0x09);        // Set filter bandwidth
////		bSleep=BMA250_ReadByte(BMA250_Addr,BMP_BWD);
////		mprintf(LOG_INFO,"BMP_BWD w/r:0x%02x:0x%02x.\r\n",0x09,bSleep);
////		BMA250_WriteByte(BMA250_Addr, BMP_BWD, bBwd);        // Set filter bandwidth
////		bSleep=BMA250_ReadByte(BMA250_Addr,BMP_BWD);
////		mprintf(LOG_INFO,"BMP_BWD w/r:0x%02x:0x%02x.\r\n",bBwd,bSleep);
//
//		bSleep=Check_BMA250_ID();
//		mprintf(LOG_INFO,"chip id:0x%02x.\r\n",bSleep);
//		BMA250_Get_Acceleration(&accel_x,&accel_y,&accel_z);
//	    bBwd=bSleep;
//	    mprintf(LOG_INFO,"xValue:%d. yValue:%d. zValue:%d.\r\n",accel_x,accel_y,accel_z);
//	    delay_ms(1000);
//	}

}
void Gsensor_Init(void)
{
	if(GsensorDeviceType==0)
	{
		GsensorDeviceType=Get_GsensorType();
	}
	if(GsensorDeviceType==SC7A20_DEVICE)
	{
		Sc7a20_Init();
	}
	else
	{
		BMA250_Init();
	}
}
/**
* BMA250 IIC写一个字节
* I2C_addr  BMA250地址
* Reg_addr:寄存器地址
* data: 写入寄存器地址
* 返回值:0,正常
*        1,错误代码
*/
uint8_t BMA250_WriteByte(uint8_t I2C_addr,uint8_t Reg_addr,uint8_t data)
{
	IIC_Start();
    IIC_Send_Byte(I2C_addr | 0X00); //WRITE i2c
    if (IIC_Wait_Ack()==1) {
	goto err;
    }

    IIC_Send_Byte(Reg_addr);
    if (IIC_Wait_Ack()==1) {
	goto err;
    }

    IIC_Send_Byte(data);
    if (IIC_Wait_Ack()==1) {
        goto err;
    }
    IIC_Stop();
    return 1;
err:
	IIC_Stop();
	return 0;
}

/**
* BMA250 IIC读一个字节
* I2C_addr  BMA250地址
* Reg_addr:寄存器地址
* 返回值:data,正常
*        0,错误代码
*/
uint8_t BMA250_ReadByte(uint8_t I2C_addr, uint8_t Reg_addr)
{
    uint8_t data;
    IIC_Start();
    IIC_Send_Byte(I2C_addr | 0X00); //WRITE i2c
    if (IIC_Wait_Ack()==1) {
	goto err;
    }

    IIC_Send_Byte(Reg_addr);
    if (IIC_Wait_Ack()==1) {
	goto err;
    }
    //IIC_Stop();

    IIC_Start();
    IIC_Send_Byte(I2C_addr | 0X01); //READ
    if (IIC_Wait_Ack()==1) {
	goto err;
    }

    data = IIC_Read_Byte(0);
    //send_noack();
    IIC_Stop();
    return data;

err:
	IIC_Stop();
	return 0;
}

//读1个字节，ack=1时，发送ACK，ack=0，发送nACK
uint8_t BMA250_IIC_Read_Byte(uint8_t ack)
{
    uint8_t data;
    data = IIC_Read_Byte(ack);
//    if(ack) IIC_Ack();
//    else IIC_NAck();
    return data;
}


/**
* BMA250 IIC连续读len个字节
* I2C_addr:  BMA250地址
* Reg_addr:寄存器地址
* len:要读取的长度
* buff:读取到的数据存储区
* 返回值:0,正常
*        其他,错误代码
*/
uint8_t BMA250_Read_LenBytes(uint8_t I2C_addr, uint8_t Reg_addr, uint8_t len, uint8_t *buff)
{
	IIC_Start();
	IIC_Send_Byte(I2C_addr | 0X00); //WRITE i2c
    if (IIC_Wait_Ack()==1) {
	goto err;
    }

    IIC_Send_Byte(Reg_addr);
    if (IIC_Wait_Ack()==1) {
	goto err;
    }
    IIC_Stop();

    IIC_Start();
    IIC_Send_Byte(I2C_addr | 0X01); //READ
    if (IIC_Wait_Ack()==1) {
	goto err;
    }
    while(len)
    {
            if(len==1) *buff=BMA250_IIC_Read_Byte(0);   //读数据,发送nACK
            else *buff=BMA250_IIC_Read_Byte(1);		//读数据,发送ACK
            len--;
            buff++;
    }
    IIC_Stop();
    return 0;
err:
	IIC_Stop();
	return 1;
}

uint8_t Check_BMA250_ID(void)
{
    return (BMA250_ReadByte(BMA250_Addr, 0x00));
}

//得到加速度值(原始值)
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
void BMA250_Get_Acceleration(short *gx, short *gy, short *gz)
{
	uint16_t x_l8,x_h8,y_l8,y_h8,z_l8,z_h8;

    x_l8 = BMA250_ReadByte(BMA250_Addr, BMP_ACC_X_LSB);
    x_h8 = BMA250_ReadByte(BMA250_Addr, BMP_ACC_X_MSB);
    y_l8 = BMA250_ReadByte(BMA250_Addr, BMP_ACC_Y_LSB);
    y_h8 = BMA250_ReadByte(BMA250_Addr, BMP_ACC_Y_MSB);
    z_l8 = BMA250_ReadByte(BMA250_Addr, BMP_ACC_Z_LSB);
    z_h8 = BMA250_ReadByte(BMA250_Addr, BMP_ACC_Z_MSB);

    *gx=(short)((x_h8<<8)|x_l8);
    *gy=(short)((y_h8<<8)|y_l8);
    *gz=(short)((z_h8<<8)|z_l8);

    *gx=(*gx)>>6;
    *gy=(*gy)>>6;
    *gz=(*gz)>>6;

}

//TODO:检测10秒内的加速度值

//初始化并读取加速度计数据
void accel_detection()
{
	if(GsensorDeviceType==SC7A20_DEVICE)
	{
		Sc7a20_Get_Acceleration(&accel_x,&accel_y,&accel_z);
	}
	else
	{
		//延时读取加速度情况
		BMA250_Get_Acceleration(&accel_x,&accel_y,&accel_z);
	}

	if(accel_x != accel_x_old ||accel_y != accel_y_old ||accel_z != accel_z_old )
	{
		//仅在雕刻模式下起作用
		//如果数据突变,就表明有较大的震动或位移,这样雕刻会错位,无法继续

		//至少积累5次参考值
		if(detection_count > 2)
		{
			//计算加速度变化
			accel_x_diff = accel_x - accel_x_old;
			accel_y_diff = accel_y - accel_y_old;
			accel_z_diff = accel_z - accel_z_old;

			//雕刻时,不允许加速度突变
			accel_diff = _ABS(accel_x_diff) + _ABS(accel_y_diff) + _ABS(accel_z_diff);
			if( accel_diff > settings.accel_sensitivity )
			{
				shake_detected = 1;
			}
		}


		if(shake_detected)
		{
			shake_detected = 0;

			//debugStr("shake_detected");

			//进入警告状态,终止雕刻
			if(sys.state == STATE_CYCLE)
			{
				//printString("Detect movement, terminate laser engraving!\r\n");
				{
					char diffStr[50];
					//sprintf(diffStr,"Acceleration fluctuation:%d\r\n",(int)(accel_diff));
					//printString(diffStr);
				}
				//printString("Reset Grbl.\r\n");
				sys.state = STATE_ALARM;
				sys.abort = 1;
			}
		}

//		uint8_t buf[100]={0};
//		sprintf(buf,"accel_xyz:[%d",(int)(accel_x)); //_g*10000.0f
//		debugStr(buf);
//		sprintf(buf,", %d ,",(int)(accel_y)); //_g*10000.0f
//		debugStr(buf);
//		sprintf(buf,"%d].\r\n",(int)(accel_z)); //_g*10000.0f
//		debugStr(buf);

		//计算均值
		accel_x_old = (accel_x_old + accel_x) / 2;
		accel_y_old = (accel_y_old + accel_y) / 2;
		accel_z_old = (accel_z_old + accel_z) / 2;
	}

	++detection_count;//统计检测次数
}

//限制加速度检测的频率
void accel_detection_limit()
{
	//NOTE: 测试,屏蔽掉加速度传感器
	//return;

	//雕刻运行时才检测
	if(sys.state == STATE_CYCLE)
	{
		//定时检查,避免速度太频繁
		if(last_accel_check_ms + accel_check_interval_ms <= HAL_GetTick())
		{
			accel_detection();
			last_accel_check_ms = HAL_GetTick();
		}
	}
	else
	{
		detection_count = 0;
	}
}
