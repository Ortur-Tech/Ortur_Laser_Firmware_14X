#include "grbl.h"

//#include "timer.h"

#ifndef DEFAULT_ACCEL_SENSITIVITY 350
	#define DEFAULT_ACCEL_SENSITIVITY 350
#endif

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

//TODO:检测10秒内的加速度值

//初始化并读取加速度计数据
void accel_detection()
{
	//延时读取加速度情况
	bma2x2_data_readout(&accel_x,&accel_y,&accel_z);

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

			debugStr("shake_detected");

			//进入警告状态,终止雕刻
			if(sys.state == STATE_CYCLE)
			{
				printString("Detect movement, terminate laser engraving!\r\n");
				{
					char diffStr[50];
					sprintf(diffStr,"Acceleration fluctuation:%d\r\n",(int)(accel_diff));
					printString(diffStr);
				}
				printString("Reset Grbl.\r\n");
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
	return;

	//雕刻运行时才检测
	if(sys.state == STATE_CYCLE)
	{
		//定时检查,避免速度太频繁
		if(last_accel_check_ms + accel_check_interval_ms <= sys_millis())
		{
			accel_detection();
			last_accel_check_ms = sys_millis();
		}
	}
	else
	{
		detection_count = 0;
	}
}
