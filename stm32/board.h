#ifndef _BOARD_H_
#define _BOARD_H_

#include "main.h"
#include "stdio.h"




#define DEBUG_LEVEL 0//LOG_INFO//LOG_ERROR
/*调试等级*/
typedef enum{
    LOG_DEBUG=1,
    LOG_INFO=2,
    LOG_WARN=4,
    LOG_ERROR=8,
    LOG_FATAL=16,
	LOG_IMP_INFO=32,
}LogLevel;


#define mprintf(level,format,...)  ((level&DEBUG_LEVEL)?printf(format,##__VA_ARGS__):0)


void soft_delay_us(uint32_t nus);

void soft_delay_ms(uint32_t nms);
void Check_Rst_Source(void);
#endif



















