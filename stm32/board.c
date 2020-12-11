/*
 * board.c
 *
 *  Created on: 2020年11月3日
 *      Author: c
 */


#include "board.h"


#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
    while ((USART1->SR & 0X40) == 0);
    USART1->DR = (uint8_t) ch;
    return ch;
}

/*当单片机主频为72兆赫兹时比较精准的软件延时*/
void soft_delay_us(uint32_t nus)
{
	 while (nus --)
	  {
	    __NOP();__NOP();__NOP();
	    __NOP();__NOP();__NOP();
	    __NOP();__NOP();__NOP();
	    __NOP();__NOP();__NOP();
	    __NOP();__NOP();__NOP();
	    __NOP();__NOP();__NOP();
	    __NOP();__NOP();__NOP();
	    __NOP();__NOP();__NOP();
	    __NOP();__NOP();__NOP();
	    __NOP();__NOP();__NOP();
	    __NOP();__NOP();__NOP();
	    __NOP();__NOP();__NOP();
	    __NOP();__NOP();__NOP();
	    __NOP();__NOP();__NOP();
	    __NOP();__NOP();__NOP();
	    __NOP();__NOP();__NOP();
	    __NOP();__NOP();__NOP();
	    __NOP();__NOP();__NOP();
	    __NOP();__NOP();__NOP();
	    __NOP();__NOP();__NOP();
	    __NOP();__NOP();__NOP();
	    __NOP();__NOP();__NOP();
	  }

}

void soft_delay_ms(uint32_t nms)
{
	while(nms--)
	{
		soft_delay_us(1000);
	}
}

void Check_Rst_Source(void)
{

	  if(LL_RCC_IsActiveFlag_IWDGRST())
	  {
	  }
	  if(LL_RCC_IsActiveFlag_LPWRRST())
	  {
	  }

	  if(LL_RCC_IsActiveFlag_PINRST())
      {
	  }

	  if(LL_RCC_IsActiveFlag_PORRST())
      {
	  }

	  if(LL_RCC_IsActiveFlag_SFTRST())
      {
	  }



	  if(LL_RCC_IsActiveFlag_WWDGRST())
      {

	  }

	  LL_RCC_ClearResetFlags();

}
