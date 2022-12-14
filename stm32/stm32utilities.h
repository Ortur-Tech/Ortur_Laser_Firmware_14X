/*
  stm32utilities.h - Header file for shared definitions, variables, and functions
  Part of Grbl32

  Copyright (c) 2018-2019 Thomas Truong

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef STM32UTILITIES_H_
#define STM32UTILITIES_H_

#ifdef __cplusplus
{
#endif


#ifdef STM32F1
//	#include "stm32f1xx_hal.h"
#endif
#ifdef STM32F4
//	#include "stm32f4xx_hal.h"
	#include "spi.h"

#endif

#include "main.h"
#include "g32core.h"
#include "grbl.h"
#include "nuts_bolts.h"

extern const PIN_MASK step_pin_mask[N_AXIS];
extern const PIN_MASK direction_pin_mask[N_AXIS];
extern const PIN_MASK limit_pin_mask[N_AXIS];
extern uint32_t uTICKS_PER_MICROSECOND;
extern float fTICKS_PER_MINUTE;

#ifdef STM32F13

void Spindle_Disable();
void Spindle_Enable();
#endif

extern uint8_t report_power_flag;
uint8_t IsMainPowrIn(void);
void Main_PowerCheck(void);
void timing_init();

//-- PIN IO, to replace legacy SPL calls to LL and HAL
//-- Port based calls
#define GPIO_ReadInputData 		LL_GPIO_ReadInputPort
#define GPIO_ReadOutputData		LL_GPIO_ReadOutputPort
#define GPIO_Write 						LL_GPIO_WriteOutputPort
//-- Pin based calls, need to use HAL since LL pins and HAL pins are incompatible for F1
void GPIO_ResetBits (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void GPIO_SetBits	(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

//-- UART
void uart_init();
void uart_sendstr(const char *pStr);
void uart_sendch(uint8_t ch);

#ifdef STM32F46   //-- board specific hardwares
	enum IOExpChip {IOC0, IOC1};
	enum IOActive {IOA_Lo, IOA_Hi};

  //-- MCP23S17 Expander Registers
  #define SPI_IOCONA		0x0A  	//Configuration
	#define SPI_IOCONB		0x0B  	//Configuration

  #define SPI_IODIRA 		0x00 		//I/O DIRECTION REGISTER (ADDR 0x00)	: Page 18 : bit=1:input, bit=0:output
  #define SPI_IODIRB	 	0x01 		//
  #define SPI_IOPOLA   	0x02   // IO polarity   (0 = normal, 1 = inverse)
  #define SPI_IOPOLB   	0x03

  #define SPI_DEFVALA  	0x06   // Default comparison for interrupt on change (interrupts on opposite)
  #define SPI_DEFVALB  	0x07
  #define SPI_INTCONA  	0x08   // Interrupt control (0 = interrupt on change from previous, 1 = interrupt on change from DEFVAL)
  #define SPI_INTCONB  	0x09

  #define SPI_GPIOA   	0x12  //GENERAL PURPOSE I/O PORT REGISTER
  #define SPI_GPIOB   	0x13  //
  #define SPI_GPPUA   	0x0C  //-- Pull up resistor
  #define SPI_GPPUB   	0x0D
  #define SPI_GPINTENA 	0x04 //-- enable interupt
  #define SPI_GPINTENB 	0x05 //-- enable interupt
  #define SPI_INFTFA   	0x0E   // Interrupt flag (read only) : (0 = no interrupt, 1 = pin caused interrupt)
  #define SPI_INFTFB   	0x0F
  #define SPI_INTCAPA  	0x10   // Interrupt capture (read only) : value of GPIO at time of last interrupt
  #define SPI_INTCAPB  	0x11

	//-- low level
  void SPIInit(SPI_HandleTypeDef *HSPI);
  uint8_t SPIRead (enum IOExpChip IOC, uint8_t Reg);
  HAL_StatusTypeDef SPIWrite(enum IOExpChip IOC, uint8_t Reg, uint8_t Val);

  //-- high level
  uint16_t GetLimitsState();
  void SetLimitActive (enum IOActive eA);	//-- True: Sensor = Hi when active/triggered, rising edge trigger
  void SetHomeActive (enum IOActive eA);	//-- True: Sensor = Hi when active/triggered, rising edge trigger

  void DebugRead(enum IOExpChip IOC, uint8_t Reg);
  uint8_t ReadInputByte();

  void EnableLimitsINT();		//-- enable limits interrupt
  void DisableLimitsINT();	//-- disable

#endif

#ifdef STM32F46 //-- board specific hardware, SPI driven limits
void spi_limits_init();
#endif


#ifdef __cplusplus
}
#endif

#endif /* STM32UTILITIES_H_ */
