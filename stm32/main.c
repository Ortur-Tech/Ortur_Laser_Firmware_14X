/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  * Copyright (c) 2018-2019 Thomas Truong
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

#include "grbl.h"
#include "sc7a20.h"

system_t sys;
int32_t sys_position[N_AXIS];      // Real-time machine (aka home) position vector in steps.
int32_t sys_probe_position[N_AXIS]; // Last probe position in machine coordinates and steps.
volatile uint8_t sys_probe_state;   // Probing state value.  Used to coordinate the probing cycle with stepper ISR.
volatile uint8_t sys_rt_exec_state;   // Global realtime executor bitflag variable for state management. See EXEC bitmasks.
volatile uint8_t sys_rt_exec_alarm;   // Global realtime executor bitflag variable for setting various alarms.
volatile uint8_t sys_rt_exec_motion_override; // Global realtime executor bitflag variable for motion-based overrides.
volatile uint8_t sys_rt_exec_accessory_override; // Global realtime executor bitflag variable for spindle/coolant overrides.
#ifdef DEBUG
  volatile uint8_t sys_rt_exec_debug;
#endif

void SystemClock_Config(void);
static void MX_NVIC_Init(void);

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

#ifdef USE_BACKTRACE
	{ //Enable fault by div 0
    volatile int * SCB_CCR = (volatile int *) 0xE000ED14; // SCB->CCR
    *SCB_CCR |= (1 << 4); /* bit4: DIV_0_TRP. */
	}
	/* CmBacktrace initialize */
	cm_backtrace_init("CmBacktrace", ORTUR_MODEL_NAME , "Grbl " GRBL_VERSION " - OLF " ORTUR_VERSION);
#endif

  /* ??????????????????*/
  HAL_Init();
  /*?????? ???????????????BootLoader?????????????????????????????????*/
  __enable_irq();
  /*?????????????????????*/
  SystemClock_Config();
#if USE_DOUBLE_SERIAL
  /*???????????????1?????????115200*/
  MX_USART1_UART_Init();
#endif

  /*????????????????????????GPIO*/
  MX_GPIO_Init();
  /*IIC?????????*/
  IIC_Init();
  /*???????????????????????????*/
  Gsensor_Init();
  /*??????USB??????????????????*/
  Reset_Usb();

#ifndef ORTUR_CNC_MODE
  Leds_Power(0); //??????????????????

  //??????,??????????????????
  PowerOpen_Loop();
  //NOTE:????????????????????????????????????,???????????????
  Leds_Power(1); //??????????????????
#endif
  //?????????????????????
  PowerLed_On();
  /*??????????????????1*/
  MX_TIM_Init();
  /*USB?????????*/
  MX_USB_DEVICE_Init();

#ifndef DEBUG
  IWDG_Init();
#endif
  /*?????????????????????*/
  MX_NVIC_Init();
  /*??????????????????*/
  timing_init();
  /*flash??????EEprom???????????????????????????*/
  eeprom_init();
  /*???????????????*/
  serial_init();   // Setup serial baud rate and interrupts
  settings_init(); // Load Grbl settings from EEPROM
  stepper_init();  // Configure stepper pins and interrupt timers
  system_init();   // Configure pinout pins and pin-change interrupt
  memset(sys_position,0,sizeof(sys_position)); // Clear machine position.

  // Initialize system state.
  #ifdef FORCE_INITIALIZATION_ALARM
    // Force Grbl into an ALARM state upon a power-cycle or hard reset.
    sys.state = STATE_ALARM;
  #else
    sys.state = STATE_IDLE;
  #endif

  // Check for power-up and set system alarm if homing is enabled to force homing cycle
  // by setting Grbl's alarm state. Alarm locks out all g-code commands, including the
  // startup scripts, but allows access to settings and internal commands. Only a homing
  // cycle '$H' or kill alarm locks '$X' will disable the alarm.
  // NOTE: The startup script will run after successful completion of the homing cycle, but
  // not after disabling the alarm locks. Prevents motion startup blocks from crashing into
  // things uncontrollably. Very bad.
  #ifdef HOMING_INIT_LOCK
    if (bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) { sys.state = STATE_ALARM; }
  #endif

  /* Infinite loop */
  while (1)
  {
    // Reset system variables.
    uint8_t prior_state = sys.state;
    memset(&sys, 0, sizeof(system_t)); // Clear system struct variable.
    sys.state = prior_state;
    sys.f_override = DEFAULT_FEED_OVERRIDE;  // Set to 100%
    sys.r_override = DEFAULT_RAPID_OVERRIDE; // Set to 100%
    sys.spindle_speed_ovr = DEFAULT_SPINDLE_SPEED_OVERRIDE; // Set to 100%
      memset(sys_probe_position,0,sizeof(sys_probe_position)); // Clear probe position.
    sys_probe_state = 0;
    sys_rt_exec_state = 0;
    sys_rt_exec_alarm = 0;
    sys_rt_exec_motion_override = 0;
    sys_rt_exec_accessory_override = 0;

    // Reset Grbl primary systems.
    serial_reset_read_buffer(); // Clear serial read buffer
    gc_init(); // Set g-code parser to default state
    spindle_init();
    coolant_init();
    limits_init();
    probe_init();
    inoutputs_init();

    plan_reset(); // Clear block buffer and planner variables
    st_reset(); // Clear stepper subsystem variables.

    // Sync cleared gcode and planner positions to current system position.
    plan_sync_position();
    gc_sync_position();

    // Print welcome message. Indicates an initialization has occured at power-up or with a reset.
    report_init_message();

    // Start Grbl main loop. Processes program inputs and executes them.
    protocol_main_loop();

  }

}

/**
  * @brief System Clock Configuration ??????72Mhz
  * @retval None
  */

void SystemClock_Config(void)
{
	uint32_t temp=0;
	uint8_t PLL=9;

 	RCC->CR|=0x00010000;  //????????????????????????HSEON
	while(!(RCC->CR>>17));//????????????????????????
	RCC->CFGR=0X00000400; //APB1=DIV2;APB2=DIV1;AHB=DIV1;
	PLL-=2;				  //??????2????????????????????????2??????????????????0??????2???
	RCC->CFGR|=PLL<<18;   //??????PLL??? 2~16
	RCC->CFGR|=1<<16;	  //PLLSRC ON
	FLASH->ACR|=0x32;	  //FLASH 2???????????????
	RCC->CR|=0x01000000;  //PLLON
	while(!(RCC->CR>>25));//??????PLL??????
	RCC->CFGR|=0x00000002;//PLL??????????????????
	while(temp!=0x02)     //??????PLL??????????????????????????????
	{
		temp=RCC->CFGR>>2;
		temp&=0x03;
	}
}

/**
  * @brief NVIC Configuration.???????????????
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* EXTI0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI0_IRQn, LIMIT_SWITCH_PR, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  /* EXTI1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(EXTI1_IRQn, LIMIT_SWITCH_PR, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
	while(1);
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
