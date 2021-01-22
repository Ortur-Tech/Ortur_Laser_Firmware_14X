/*
  serial.c - Low level functions for sending and recieving bytes via the serial port
  Part of Grbl

  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

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

#include "grbl.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"
#include "gpio.h"

#ifdef STM32
  #ifdef STM32F1
		#define pUSART ((USART_TypeDef *) USART1_BASE)
  #endif
  #ifdef STM32F4
		#define pUSART ((USART_TypeDef *) USART1_BASE)
  #endif

  #define RX_RING_BUFFER (RX_BUFFER_SIZE)
  #define TX_RING_BUFFER (TX_BUFFER_SIZE)
#elif ATMEGA328P
  #define RX_RING_BUFFER (RX_BUFFER_SIZE+1)
  #define TX_RING_BUFFER (TX_BUFFER_SIZE+1)
#endif

uint8_t serial_rx_buffer[RX_RING_BUFFER];
uint8_t serial_rx_buffer_head = 0;
uint8_t serial_rx_buffer_tail = 0;

/*虚拟usb串口接收缓冲区*/
uint8_t usb_serial_rx_buffer[RX_RING_BUFFER];
uint8_t usb_serial_rx_buffer_head = 0;
uint8_t usb_serial_rx_buffer_tail = 0;

uint8_t serial_tx_buffer[TX_RING_BUFFER];
uint8_t serial_tx_buffer_head = 0;
volatile uint8_t serial_tx_buffer_tail = 0;


// Returns the number of bytes available in the RX serial buffer.
uint8_t serial_get_rx_buffer_available()
{
	uint8_t rtail = usb_serial_rx_buffer_tail; // Copy to limit multiple calls to volatile
	if (usb_serial_rx_buffer_head >= rtail) { return(RX_BUFFER_SIZE - (usb_serial_rx_buffer_head-rtail)); }
	return((rtail-usb_serial_rx_buffer_head-1));
//  uint8_t rtail = serial_rx_buffer_tail; // Copy to limit multiple calls to volatile
//  if (serial_rx_buffer_head >= rtail) { return(RX_BUFFER_SIZE - (serial_rx_buffer_head-rtail)); }
//  return((rtail-serial_rx_buffer_head-1));
}


// Returns the number of bytes used in the RX serial buffer.
// NOTE: Deprecated. Not used unless classic status reports are enabled in config.h.
uint8_t serial_get_rx_buffer_count()
{
	uint8_t rtail = usb_serial_rx_buffer_tail; // Copy to limit multiple calls to volatile
	if (usb_serial_rx_buffer_head >= rtail) { return(usb_serial_rx_buffer_head-rtail); }
	return (RX_BUFFER_SIZE - (rtail-usb_serial_rx_buffer_head));
//  uint8_t rtail = serial_rx_buffer_tail; // Copy to limit multiple calls to volatile
//  if (serial_rx_buffer_head >= rtail) { return(serial_rx_buffer_head-rtail); }
//  return (RX_BUFFER_SIZE - (rtail-serial_rx_buffer_head));
}


// Returns the number of bytes used in the TX serial buffer.
// NOTE: Not used except for debugging and ensuring no TX bottlenecks.
uint8_t serial_get_tx_buffer_count()
{

	uint8_t ttail = serial_tx_buffer_tail; // Copy to limit multiple calls to volatile
	if (serial_tx_buffer_head >= ttail) { return(serial_tx_buffer_head-ttail); }
	return (TX_RING_BUFFER - (ttail-serial_tx_buffer_head));
}

void serial_init()
{
#ifdef ATMEGA328P
  // Set baud rate
  #if BAUD_RATE < 57600
    uint16_t UBRR0_value = ((F_CPU / (8L * BAUD_RATE)) - 1)/2 ;
    UCSR0A &= ~(1 << U2X0); // baud doubler off  - Only needed on Uno XXX
  #else
    uint16_t UBRR0_value = ((F_CPU / (4L * BAUD_RATE)) - 1)/2;
    UCSR0A |= (1 << U2X0);  // baud doubler on for high baud rates, i.e. 115200
  #endif
  UBRR0H = UBRR0_value >> 8;
  UBRR0L = UBRR0_value;

  // enable rx, tx, and interrupt on complete reception of a byte
  UCSR0B |= (1<<RXEN0 | 1<<TXEN0 | 1<<RXCIE0);

  // defaults to 8-bit, no parity, 1 stop bit
#endif
}
#define USBCDC 1  //USB CDC VCP
#define HWUART 2  //HW UART

uint8_t last_steam = USBCDC;
uint8_t steamSwitchAble = 0;

/**
 * @brief usb_serial_write 写数据到USB
 * @param data
 */
void usb_serial_write(uint8_t data)
{
	//usb串口未连接时,不发送任何数据
	if(isUsbCDCConnected())
	{
		__disable_irq();
		// Store data and advance head
		serial_tx_buffer[serial_tx_buffer_head++] = data;
		__enable_irq();
		if((serial_tx_buffer[serial_tx_buffer_head-1]=='\n')||(TX_RING_BUFFER<=serial_tx_buffer_head))
		{
			while(USBD_BUSY==CDC_Transmit_FS(serial_tx_buffer,serial_tx_buffer_head))
			{
				if(!isUsbCDCConnected())
				{
					break;
				}
			}
            /*64字节的整数倍需要外加一个空trans否则主机收不到这包数据*/
			if((serial_tx_buffer_head%64)==0)
			{
				CDC_Transmit_FS(serial_tx_buffer,0);
			}
			serial_tx_buffer_head=0;
		}
	}
}
// Writes one byte to the TX serial buffer. Called by main program.
/**
 * @brief serial_write  写数据到上次使用的串口
 * @param data
 */
void serial_write(uint8_t data)
{
#ifdef STM32
#ifdef USE_USB
#if USE_DOUBLE_SERIAL
	if(last_steam == USBCDC)
	{
		usb_serial_write(data);
	}
	else
	{
		uart_sendch(data);
	}
#else
	usb_serial_write(data);
#endif
#else
	uart_sendch(data);
#endif
#elif ATMEGA328P

  // Calculate next head
  uint8_t next_head = serial_tx_buffer_head + 1;
  if (next_head == TX_RING_BUFFER) { next_head = 0; }

  // Wait until there is space in the buffer
  while (next_head == serial_tx_buffer_tail) {
    // TODO: Restructure st_prep_buffer() calls to be executed here during a long print.
    if (sys_rt_exec_state & EXEC_RESET) { return; } // Only check for abort to avoid an endless loop.
  }

  // Store data and advance head
  serial_tx_buffer[serial_tx_buffer_head] = data;
  serial_tx_buffer_head = next_head;

  // Enable Data Register Empty Interrupt to make sure tx-streaming is running
  UCSR0B |=  (1 << UDRIE0);
#endif
}
/**
 * @brief serial_write_all  写数据到两个串口
 * @param data
 */
void serial_write_all(uint8_t data)
{
	usb_serial_write(data);
#if USE_DOUBLE_SERIAL
	uart_sendch(data);
#endif
}
#ifdef ATMEGA328P
// Data Register Empty Interrupt handler
ISR(SERIAL_UDRE)
{
  uint8_t tail = serial_tx_buffer_tail; // Temporary serial_tx_buffer_tail (to optimize for volatile)

  // Send a byte from the buffer
  UDR0 = serial_tx_buffer[tail];

  // Update tail position
  tail++;
  if (tail == TX_RING_BUFFER) { tail = 0; }

  serial_tx_buffer_tail = tail;

  // Turn off Data Register Empty Interrupt to stop tx-streaming if this concludes the transfer
  if (tail == serial_tx_buffer_head) { UCSR0B &= ~(1 << UDRIE0); }
}
#endif
/**
 * @brief usbGetC
 * @return 从缓冲区获取的数据
 */
char usbGetC(void)
{
	  uint8_t tail = usb_serial_rx_buffer_tail; // Temporary serial_rx_buffer_tail (to optimize for volatile)
	  if (usb_serial_rx_buffer_head == tail) {
	    return SERIAL_NO_DATA;
	  } else {
	    uint8_t data = usb_serial_rx_buffer[tail];

	    tail++;
	    if (tail == RX_RING_BUFFER) { tail = 0; }
	    usb_serial_rx_buffer_tail = tail;

	    return data;
	  }
}
/**
 * @brief serialGetC
 * @return 从缓冲区获取的数据
 */
char serialGetC(void)
{
	  uint8_t tail = serial_rx_buffer_tail; // Temporary serial_rx_buffer_tail (to optimize for volatile)
	  if (serial_rx_buffer_head == tail) {
	    return SERIAL_NO_DATA;
	  } else {
	    uint8_t data = serial_rx_buffer[tail];

	    tail++;
	    if (tail == RX_RING_BUFFER) { tail = 0; }
	    serial_rx_buffer_tail = tail;

	    return data;
	  }
}

/**
 * @brief Fetches the first byte in the serial read buffer.
 * @return
 */
uint8_t serial_read()
{
#if USE_DOUBLE_SERIAL
	int16_t c = SERIAL_NO_DATA;

	if(isUsbCDCConnected())
	{
		if(last_steam == USBCDC || steamSwitchAble )
		{
			c = usbGetC();
			if(last_steam != USBCDC && (c != SERIAL_NO_DATA) )
			{
				last_steam = USBCDC;
			}
		}
	}
	else if(last_steam == USBCDC) //在CDC已经断开连接的情况下,应该将steam指向转到 硬件串口 HWUART
	{
		steamSwitchAble = true;
		last_steam = HWUART;
		c = '\n'; //强行补换行符防止命令被截断,或者污染后续的命令字符串
	}

	if(c == SERIAL_NO_DATA )
	{
		if(last_steam == HWUART || steamSwitchAble )
		{
			c = serialGetC();
			if(last_steam != HWUART && (c != SERIAL_NO_DATA))
			{
				last_steam = HWUART;
			}
		}
	}
	return c;
#else
	int16_t c = SERIAL_NO_DATA;
	c = usbGetC();
	last_steam = USBCDC;
	steamSwitchAble = (c == SERIAL_NO_DATA);
	return c;
#endif
}

#ifdef STM32
/**
 * @brief 串口接收中断处理函数
 * @param data 从串口传过来的数据
 * @param steam USBCDC：虚拟串口 其他：硬件串口
 */
void HandleUartIT(uint8_t data,uint8_t steam)
{
	uint8_t next_head;
  // Pick off realtime command characters directly from the serial stream. These characters are
  // not passed into the main buffer, but these set system state flag bits for realtime execution.
	switch (data) {
    case CMD_RESET:         mc_reset(); break; // Call motion control reset routine.
    case CMD_STATUS_REPORT: system_set_exec_state_flag(EXEC_STATUS_REPORT); break; // Set as true
    case CMD_CYCLE_START:   system_set_exec_state_flag(EXEC_CYCLE_START); break; // Set as true
    case CMD_FEED_HOLD:     system_set_exec_state_flag(EXEC_FEED_HOLD); break; // Set as true
    default :
      if (data > 0x7F) { // Real-time control characters are extended ACSII only.
        switch(data) {
          case CMD_SAFETY_DOOR:   system_set_exec_state_flag(EXEC_SAFETY_DOOR); break; // Set as true
          case CMD_JOG_CANCEL:
            if (sys.state & STATE_JOG) { // Block all other states from invoking motion cancel.
              system_set_exec_state_flag(EXEC_MOTION_CANCEL);
            }
            break;
          #ifdef DEBUG
          case CMD_DEBUG_REPORT: {
            #ifdef ATMEGA328P
            uint8_t sreg = SREG; cli(); bit_true(sys_rt_exec_debug,EXEC_DEBUG_REPORT); SREG = sreg;
            #endif
            } break;
          #endif
          case CMD_FEED_OVR_RESET: system_set_exec_motion_override_flag(EXEC_FEED_OVR_RESET); break;
          case CMD_FEED_OVR_COARSE_PLUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_COARSE_PLUS); break;
          case CMD_FEED_OVR_COARSE_MINUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_COARSE_MINUS); break;
          case CMD_FEED_OVR_FINE_PLUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_FINE_PLUS); break;
          case CMD_FEED_OVR_FINE_MINUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_FINE_MINUS); break;
          case CMD_RAPID_OVR_RESET: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_RESET); break;
          case CMD_RAPID_OVR_MEDIUM: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_MEDIUM); break;
          case CMD_RAPID_OVR_LOW: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_LOW); break;
          case CMD_SPINDLE_OVR_RESET: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_RESET); break;
          case CMD_SPINDLE_OVR_COARSE_PLUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_COARSE_PLUS); break;
          case CMD_SPINDLE_OVR_COARSE_MINUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_COARSE_MINUS); break;
          case CMD_SPINDLE_OVR_FINE_PLUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_FINE_PLUS); break;
          case CMD_SPINDLE_OVR_FINE_MINUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_FINE_MINUS); break;
          case CMD_SPINDLE_OVR_STOP: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_STOP); break;
          case CMD_COOLANT_FLOOD_OVR_TOGGLE: system_set_exec_accessory_override_flag(EXEC_COOLANT_FLOOD_OVR_TOGGLE); break;
          #ifdef ENABLE_M7
            case CMD_COOLANT_MIST_OVR_TOGGLE: system_set_exec_accessory_override_flag(EXEC_COOLANT_MIST_OVR_TOGGLE); break;
          #endif
        }
        // Throw away any unfound extended-ASCII character by not passing it to the serial buffer.
      }
      else
      {
#if USE_DOUBLE_SERIAL
    	if(steam==USBCDC)
    	{
			// Write character to buffer
			next_head = usb_serial_rx_buffer_head + 1;
			if (next_head == RX_RING_BUFFER) { next_head = 0; }

			// Write data to buffer unless it is full.
			if (next_head != usb_serial_rx_buffer_tail)
			{
				usb_serial_rx_buffer[usb_serial_rx_buffer_head] = data;
				usb_serial_rx_buffer_head = next_head;
			}
    	}
    	else
    	{

    		// Write character to buffer
			next_head = serial_rx_buffer_head + 1;
			if (next_head == RX_RING_BUFFER) { next_head = 0; }

			// Write data to buffer unless it is full.
			if (next_head != serial_rx_buffer_tail)
			{
			  serial_rx_buffer[serial_rx_buffer_head] = data;
			  serial_rx_buffer_head = next_head;
			}
    	}
#else
    	// Write character to buffer
		next_head = usb_serial_rx_buffer_head + 1;
		if (next_head == RX_RING_BUFFER) { next_head = 0; }

		// Write data to buffer unless it is full.
		if (next_head != usb_serial_rx_buffer_tail)
		{
			usb_serial_rx_buffer[usb_serial_rx_buffer_head] = data;
			usb_serial_rx_buffer_head = next_head;
		}
#endif
      }
  }
}
#endif

/*当3秒内接收到不完整命令时允许串口接收切换,并将不完整的命令补'\n'*/
#define SERIAL_SWITCH_TIME 3000

uint32_t last_steam_use_time=0;
/**
 * @brief USART1_IRQHandler
 */
void USART1_IRQHandler (void)
{
#if USE_DOUBLE_SERIAL
	uint8_t data=0;

	if(USART1->SR &(1<<5))  //接收到数据
	{
		data = USART1->DR;
		HandleUartIT(data,HWUART);
	}
#endif
}

#ifdef USE_USB
/**
 * @brief OnUsbDataRx USB数据接收
 * @param dataIn 数据指针
 * @param length 数据长度
 */
void OnUsbDataRx(uint8_t* dataIn, uint8_t length)
{
    uint8_t data;

	// Write data to buffer unless it is full.
	while (length != 0)
	{
		data = *dataIn ++;
		HandleUartIT(data,USBCDC);
		length--;
   }
}
#endif

#ifdef ATMEGA328P

ISR(SERIAL_RX)
{
  uint8_t data = UDR0;
  uint8_t next_head;

  // Pick off realtime command characters directly from the serial stream. These characters are
  // not passed into the main buffer, but these set system state flag bits for realtime execution.
  switch (data) {
    case CMD_RESET:         mc_reset(); break; // Call motion control reset routine.
    case CMD_STATUS_REPORT: system_set_exec_state_flag(EXEC_STATUS_REPORT); break; // Set as true
    case CMD_CYCLE_START:   system_set_exec_state_flag(EXEC_CYCLE_START); break; // Set as true
    case CMD_FEED_HOLD:     system_set_exec_state_flag(EXEC_FEED_HOLD); break; // Set as true
    default :
      if (data > 0x7F) { // Real-time control characters are extended ACSII only.
        switch(data) {
          case CMD_SAFETY_DOOR:   system_set_exec_state_flag(EXEC_SAFETY_DOOR); break; // Set as true
          case CMD_JOG_CANCEL:   
            if (sys.state & STATE_JOG) { // Block all other states from invoking motion cancel.
              system_set_exec_state_flag(EXEC_MOTION_CANCEL); 
            }
            break; 
          #ifdef DEBUG
            case CMD_DEBUG_REPORT: {uint8_t sreg = SREG; cli(); bit_true(sys_rt_exec_debug,EXEC_DEBUG_REPORT); SREG = sreg;} break;
          #endif
          case CMD_FEED_OVR_RESET: system_set_exec_motion_override_flag(EXEC_FEED_OVR_RESET); break;
          case CMD_FEED_OVR_COARSE_PLUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_COARSE_PLUS); break;
          case CMD_FEED_OVR_COARSE_MINUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_COARSE_MINUS); break;
          case CMD_FEED_OVR_FINE_PLUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_FINE_PLUS); break;
          case CMD_FEED_OVR_FINE_MINUS: system_set_exec_motion_override_flag(EXEC_FEED_OVR_FINE_MINUS); break;
          case CMD_RAPID_OVR_RESET: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_RESET); break;
          case CMD_RAPID_OVR_MEDIUM: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_MEDIUM); break;
          case CMD_RAPID_OVR_LOW: system_set_exec_motion_override_flag(EXEC_RAPID_OVR_LOW); break;
          case CMD_SPINDLE_OVR_RESET: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_RESET); break;
          case CMD_SPINDLE_OVR_COARSE_PLUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_COARSE_PLUS); break;
          case CMD_SPINDLE_OVR_COARSE_MINUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_COARSE_MINUS); break;
          case CMD_SPINDLE_OVR_FINE_PLUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_FINE_PLUS); break;
          case CMD_SPINDLE_OVR_FINE_MINUS: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_FINE_MINUS); break;
          case CMD_SPINDLE_OVR_STOP: system_set_exec_accessory_override_flag(EXEC_SPINDLE_OVR_STOP); break;
          case CMD_COOLANT_FLOOD_OVR_TOGGLE: system_set_exec_accessory_override_flag(EXEC_COOLANT_FLOOD_OVR_TOGGLE); break;
          #ifdef ENABLE_M7
            case CMD_COOLANT_MIST_OVR_TOGGLE: system_set_exec_accessory_override_flag(EXEC_COOLANT_MIST_OVR_TOGGLE); break;
          #endif
        }
        // Throw away any unfound extended-ASCII character by not passing it to the serial buffer.
      } else { // Write character to buffer
        next_head = serial_rx_buffer_head + 1;
        if (next_head == RX_RING_BUFFER) { next_head = 0; }

        // Write data to buffer unless it is full.
        if (next_head != serial_rx_buffer_tail) {
          serial_rx_buffer[serial_rx_buffer_head] = data;
          serial_rx_buffer_head = next_head;
        }
      }
  }
}
#endif

